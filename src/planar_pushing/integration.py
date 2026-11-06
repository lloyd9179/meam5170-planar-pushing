"""Explicit MuJoCo / ideal-point-contact bridge; simulator core is unchanged."""
import numpy as np
import mujoco
from .env import EnvConfig, PlanarPushingEnv
from .models.contact_geometry import rotation


def wrap_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


def model_state(env, model, face, previous_theta=None):
    state = env.get_state()
    theta = state.theta if previous_theta is None else previous_theta + wrap_angle(state.theta-previous_theta)
    center_body = rotation(theta).T @ (np.asarray(state.pusher_position)-state.slider_position[:2])
    r0, n, t, _, _ = model.contact_geometry(0, face)
    s = float(t @ (center_body-r0))
    return np.array([*state.slider_position[:2], theta, s])


def world_velocity(model, x, u, face):
    """Point-velocity approximation: R(theta) F u, no hidden position servo.

    Finite-radius center motion differs from the geometric fixed-s trajectory
    by -rho*omega*R*J*n. We do NOT add this tangential slip term silently.
    Actual s drift and material slip are measured by contact_diagnostics().
    """
    _, _, _, F, _ = model.contact_geometry(x[3], face)
    return rotation(x[2]) @ F @ np.asarray(u)


def local_velocity(model, x, velocity, face):
    _, _, _, F, _ = model.contact_geometry(x[3], face)
    return F.T @ rotation(x[2]).T @ velocity


def contact_diagnostics(env, model, x, face, commanded_world):
    """Read actual contact positions/forces, plus planar material slip estimate.

    mocap mj_objectVelocity is not the commanded velocity; use applied center
    velocity and the slider's measured world twist, including moment arm.
    """
    r, n, t, _, _ = model.contact_geometry(x[3], face)
    state = env.get_state(); R = rotation(x[2])
    pc = R.T @ (np.asarray(state.pusher_position)-x[:2])
    gap = float(-(pc-r)@n-env.config.pusher_radius)
    positions, forces = [], []
    for i in range(env.data.ncon):
        c = env.data.contact[i]
        names = {env.model.geom(c.geom1).name, env.model.geom(c.geom2).name}
        if names == {'pusher_geom', 'slider_geom'}:
            f = np.zeros(6); mujoco.mj_contactForce(env.model, env.data, i, f)
            if f[0] > 1e-5:
                positions.append(c.pos[:2].copy()); forces.append(float(f[0]))
    if not positions:
        return dict(contact=False, normal_force=0., gap=gap, contact_point_error=np.nan,
                    contact_s=np.nan, slip_speed=np.nan)
    point = np.average(positions, axis=0, weights=forces)
    local_point = R.T @ (point-x[:2])
    expected = x[:2]+R@r
    arm = np.r_[point, state.slider_position[2]]-state.slider_position
    v_contact = np.asarray(state.slider_linear_velocity)+np.cross(state.slider_angular_velocity, arm)
    pusher_velocity=np.zeros(6)
    mujoco.mj_objectVelocity(env.model,env.data,mujoco.mjtObj.mjOBJ_BODY,
                            env.model.body('pusher').id,pusher_velocity,0)
    # The commanded-velocity estimate for mocap is deliberately labeled in reports.
    actual_velocity=(np.asarray(commanded_world) if env.config.pusher_mode=='mocap'
                     else pusher_velocity[3:5])
    slip = float((R@t) @ (actual_velocity-v_contact[:2]))
    return dict(contact=True, normal_force=sum(forces), gap=gap,
                contact_point_error=float(np.linalg.norm(point-expected)),
                contact_s=float(t@(local_point-r)), slip_speed=slip)


def prepare_contact(model, face='left', s=0., *, dt=.02, goal=(.04,0,0), pusher_mode='velocity_servo'):
    """Set initial conditions once, settle, then build contact using real pushes."""
    cfg = model.config
    env = PlanarPushingEnv(EnvConfig(slider_size=(cfg.slider_length,cfg.slider_width,.02),
                                    dt_control=dt, mu_pusher=cfg.mu_pusher,
                                    mu_ground=cfg.mu_ground_nominal, goal_pose=tuple(goal),
                                    pusher_mode=pusher_mode, dt_physics=.0002 if pusher_mode=='velocity_servo' else .001,
                                    pusher_servo_gain=500.))
    r,n,t,F,A = model.contact_geometry(s, face)
    env.reset(0, pusher_position=r-env.config.pusher_radius*n)
    for _ in range(round(.2/dt)):
        env.step([0,0])
    for _ in range(round(.6/dt)):
        x = model_state(env,model,face)
        cmd = world_velocity(model,x,[.002,0],face)
        env.step(cmd)
        x = model_state(env,model,face)
        if contact_diagnostics(env,model,x,face,cmd)['normal_force'] > .03:
            return env,x
    raise RuntimeError('Could not establish loaded contact by gentle physical approach')
