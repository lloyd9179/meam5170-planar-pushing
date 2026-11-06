"""MuJoCo/OSQP dependencies stop here; policies and supervisor use contracts only."""
import numpy as np
import mujoco
from ..models.contact_geometry import rotation
from .contracts import WorldObservation, ContactSample, PushResult
from .selection import angle_error


class MuJoCoObservationAdapter:
    def __init__(self, env):
        self.env = env

    def observe(self):
        env = self.env
        state = env.get_state()
        pusher_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(env.model, env.data, mujoco.mjtObj.mjOBJ_BODY,
                                env.model.body('pusher').id, pusher_velocity, 0)
        contacts = []
        for i in range(env.data.ncon):
            contact = env.data.contact[i]
            names = {env.model.geom(contact.geom1).name, env.model.geom(contact.geom2).name}
            if names == {'pusher_geom', 'slider_geom'}:
                force = np.zeros(6)
                mujoco.mj_contactForce(env.model, env.data, i, force)
                contacts.append(ContactSample(tuple(contact.pos[:2]), max(0., float(force[0]))))
        return WorldObservation(state.slider_pose, state.pusher_position,
                                (*state.slider_linear_velocity[:2], state.slider_angular_velocity[2]),
                                tuple(pusher_velocity[3:5]), state.time, tuple(contacts),
                                env.config.pusher_mode == 'velocity_servo')


class LocalMPCPushAdapter:
    def __init__(self, mpc):
        self.mpc = mpc
        self.reset()

    def reset(self):
        self.mpc.reset()
        self.target = None
        self.theta = None

    def activate(self, target):
        # Reset even if face is unchanged: a different selected s is a new contact task.
        self.reset()
        self.target = target

    def compute(self, observation, contact, goal_pose):
        if self.target is None or self.target.face != contact.face:
            return PushResult((0., 0.), False, 'contact_not_activated')
        if not observation.pusher_velocity_valid:
            return PushResult((0., 0.), False, 'execution_velocity_unavailable')
        lo, hi = self.mpc.model.contact_bounds(contact.face)
        if not contact.in_bounds or not lo <= contact.measured_s <= hi:
            return PushResult((0., 0.), False, 'measured_contact_out_of_model_bounds')
        raw_theta = observation.slider_pose[2]
        self.theta = raw_theta if self.theta is None else self.theta+angle_error(raw_theta, self.theta)
        x = np.array([*observation.slider_pose[:2], self.theta, contact.measured_s])
        result = self.mpc.solve(x, goal_pose, contact.face)
        command = np.zeros(2)
        if result.feasible:
            _, _, _, F, _ = self.mpc.model.contact_geometry(x[3], contact.face)
            command = rotation(x[2]) @ F @ result.control
        return PushResult(tuple(command), result.feasible, result.status, result.solve_time,
                          result.iterations, result.objective, result.constraint_violation)
