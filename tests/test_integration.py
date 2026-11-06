import numpy as np
import pytest
import mujoco
from planar_pushing import EnvConfig,PlanarPushingEnv
from planar_pushing.models import QuasiStaticPushingModel,ContactFace
from planar_pushing.integration import prepare_contact,model_state,contact_diagnostics,world_velocity
from planar_pushing.mpc_demo import run_closed_loop


def test_servo_velocity_is_visible_to_contact_solver():
    env=PlanarPushingEnv(EnvConfig(pusher_mode='velocity_servo',dt_physics=.0002,pusher_servo_gain=500))
    initial=env.reset(pusher_position=(-.2,-.2))
    u=np.array([.01,.008]);state,info=env.step(u)
    v=np.zeros(6)
    mujoco.mj_objectVelocity(env.model,env.data,mujoco.mjtObj.mjOBJ_BODY,env.model.body('pusher').id,v,0)
    np.testing.assert_allclose(v[3:5],u,atol=1e-8)
    np.testing.assert_allclose(np.subtract(state.pusher_position,initial.pusher_position),u*.02,atol=3e-6)
    assert np.linalg.norm(state.slider_position[:2])<1e-10
    env.reset(pusher_position=(-.2,-.2));assert np.linalg.norm(env.data.qvel)==0
    np.testing.assert_allclose(env.get_state().pusher_position,[-.2,-.2])


@pytest.mark.parametrize('face',list(ContactFace))
def test_actual_contact_geometry_all_faces(face):
    m=QuasiStaticPushingModel();env,x=prepare_contact(m,face,.01)
    for _ in range(10):
        cmd=world_velocity(m,x,[.005,0],face);env.step(cmd);x=model_state(env,m,face,x[2])
    d=contact_diagnostics(env,m,x,face,cmd)
    assert d['contact'] and d['normal_force']>.01
    assert d['contact_point_error']<.0005
    assert abs(x[3]-.01)<.001
    _,_,_,F,_=m.contact_geometry(x[3],face)
    assert (cmd @ (np.array([[np.cos(x[2]),-np.sin(x[2])],[np.sin(x[2]),np.cos(x[2])]])@F[:,0]))>0


@pytest.mark.parametrize('backend',['model','mujoco'])
@pytest.mark.parametrize('task',['translation','rotation'])
def test_closed_loop_tracking_regression(backend,task):
    data=run_closed_loop(task,backend,duration=4.)
    s=data['summary']
    assert s['termination']=='duration_complete'
    assert s['fallbacks']==0 and s['sticking_infeasible_commands']==0
    assert s['final_position_error_m']<.2*s['initial_position_error_m']
    if task=='rotation': assert s['final_yaw_error_rad']<.1*s['initial_yaw_error_rad']
    assert s['max_constraint_violation']<5e-6
