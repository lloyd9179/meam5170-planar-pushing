import numpy as np
import pytest
from planar_pushing.models import ModelConfig,QuasiStaticPushingModel
from planar_pushing.control import LocalQPMPC,MPCConfig


@pytest.mark.parametrize('H',[1,5,15,23])
def test_dimensions_psd_affine_and_objective(H):
    m=QuasiStaticPushingModel();c=MPCConfig(horizon=H);mpc=LocalQPMPC(m,c)
    x=np.array([.1,-.02,.7,.02]);goal=np.array([.15,-.03,.3])
    u=np.tile([.012,.001],(H,1));previous=np.array([.002,-.001])
    qp=mpc.build_qp(x,goal,'left',u,previous)
    assert qp.P.shape==(6*H,6*H) and qp.A.shape==(11*H,6*H)
    np.testing.assert_allclose(qp.P.toarray(),qp.P.toarray().T,atol=1e-13)
    assert np.linalg.eigvalsh(qp.P.toarray()).min()>-1e-10
    z=np.r_[qp.nominal_states[1:].ravel(),u.ravel()]
    np.testing.assert_allclose((qp.A@z)[qp.rows['dynamics']],qp.upper[qp.rows['dynamics']],atol=1e-12)
    # Full direct objective checks 1/2, terminal Qf, increment cross terms and constants.
    states=qp.nominal_states;Q,Qf,R,Rd=[np.array(getattr(c,k)) for k in ('Q','Qf','R','Rd')]
    cost=0.;prev=previous
    for k in range(H):
        e=states[k]-qp.goal;du=u[k]-prev
        cost+=e@Q@e+u[k]@R@u[k]+du@Rd@du;prev=u[k]
    e=states[-1]-qp.goal;cost+=e@Qf@e+c.regularization*(z@z)
    assert .5*z@(qp.P@z)+qp.q@z+qp.constant == pytest.approx(cost,abs=1e-12)
    # Changing yaw reference by full turns must leave the QP unchanged.
    equivalent=mpc.build_qp(x,goal+[0,0,2*np.pi],'left',u,previous)
    np.testing.assert_allclose(qp.q,equivalent.q,atol=1e-12)


def test_one_step_integrator_analytic_solution():
    # Center, theta=0, s=0, one step: x1=dt*vn; y/yaw targets zero.
    c=MPCConfig(horizon=1,dt=.02);mpc=LocalQPMPC(QuasiStaticPushingModel(),c)
    result=mpc.solve([0,0,0,0],[.0001,0,0],'left')
    expected=(80*.02*.0001)/(80*.02**2+.1+.2+c.regularization*(1+.02**2))
    assert result.feasible
    assert result.control[0]==pytest.approx(expected,abs=2e-7)
    assert abs(result.control[1])<1e-8


def test_constraints_warmstart_and_reset():
    m=QuasiStaticPushingModel();mpc=LocalQPMPC(m)
    x=np.array([0,0,.1,.02]);goal=[.04,-.01,-.1]
    first=mpc.solve(x,goal,'left');assert first.feasible and not first.warm_started
    second=mpc.solve(m.discrete_dynamics(x,first.control,.02,'left'),goal,'left')
    assert second.feasible and second.warm_started
    np.testing.assert_allclose(mpc.last_problem.nominal_controls[:-1],first.predicted_controls[1:])
    assert second.constraint_violation<5e-6
    for state,u in zip(second.predicted_states,second.predicted_controls):
        assert m.is_sticking_feasible(state,u,'left',5e-6)
        assert u[0]>=-5e-6 and np.max(abs(u))<=.025+5e-6
    mpc.reset();assert not mpc.solve(x,goal,'left').warm_started


def test_real_infeasible_and_iteration_failure():
    m=QuasiStaticPushingModel(ModelConfig(mu_pusher=0))
    mpc=LocalQPMPC(m,MPCConfig(velocity_lower=(.01,0),velocity_upper=(.02,0)))
    result=mpc.solve([0,0,0,.02],[.02,0,0],'left')
    assert not result.feasible and result.fallback_used
    assert 'infeasible' in result.status
    np.testing.assert_array_equal(result.control,[0,0])
    mpc=LocalQPMPC(QuasiStaticPushingModel(),MPCConfig(max_iter=1))
    result=mpc.solve([0,0,.1,.02],[.1,-.02,-.2],'left')
    assert not result.feasible and result.fallback_used
    assert 'maximum iterations' in result.status
    assert not mpc.solve([0,0,0,.039],[.1,0,0],'left').feasible


def test_prediction_model_closed_loop():
    m=QuasiStaticPushingModel();mpc=LocalQPMPC(m)
    x=np.array([0.,0,0,0]);goal=np.array([.04,0,0]);initial=np.linalg.norm(x[:2]-goal[:2])
    for _ in range(200):
        r=mpc.solve(x,goal,'left');assert r.feasible
        x=m.discrete_dynamics(x,r.control,.02,'left')
    assert np.linalg.norm(x[:2]-goal[:2])<initial*.03


@pytest.mark.parametrize('kw',[{'horizon':0},{'dt':0},{'Q':[-1,0,0,0]},{'velocity_lower':(-1,-1)}])
def test_invalid_config(kw):
    with pytest.raises(ValueError): MPCConfig(**kw)
