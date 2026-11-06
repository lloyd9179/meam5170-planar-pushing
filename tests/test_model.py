import numpy as np
import pytest
from planar_pushing.models import ContactFace, ModelConfig, QuasiStaticPushingModel


@pytest.mark.parametrize('face', list(ContactFace))
def test_geometry_and_center(face):
    m = QuasiStaticPushingModel()
    r, n, t, F, A = m.contact_geometry(0, face)
    np.testing.assert_allclose(F.T@F, np.eye(2))
    assert np.linalg.det(F) == pytest.approx(1)
    assert r@n < 0
    r2, *_ = m.contact_geometry(.01, face)
    np.testing.assert_allclose(r2-r, .01*t)
    G, D = m.matrices(0, face)
    np.testing.assert_allclose(A@G, F, atol=1e-14)
    twist = G @ [.01, 0]
    assert abs(twist[2]) < 1e-14
    assert twist[:2]@n > 0
    K = A@m.H@A.T
    assert np.linalg.eigvalsh(K).min() > 0
    assert np.linalg.cond(K) < 10


@pytest.mark.parametrize('face', list(ContactFace))
def test_offset_and_force_cone(face):
    m = QuasiStaticPushingModel()
    x = np.array([.1, .2, .3, .02])
    u = np.array([.01, 0])
    r, n, t, F, A = m.contact_geometry(x[3], face)
    assert m.continuous_dynamics(x, u, face)[2] < 0
    _, D = m.matrices(x[3], face)
    f = D@u
    expected = (n@f >= 0 and abs(t@f) <= m.config.mu_pusher*(n@f))
    assert m.is_sticking_feasible(x, u, face) == expected
    assert m.is_sticking_feasible(x, [0, 0], face)
    assert not m.is_sticking_feasible(x, [-.01, 0], face)
    assert not m.is_sticking_feasible(x, [.001, .1], face)
    # Construct force exactly on both cone edges, independently of implementation.
    K = A@m.H@A.T
    for sign in (-1, 1):
        cone_u = F.T@K@(n + sign*m.config.mu_pusher*t)*.001
        if cone_u[0] >= 0:
            assert m.is_sticking_feasible(x, cone_u, face)


@pytest.mark.parametrize('face', list(ContactFace))
@pytest.mark.parametrize('u', [[0., 0.], [.014, -.002]])
def test_jacobian_directional_derivative(face, u):
    m = QuasiStaticPushingModel()
    x = np.array([.03, -.02, np.pi + .000001, .023])
    A, B = m.linearize(x, u, .02, face)
    dx = np.array([.4, -.3, .2, .005]); du = np.array([.01, -.02])
    h = 3e-5  # independent scale/direction and nonlinear secant stencil
    f0 = m.discrete_dynamics(x, u, .02, face)
    forward = (m.discrete_dynamics(x+h*dx, np.array(u)+h*du, .02, face)-f0)/h
    np.testing.assert_allclose(A@dx+B@du, forward, atol=2e-8, rtol=2e-5)
    assert A.shape == (4, 4) and B.shape == (4, 2)
    assert A[3, 3] == pytest.approx(1)
    np.testing.assert_allclose(B[3], 0)


def test_euler_parameters_and_ground_cancellation():
    m = QuasiStaticPushingModel(); x = np.array([0, 0, 0, .02]); u = [.01, .001]
    dt = 1e-5
    np.testing.assert_allclose((m.discrete_dynamics(x, u, dt, 'left')-x)/dt,
                               m.continuous_dynamics(x, u, 'left'), atol=1e-12)
    m2 = QuasiStaticPushingModel(ModelConfig(mu_ground_nominal=.9))
    np.testing.assert_array_equal(m.continuous_dynamics(x, u, 'left'), m2.continuous_dynamics(x, u, 'left'))
    m3 = QuasiStaticPushingModel(ModelConfig(ell=.06))
    assert not np.isclose(m.continuous_dynamics(x,u,'left')[2], m3.continuous_dynamics(x,u,'left')[2])
    A, _ = m.linearize([0,0,0,.04-1e-7], u, .02, 'left')
    assert np.isfinite(A).all()


@pytest.mark.parametrize('x,u', [([1,2,3],[0,0]), ([0,0,0,0],[1]), ([0,0,np.nan,0],[0,0]), ([0,0,0,.04],[0,0])])
def test_model_invalid(x,u):
    with pytest.raises(ValueError):
        QuasiStaticPushingModel().continuous_dynamics(x,u,'left')
