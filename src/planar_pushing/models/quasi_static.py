"""Ellipsoidal limit-surface, single fixed-face sticking prediction model."""
from dataclasses import dataclass
import numpy as np
from .contact_geometry import geometry, physical_half_length, rotation


def vector(value, size, name):
    x = np.asarray(value, dtype=float)
    if x.shape != (size,) or not np.isfinite(x).all():
        raise ValueError(f'{name} must be a finite ({size},) vector')
    return x


@dataclass(frozen=True)
class ModelConfig:
    slider_length: float = .12
    slider_width: float = .08
    ell: float | None = None
    mu_pusher: float = .5
    mu_ground_nominal: float = .3
    fd_step: float = 1e-6
    corner_margin: float = .002

    def __post_init__(self):
        for key in ('slider_length', 'slider_width', 'fd_step', 'corner_margin'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if 2*self.corner_margin >= min(self.slider_length, self.slider_width):
            raise ValueError('corner_margin leaves no usable face')
        if self.ell is None:
            object.__setattr__(self, 'ell', np.hypot(self.slider_length, self.slider_width)/np.sqrt(12))
        if not np.isfinite(self.ell) or self.ell <= 0:
            raise ValueError('ell must be finite and positive')
        for key in ('mu_pusher', 'mu_ground_nominal'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) < 0:
                raise ValueError(f'{key} must be finite and nonnegative')


class QuasiStaticPushingModel:
    state_dim, control_dim = 4, 2

    def __init__(self, config=None):
        self.config = config or ModelConfig()
        self.H = np.diag([1., 1., 1/self.config.ell**2])

    def contact_geometry(self, s, face):
        return geometry(face, s, self.config.slider_length, self.config.slider_width)

    def contact_bounds(self, face):
        half = physical_half_length(face, self.config.slider_length, self.config.slider_width)
        return -half+self.config.corner_margin, half-self.config.corner_margin

    def matrices(self, s, face):
        _, n, t, F, A = self.contact_geometry(s, face)
        K = A @ self.H @ A.T
        D = np.linalg.solve(K, F)
        G = self.H @ A.T @ D
        if not np.isfinite(G).all() or np.linalg.cond(K) > 1e10:
            raise ValueError('ill-conditioned contact mobility matrix')
        return G, D

    def continuous_dynamics(self, x, u, contact_face):
        x, u = vector(x, 4, 'x'), vector(u, 2, 'u')
        G, _ = self.matrices(x[3], contact_face)
        twist = G @ u
        return np.r_[rotation(x[2]) @ twist[:2], twist[2], 0.]

    def discrete_dynamics(self, x, u, dt, contact_face):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        x = vector(x, 4, 'x')
        return x + dt*self.continuous_dynamics(x, u, contact_face)

    def linearize(self, x, u, dt, contact_face):
        x, u = vector(x, 4, 'x'), vector(u, 2, 'u')
        self.discrete_dynamics(x, u, dt, contact_face)
        A, B = np.empty((4, 4)), np.empty((4, 2))
        half = physical_half_length(contact_face, self.config.slider_length, self.config.slider_width)
        for i in range(4):
            h = self.config.fd_step*max(1., abs(x[i]))
            if i == 3:
                h = min(h, (half-abs(x[3]))/4)
            if h <= np.finfo(float).eps*max(1., abs(x[i])):
                raise ValueError('state too close to physical corner to differentiate')
            d = np.eye(4)[i]*h
            A[:, i] = (self.discrete_dynamics(x+d, u, dt, contact_face)
                       - self.discrete_dynamics(x-d, u, dt, contact_face))/(2*h)
        for i in range(2):
            h = self.config.fd_step*max(1., abs(u[i]))
            d = np.eye(2)[i]*h
            B[:, i] = (self.discrete_dynamics(x, u+d, dt, contact_face)
                       - self.discrete_dynamics(x, u-d, dt, contact_face))/(2*h)
        if not np.isfinite(A).all() or not np.isfinite(B).all():
            raise ValueError('nonfinite Jacobian')
        return A, B

    def sticking_matrix(self, x, contact_face):
        x = vector(x, 4, 'x')
        _, n, t, _, _ = self.contact_geometry(x[3], contact_face)
        _, D = self.matrices(x[3], contact_face)
        mu = self.config.mu_pusher
        return np.vstack((-n @ D, (t-mu*n) @ D, (-t-mu*n) @ D, [-1., 0.]))

    def is_sticking_feasible(self, x, u, contact_face, tolerance=1e-9):
        if not np.isfinite(tolerance) or tolerance < 0:
            raise ValueError('tolerance must be finite and nonnegative')
        return bool(np.all(self.sticking_matrix(x, contact_face) @ vector(u, 2, 'u') <= tolerance))
