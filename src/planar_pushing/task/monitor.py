"""Pure contact estimation from physical geometry and simulator-free observations."""
from dataclasses import dataclass
import numpy as np
from ..models.contact_geometry import rotation
from .contracts import ContactEstimate


@dataclass(frozen=True)
class ContactMonitorConfig:
    load_threshold: float = .01  # N
    face_distance_tolerance: float = .0015
    match_s_tolerance: float = .003
    match_gap_tolerance: float = .0005
    slip_flag_speed: float = .002

    def __post_init__(self):
        if not np.isfinite(list(self.__dict__.values())).all() or min(self.__dict__.values()) <= 0:
            raise ValueError('contact monitor settings must be finite and positive')


class ContactMonitor:
    def __init__(self, geometry, config=None):
        self.geometry = geometry
        self.config = config or ContactMonitorConfig()

    def estimate(self, observation, target):
        pose = np.asarray(observation.slider_pose)
        R = rotation(pose[2])
        r0, n, t = self.geometry.frame(target.face)
        pc = R.T @ (np.asarray(observation.pusher_position)-pose[:2])
        s = float((pc-r0) @ t)
        r = r0+s*t  # algebra valid even outside face; do not crash at an edge
        gap = float(-(pc-r) @ n-self.geometry.pusher_radius)
        lo, hi = self.geometry.bounds(target.face)
        selected = []
        for sample in observation.contacts:
            local = R.T @ (np.asarray(sample.position_world)-pose[:2])
            if (abs((local-r0) @ n) <= self.config.face_distance_tolerance
                    and abs((local-r0) @ t) <= hi+self.geometry.corner_margin):
                selected.append(sample)
        force = sum(c.normal_force for c in selected)
        loaded = force >= self.config.load_threshold
        slip, point_error = None, None
        if loaded:
            point = np.average([c.position_world for c in selected], axis=0,
                               weights=[c.normal_force for c in selected])
            point_error = float(np.linalg.norm(point-(pose[:2]+R@r)))
            if observation.pusher_velocity_valid:
                arm = point-pose[:2]
                vx, vy, omega = observation.slider_twist
                material_velocity = np.array([vx, vy])+omega*np.array([-arm[1], arm[0]])
                slip = float((R@t) @ (np.asarray(observation.pusher_velocity)-material_velocity))
        return ContactEstimate(target.face, s, gap, lo <= s <= hi, loaded, force, slip, point_error)

    def matches(self, estimate, target):
        c = self.config
        return (estimate.face == target.face and estimate.in_bounds and estimate.loaded
                and abs(estimate.measured_s-target.s) <= c.match_s_tolerance
                and abs(estimate.gap) <= c.match_gap_tolerance)
