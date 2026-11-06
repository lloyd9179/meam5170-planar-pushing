"""Physical scene geometry, independent of nominal friction/dynamics parameters."""
from dataclasses import dataclass
import numpy as np
from ..models.contact_geometry import geometry, physical_half_length, rotation
from .contracts import ContactTarget


@dataclass(frozen=True)
class RectangleGeometry:
    length: float = .12
    width: float = .08
    pusher_radius: float = .01
    corner_margin: float = .002

    def __post_init__(self):
        if not np.isfinite([self.length, self.width, self.pusher_radius, self.corner_margin]).all():
            raise ValueError('nonfinite geometry')
        if min(self.length, self.width, self.pusher_radius, self.corner_margin) <= 0:
            raise ValueError('geometry dimensions and margin must be positive')
        if 2*self.corner_margin >= min(self.length, self.width):
            raise ValueError('corner margin leaves no usable contact')

    def bounds(self, face):
        half = physical_half_length(face, self.length, self.width)-self.corner_margin
        return -half, half

    def frame(self, face):
        r0, n, t, _, _ = geometry(face, 0., self.length, self.width)
        return r0, n, t

    def validate_target(self, target):
        lo, hi = self.bounds(target.face)
        if not lo <= target.s <= hi:
            raise ValueError('desired contact outside safe physical face interval')

    def center_position(self, slider_pose, target, gap=0.):
        self.validate_target(target)
        if not np.isfinite(gap) or gap < 0:
            raise ValueError('initial/precontact gap must be nonnegative')
        r0, n, t = self.frame(target.face)
        pose = np.asarray(slider_pose)
        return pose[:2]+rotation(pose[2]) @ (r0+target.s*t-(self.pusher_radius+gap)*n)
