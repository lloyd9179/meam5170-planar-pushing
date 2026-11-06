"""Analytic 2D geometry: rectangular C-space envelope and circular-pusher distance."""
import numpy as np
from typing import Protocol
from ..models.contact_geometry import rotation
from ..task.contracts import finite_tuple


def body_point(point, pose):
    return rotation(pose[2]).T @ (np.asarray(point)-pose[:2])


def world_point(point, pose):
    return np.asarray(pose[:2])+rotation(pose[2]) @ point


def circle_clearance(point_body, geometry):
    """Signed distance from circular pusher to the physical rectangle (m)."""
    q = np.abs(point_body)-np.array([geometry.length, geometry.width])/2
    return float(np.linalg.norm(np.maximum(q, 0))+min(float(np.max(q)), 0)-geometry.pusher_radius)


def segment_hits_open_box(a, b, half, epsilon=1e-12):
    """Slab intersection with the OPEN interior; boundary following is permitted.

    Epsilon only removes roundoff at boundaries; routing vertices have an
    explicit millimeter padding and are not relying on this numerical epsilon.
    """
    a, b, half = np.asarray(a), np.asarray(b), np.asarray(half)-epsilon
    low, high = 0., 1.
    for axis in range(2):
        delta = b[axis]-a[axis]
        if abs(delta) <= 1e-15:
            if abs(a[axis]) >= half[axis]:
                return False
        else:
            left, right = sorted(((-half[axis]-a[axis])/delta, (half[axis]-a[axis])/delta))
            low, high = max(low, left), min(high, right)
            if low > high:
                return False
    return low <= high


def buffered_segment_free(a_world, b_world, pose, geometry, clearance):
    a = body_point(a_world, pose)
    b = body_point(b_world, pose)
    half = np.array([geometry.length, geometry.width])/2+geometry.pusher_radius+clearance
    return not segment_hits_open_box(a, b, half)


class CollisionChecker(Protocol):
    def clearance(self, point_world, slider_pose) -> float: ...
    def segment_free(self, a_world, b_world, slider_pose, margin: float) -> bool: ...


class RectangleCollisionChecker:
    """Injectable safety geometry, independent of route cost and dynamics."""
    def __init__(self, geometry):
        self.geometry = geometry

    def clearance(self, point_world, slider_pose):
        return circle_clearance(body_point(point_world, slider_pose), self.geometry)

    def segment_free(self, a_world, b_world, slider_pose, margin):
        return buffered_segment_free(a_world, b_world, slider_pose, self.geometry, margin)
