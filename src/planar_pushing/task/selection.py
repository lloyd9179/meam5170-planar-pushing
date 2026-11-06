"""Controller A: deterministic geometric rules, no rollout or solver scoring."""
from dataclasses import dataclass
import numpy as np
from ..models.contact_geometry import ContactFace, rotation
from .contracts import ContactTarget, SelectionResult, finite_tuple


def angle_error(goal, current):
    return float(np.arctan2(np.sin(goal-current), np.cos(goal-current)))


@dataclass(frozen=True)
class RuleSelectionConfig:
    rotation_gain: float = .16  # m/rad, heuristic offset=-gain*yaw_error
    rotation_deadband: float = .02
    max_offset_fraction: float = .6  # fraction of safe half-face length
    translation_deadband: float = .005
    face_hysteresis: float = .1  # normal alignment score [-1,1]
    blocked_s_tolerance: float = .003

    def __post_init__(self):
        values = list(self.__dict__.values())
        if not np.isfinite(values).all() or min(values) < 0:
            raise ValueError('rule parameters must be finite and nonnegative')
        if not 0 < self.max_offset_fraction <= 1 or self.face_hysteresis > 2:
            raise ValueError('invalid offset fraction / hysteresis')


class RuleContactSelector:
    # Explicit tie/fallback order, not a candidate MPC search.
    face_order = (ContactFace.LEFT, ContactFace.BOTTOM, ContactFace.RIGHT, ContactFace.TOP)

    def __init__(self, geometry, config=None):
        self.geometry = geometry
        self.config = config or RuleSelectionConfig()

    def reset(self, seed=0):
        pass  # no randomness or hidden episode state

    def select(self, observation, goal_pose, context):
        goal = np.array(finite_tuple(goal_pose, 3, 'goal_pose'))
        pose = np.asarray(observation.slider_pose)
        error_body = rotation(pose[2]).T @ (goal[:2]-pose[:2])
        distance = np.linalg.norm(error_body)
        direction = error_body/distance if distance > self.config.translation_deadband else np.zeros(2)
        scores = {face: float(self.geometry.frame(face)[1] @ direction) for face in self.face_order}
        ranked = sorted(self.face_order, key=lambda face: -scores[face])
        if context.current is not None:
            face = context.current.face
            if scores[face] >= scores[ranked[0]]-self.config.face_hysteresis:
                ranked.remove(face)
                ranked.insert(0, face)
        yaw_error = angle_error(goal[2], pose[2])
        for rank, face in enumerate(ranked):
            _, maximum = self.geometry.bounds(face)
            s = 0. if abs(yaw_error) <= self.config.rotation_deadband else float(
                np.clip(-self.config.rotation_gain*yaw_error,
                        -maximum*self.config.max_offset_fraction,
                        maximum*self.config.max_offset_fraction))
            target = ContactTarget(face, s)
            if any(b.face == face and abs(b.s-s) <= self.config.blocked_s_tolerance for b in context.blocked):
                continue
            reason = (f'rule_{"primary" if rank == 0 else "fallback"}:normal_alignment={scores[face]:.3f};'
                      f'yaw_error={yaw_error:.6f};s={s:.6f};trigger={context.trigger}')
            return SelectionResult(target, reason)
        return SelectionResult(None, 'all_deterministic_rule_targets_blocked')
