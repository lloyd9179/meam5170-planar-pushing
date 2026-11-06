"""Replaceable routing objective. This is travel cost, not pushing model cost."""
from dataclasses import dataclass
import numpy as np
from ..task.contracts import MotionCost
from ..models.contact_geometry import rotation


@dataclass(frozen=True)
class TravelCostConfig:
    retreat_speed: float = .015
    transit_speed: float = .03
    approach_speed: float = .003
    turn_seconds_per_radian: float = .05
    fixed_settle_seconds: float = .1

    def __post_init__(self):
        for key, value in self.__dict__.items():
            if not np.isfinite(value) or value < 0 or (key.endswith('speed') and value == 0):
                raise ValueError(key)


class NominalTimeCost:
    def __init__(self, geometry, config=None):
        self.geometry = geometry
        self.config = config or TravelCostConfig()

    def evaluate(self, plan, observation):
        c = self.config
        segments = np.diff(np.asarray(plan.waypoints_world), axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        retreat = float(sum(lengths[:plan.retreat_end_index]))
        transit = plan.length_m-retreat
        directions = [v/length for v, length in zip(segments, lengths) if length > 1e-12]
        _, normal, _ = self.geometry.frame(plan.target.face)
        directions.append(rotation(plan.slider_pose_anchor[2]) @ normal)
        turns = sum(float(np.arccos(np.clip(a @ b, -1., 1.))) for a, b in zip(directions, directions[1:]))
        nominal = retreat/c.retreat_speed+transit/c.transit_speed+plan.precontact_gap/c.approach_speed
        turn_cost = c.turn_seconds_per_radian*turns
        total = nominal+turn_cost+c.fixed_settle_seconds
        components = dict(Jtravel_m=plan.length_m, retreat_m=retreat, transit_m=transit,
                          approach_m=plan.precontact_gap, nominal_motion_s=nominal,
                          turn_radians=turns, turn_penalty_s=turn_cost,
                          settle_s=c.fixed_settle_seconds)
        return MotionCost(float(total), 's', tuple(components.items()))
