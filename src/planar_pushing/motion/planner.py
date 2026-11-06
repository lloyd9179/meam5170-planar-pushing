"""Deterministic conservative-rectangle visibility graph; no dynamics dependency."""
from dataclasses import dataclass
import numpy as np
from ..task.contracts import RepositionPlan, PlanningResult, MotionCost
from .collision import body_point, world_point, circle_clearance, segment_hits_open_box
from .cost import NominalTimeCost


@dataclass(frozen=True)
class PlannerConfig:
    clearance: float = .005
    vertex_padding: float = .001
    precontact_gap: float = .008
    max_recovery_penetration: float = .001

    def __post_init__(self):
        if not np.isfinite(list(self.__dict__.values())).all() or min(self.__dict__.values()) <= 0:
            raise ValueError('planner distances must be finite and positive')
        if self.precontact_gap <= self.clearance+self.vertex_padding:
            raise ValueError('precontact must lie outside the routing envelope')


class VisibilityRepositionPlanner:
    def __init__(self, geometry, config=None, cost_evaluator=None):
        self.geometry = geometry
        self.config = config or PlannerConfig()
        self.cost_evaluator = cost_evaluator or NominalTimeCost(geometry)
        self.last_candidates = ()

    def plan(self, observation, target):
        self.last_candidates = ()
        try:
            self.geometry.validate_target(target)
        except ValueError as error:
            return PlanningResult(None, 'invalid_contact_target:'+str(error))
        c = self.config
        pose = observation.slider_pose
        start = body_point(observation.pusher_position, pose)
        if circle_clearance(start, self.geometry) < -c.max_recovery_penetration:
            return PlanningResult(None, 'initial_pusher_deeply_interpenetrating')
        half = np.array([self.geometry.length, self.geometry.width])/2+self.geometry.pusher_radius+c.clearance
        route_half = half+c.vertex_padding
        prefix = [start.copy()]
        # Physical-contact start lies inside the conservative buffer. Egress is
        # allowed only outward along an axis; distance to the actual box increases.
        retreat_end = 0
        if np.all(np.abs(start) < route_half):
            axis = int(np.argmax(np.abs(start)-np.array([self.geometry.length, self.geometry.width])/2))
            exit_point = start.copy()
            exit_point[axis] = (1. if start[axis] >= 0 else -1.)*route_half[axis]
            prefix.append(exit_point)
            retreat_end = 1
        goal_world = self.geometry.center_position(pose, target, c.precontact_gap)
        goal = body_point(goal_world, pose)
        nodes = [prefix[-1], goal, *[np.array([sx*route_half[0], sy*route_half[1]])
                                      for sx, sy in ((-1,-1), (-1,1), (1,1), (1,-1))]]
        adjacent = {i: [] for i in range(len(nodes))}
        for i in range(len(nodes)):
            for j in range(i+1, len(nodes)):
                if not segment_hits_open_box(nodes[i], nodes[j], half):
                    adjacent[i].append(j)
                    adjacent[j].append(i)
        paths = []
        def visit(path):
            if path[-1] == 1:
                paths.append(path)
                return
            for j in adjacent[path[-1]]:
                if j not in path:
                    visit(path+[j])
        visit([0])
        candidates = []
        for path in paths:
            points = prefix+[nodes[j] for j in path[1:]]
            plan = RepositionPlan(target, tuple(tuple(world_point(p, pose)) for p in points),
                                  tuple(pose), retreat_end, c.precontact_gap, c.clearance)
            try:
                cost = self.cost_evaluator.evaluate(plan, observation)
            except (ValueError, TypeError, FloatingPointError) as error:
                return PlanningResult(None, 'invalid_cost_evaluation:'+str(error))
            if not isinstance(cost, MotionCost):
                return PlanningResult(None, 'invalid_cost_contract')
            if candidates and cost.unit != candidates[0].cost.unit:
                return PlanningResult(None, 'inconsistent_cost_units')
            candidates.append(PlanningResult(plan, 'visibility_graph_feasible', cost))
        if not candidates:
            return PlanningResult(None, 'no_collision_free_visibility_path')
        # All simple graph paths are enumerated (<=65 with four corners).
        # The tie-break is deterministic; a replacement cost cannot bypass geometry.
        self.last_candidates = tuple(candidates)
        return min(candidates, key=lambda item: (item.cost.total, item.plan.length_m, item.plan.waypoints_world))
