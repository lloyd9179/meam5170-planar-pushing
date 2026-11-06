"""Feedback execution of retreat, free transit, and guarded contact approach."""
from dataclasses import dataclass, asdict
import numpy as np
from ..task.contracts import Phase, MotionStatus, MotionUpdate
from ..task.selection import angle_error
from ..models.contact_geometry import rotation
from .collision import RectangleCollisionChecker


@dataclass(frozen=True)
class MotionConfig:
    dt: float = .02
    retreat_speed: float = .015
    transit_speed: float = .03
    approach_speed: float = .003
    position_gain: float = 6.
    waypoint_tolerance: float = .0004
    execution_clearance: float = .0015
    settle_seconds: float = .1
    settle_timeout: float = 2.
    slider_linear_tolerance: float = .0015
    slider_angular_tolerance: float = .02
    pose_replan_distance: float = .001
    pose_replan_angle: float = .02
    max_replans: int = 3
    max_seconds: float = 30.
    approach_timeout: float = 10.
    contact_hold_seconds: float = .04
    preload_depth: float = .0001
    max_approach_penetration: float = .0008
    max_recovery_penetration: float = .001

    def __post_init__(self):
        for key, value in self.__dict__.items():
            if key == 'max_replans':
                if not isinstance(value, int) or value < 0:
                    raise ValueError(key)
            elif not np.isfinite(value) or value <= 0:
                raise ValueError(key)
        if self.settle_timeout < self.settle_seconds:
            raise ValueError('settle_timeout shorter than settle interval')


class FeedbackRepositionExecutor:
    def __init__(self, geometry, planner, monitor, config=None, collision_checker=None):
        self.geometry, self.planner, self.monitor = geometry, planner, monitor
        self.config = config or MotionConfig()
        self.collision = collision_checker or RectangleCollisionChecker(geometry)
        self.records = []
        self.events = []
        self.cancel()

    def cancel(self):
        self.request = self.result = None
        self.phase = Phase.RETRACT
        self.started_at = self.last_time = None
        self.quiet_since = self.settle_started = self.approach_started = self.loaded_since = None
        self.index = 0
        self.replans = 0
        self.failure = None
        self.updated = False

    def _out(self, status=MotionStatus.RUNNING, reason='', command=(0.,0.)):
        return MotionUpdate(tuple(command), self.phase, status, reason)

    def _fail(self, observation, reason):
        self.failure = reason
        self.events.append(dict(time=observation.time, event='failed', reason=reason))
        return self._out(MotionStatus.FAILED, reason)

    def _plan(self, observation, reason):
        try:
            self.result = self.planner.plan(observation, self.request.target)
        except (ValueError, RuntimeError, FloatingPointError) as error:
            self.failure = 'planner_error:'+str(error)
            return
        if self.result.plan is None:
            self.failure = self.result.reason
            return
        plan = self.result.plan
        if plan.target != self.request.target:
            self.failure = 'planner_returned_wrong_target'
            return
        if np.linalg.norm(np.subtract(plan.waypoints_world[0], observation.pusher_position)) > self.config.waypoint_tolerance:
            self.failure = 'plan_start_does_not_match_observation'
            return
        end = self.geometry.center_position(plan.slider_pose_anchor, plan.target, plan.precontact_gap)
        if np.linalg.norm(np.subtract(plan.waypoints_world[-1], end)) > self.config.waypoint_tolerance:
            self.failure = 'plan_endpoint_is_not_precontact'
            return
        for a, b in zip(plan.waypoints_world[plan.retreat_end_index:], plan.waypoints_world[plan.retreat_end_index+1:]):
            if not self.collision.segment_free(a, b, plan.slider_pose_anchor, plan.clearance):
                self.failure = 'planner_transit_segment_blocked'
                return
        self.phase = Phase.RETRACT
        self.index = 1
        self.quiet_since = self.settle_started = self.approach_started = self.loaded_since = None
        self.records.append(dict(time=observation.time, reason=reason, plan=asdict(plan),
                                 length_m=plan.length_m, cost=asdict(self.result.cost) if self.result.cost else None))

    def start(self, observation, request):
        self.cancel()
        self.request = request
        self.started_at = self.last_time = observation.time
        self._plan(observation, 'initial_plan')

    def _replan(self, observation, reason):
        if self.replans >= self.config.max_replans:
            return self._fail(observation, 'replan_limit:'+reason)
        self.replans += 1
        self.events.append(dict(time=observation.time, event='replan', reason=reason))
        self._plan(observation, reason)
        if self.failure:
            return self._fail(observation, self.failure)
        return self._out(reason='replanned:'+reason)

    def _follow(self, observation, endpoint, speed, phase):
        delta = np.asarray(endpoint)-observation.pusher_position
        distance = np.linalg.norm(delta)
        if distance <= self.config.waypoint_tolerance:
            return None
        velocity = delta/distance*min(speed, self.config.position_gain*distance)
        future = np.asarray(observation.pusher_position)+velocity*self.config.dt
        if phase == Phase.TRANSIT:
            if not self.collision.segment_free(observation.pusher_position, future, observation.slider_pose,
                                                self.config.execution_clearance):
                return self._replan(observation, 'transit_swept_segment_blocked')
        else:
            clear = self.collision.clearance(observation.pusher_position, observation.slider_pose)
            if clear < -self.config.max_recovery_penetration:
                return self._fail(observation, 'retreat_initial_penetration_exceeds_limit')
            if self.collision.clearance(future, observation.slider_pose) < clear-1e-6:
                return self._fail(observation, 'retreat_would_reduce_clearance')
        return self._out(reason='following_'+phase.value, command=velocity)

    def update(self, observation):
        c = self.config
        if self.request is None:
            return self._fail(observation, 'executor_not_started')
        if not observation.pusher_velocity_valid:
            return self._fail(observation, 'physical_pusher_velocity_unavailable')
        elapsed = observation.time-self.last_time
        if ((self.updated and abs(elapsed-c.dt) > 1e-8)
                or (not self.updated and min(abs(elapsed), abs(elapsed-c.dt)) > 1e-8)):
            return self._fail(observation, 'control_timestep_mismatch')
        if observation.time < self.last_time:
            return self._fail(observation, 'observation_time_reversed')
        self.last_time = observation.time
        self.updated = True
        if self.failure:
            return self._fail(observation, self.failure)
        if observation.time-self.started_at >= c.max_seconds:
            return self._fail(observation, 'motion_timeout')
        plan = self.result.plan
        if plan.clearance <= c.execution_clearance+c.waypoint_tolerance:
            return self._fail(observation, 'plan_clearance_smaller_than_tracking_budget')
        pose = observation.slider_pose
        if (np.linalg.norm(np.subtract(pose[:2], plan.slider_pose_anchor[:2])) > c.pose_replan_distance
                or abs(angle_error(pose[2], plan.slider_pose_anchor[2])) > c.pose_replan_angle):
            return self._replan(observation, 'slider_pose_changed')
        if self.phase == Phase.RETRACT:
            if self.index <= plan.retreat_end_index:
                update = self._follow(observation, plan.waypoints_world[self.index], c.retreat_speed, Phase.RETRACT)
                if update is not None:
                    return update
                self.index += 1
                return self._out(reason='retreat_waypoint_reached')
            if self.settle_started is None:
                self.settle_started = observation.time
            force = sum(sample.normal_force for sample in observation.contacts)
            quiet = (np.linalg.norm(observation.slider_twist[:2]) <= c.slider_linear_tolerance
                     and abs(observation.slider_twist[2]) <= c.slider_angular_tolerance
                     and force < self.monitor.config.load_threshold)
            if quiet:
                if self.quiet_since is None:
                    self.quiet_since = observation.time
                if observation.time-self.quiet_since >= c.settle_seconds-1e-10:
                    self.phase = Phase.TRANSIT
                    self.index = plan.retreat_end_index+1
                    return self._out(reason='retreat_unloaded_and_slider_settled')
            else:
                self.quiet_since = None
            if observation.time-self.settle_started > c.settle_timeout:
                return self._fail(observation, 'slider_did_not_settle')
            return self._out(reason='waiting_for_unloaded_stationary_slider')
        if self.phase == Phase.TRANSIT:
            if sum(sample.normal_force for sample in observation.contacts) >= self.monitor.config.load_threshold:
                return self._fail(observation, 'unexpected_transit_contact')
            if self.index < len(plan.waypoints_world):
                update = self._follow(observation, plan.waypoints_world[self.index], c.transit_speed, Phase.TRANSIT)
                if update is not None:
                    return update
                self.index += 1
                return self._out(reason='transit_waypoint_reached')
            self.phase = Phase.APPROACH
            self.approach_started = observation.time
            return self._out(reason='precontact_reached')
        estimate = self.monitor.estimate(observation, plan.target)
        if observation.time-self.approach_started > c.approach_timeout:
            return self._fail(observation, 'approach_timeout')
        if not estimate.in_bounds or estimate.gap < -c.max_approach_penetration:
            return self._fail(observation, 'approach_geometry_invalid')
        if self.monitor.matches(estimate, plan.target):
            if self.loaded_since is None:
                self.loaded_since = observation.time
            if observation.time-self.loaded_since >= c.contact_hold_seconds-1e-10:
                self.events.append(dict(time=observation.time, event='ready', reason='loaded_target_contact'))
                return self._out(MotionStatus.READY, 'loaded_target_contact')
        else:
            self.loaded_since = None
        # Track the real moving face, never the slider state. Tangential correction
        # is allowed only before loaded contact; loaded misalignment is a failure.
        r0, n, tangent = self.geometry.frame(plan.target.face)
        R = rotation(pose[2])
        tangent_error = plan.target.s-estimate.measured_s
        if estimate.loaded and abs(tangent_error) > self.monitor.config.match_s_tolerance:
            return self._fail(observation, 'approach_contact_misaligned')
        vn = np.clip(c.position_gain*(estimate.gap+c.preload_depth), 0., c.approach_speed)
        vt = 0. if estimate.loaded else np.clip(c.position_gain*tangent_error, -.002, .002)
        if abs(tangent_error) > self.monitor.config.match_s_tolerance:
            vn = 0.
        return self._out(reason='guarded_normal_approach', command=R @ (vn*n+vt*tangent))
