"""Hybrid task execution. No env handles, qpos writes, resets or path algorithms."""
from dataclasses import dataclass, replace
import numpy as np
from .contracts import (Phase, TaskDecision, SelectionContext, RepositionRequest,
                        MotionStatus, finite_tuple)
from .selection import angle_error
from .progress import StagnationDetector


@dataclass(frozen=True)
class SupervisorConfig:
    position_tolerance: float = .005
    angle_tolerance: float = float(np.deg2rad(3.))
    goal_hold_seconds: float = .2
    contact_confirm_seconds: float = .06
    contact_confirm_timeout: float = .5
    min_contact_hold_seconds: float = .5
    loss_gap: float = .0005
    loss_seconds: float = .06
    max_s_drift: float = .008
    max_contact_penetration: float = .0015
    max_task_seconds: float = 30.
    max_selection_attempts: int = 4
    max_command_speed: float = .08

    def __post_init__(self):
        for key, value in self.__dict__.items():
            if key == 'max_selection_attempts':
                if not isinstance(value, int) or value < 1:
                    raise ValueError(key)
            elif not np.isfinite(value) or value <= 0:
                raise ValueError(key)
        if self.contact_confirm_timeout < self.contact_confirm_seconds:
            raise ValueError('confirmation timeout shorter than confirmation interval')


class TaskSupervisor:
    def __init__(self, selector, push_controller, monitor, *, config=None,
                 progress_config=None, reposition_executor=None):
        self.selector = selector
        self.push_controller = push_controller
        self.monitor = monitor
        self.config = config or SupervisorConfig()
        self.progress = StagnationDetector(progress_config)
        self.executor = reposition_executor
        self.reset()

    def reset(self, seed=0):
        if self.executor is not None:
            self.executor.cancel()
        self.selector.reset(seed)
        self.push_controller.reset()
        self.phase = Phase.SELECT
        self.target = self.active_target = self.request = None
        self.blocked = []
        self.attempts = 0
        self.goal = self.started_at = self.last_time = None
        self.confirmed_since = self.confirm_started = self.hold_since = self.loss_since = None
        self.activated_at = None
        self.motion_started = False
        self.last_progress = None
        self.progress.reset()
        self.events = []

    def _phase(self, phase, observation, reason):
        if phase != self.phase:
            self.events.append(dict(time=observation.time, previous=self.phase.value,
                                    phase=phase.value, reason=reason))
        self.phase = phase

    def _emit(self, reason, contact=None, push=None, command=(0., 0.)):
        command = finite_tuple(command, 2, 'velocity_world')
        return TaskDecision(command, self.phase, reason, self.target, contact, self.request, push)

    def _fail(self, observation, reason, contact=None):
        self.push_controller.reset()
        if self.executor is not None:
            self.executor.cancel()
        self._phase(Phase.FAILED, observation, reason)
        return self._emit(reason, contact)

    def _request(self, observation, trigger):
        self.push_controller.reset()
        self.progress.reset()
        self.last_progress = None
        self.request = RepositionRequest(self.target, self.active_target, trigger,
                                         observation.time, observation.slider_pose)
        self.motion_started = False
        self._phase(Phase.REPOSITION_REQUIRED, observation, trigger)
        if self.executor is None:
            return self._emit('step4_required:'+trigger)
        try:
            self.executor.start(observation, self.request)
        except (ValueError, RuntimeError) as error:
            return self._fail(observation, 'reposition_start_error:'+str(error))
        self.motion_started = True
        return self._emit('reposition_started:'+trigger)

    def _select(self, observation, trigger):
        if self.attempts >= self.config.max_selection_attempts:
            return self._fail(observation, 'selection_attempt_limit:'+trigger)
        context = SelectionContext(self.active_target, tuple(self.blocked), trigger)
        result = self.selector.select(observation, self.goal, context)
        self.attempts += 1
        if result.target is None:
            return self._fail(observation, result.reason)
        self.monitor.geometry.validate_target(result.target)
        self.target = result.target
        self.events.append(dict(time=observation.time, event='selection', face=self.target.face.value,
                                s=self.target.s, reason=result.reason))
        estimate = self.monitor.estimate(observation, self.target)
        if self.monitor.matches(estimate, self.target):
            self.confirmed_since = self.confirm_started = observation.time
            self._phase(Phase.CONFIRM_CONTACT, observation, result.reason)
            return self._emit('confirming:'+result.reason, estimate)
        return self._request(observation, trigger)

    def _reselect(self, observation, trigger):
        if self.target is not None:
            self.blocked.append(self.target)
        self.push_controller.reset()
        self.progress.reset()
        self.last_progress = None
        self._phase(Phase.SELECT, observation, trigger)
        return self._select(observation, trigger)

    def tick(self, observation, goal_pose):
        goal = finite_tuple(goal_pose, 3, 'goal_pose')
        if self.last_time is not None and observation.time <= self.last_time:
            raise ValueError('one tick per strictly increasing observation time; reset for new episode')
        # A new goal is a new task, never an environment reset.
        if self.goal is not None and (np.linalg.norm(np.subtract(goal[:2], self.goal[:2])) > 1e-12
                                      or abs(angle_error(goal[2], self.goal[2])) > 1e-12):
            self.reset()
        self.goal = goal
        self.last_time = observation.time
        if self.started_at is None:
            self.started_at = observation.time
        if self.phase in (Phase.SUCCEEDED, Phase.FAILED):
            return self._emit('terminal')
        if observation.time-self.started_at >= self.config.max_task_seconds:
            return self._fail(observation, 'task_timeout')
        near_goal = (np.linalg.norm(np.subtract(goal[:2], observation.slider_pose[:2])) <= self.config.position_tolerance
                     and abs(angle_error(goal[2], observation.slider_pose[2])) <= self.config.angle_tolerance)
        if near_goal:
            if self.phase != Phase.GOAL_HOLD:
                if self.executor is not None:
                    self.executor.cancel()
                self.push_controller.reset()
                self.request = None
                self.motion_started = False
                self.hold_since = observation.time
                self.progress.reset()
                self.last_progress = None
                self._phase(Phase.GOAL_HOLD, observation, 'goal_within_tolerance')
            if observation.time-self.hold_since >= self.config.goal_hold_seconds-1e-10:
                self._phase(Phase.SUCCEEDED, observation, 'goal_confirmed')
            return self._emit('goal_hold' if self.phase == Phase.GOAL_HOLD else 'goal_confirmed')
        if self.phase == Phase.GOAL_HOLD:
            self.progress.reset()
            self._phase(Phase.SELECT, observation, 'goal_left_tolerance')
            return self._select(observation, 'goal_left_tolerance')
        if self.phase == Phase.SELECT:
            return self._select(observation, 'initial')
        if self.request is not None:
            if self.executor is None:
                return self._emit('step4_required:'+self.request.trigger)
            try:
                update = self.executor.update(observation)
            except (ValueError, RuntimeError) as error:
                return self._fail(observation, 'reposition_update_error:'+str(error))
            if update.status == MotionStatus.FAILED:
                return self._fail(observation, 'reposition_failed:'+update.reason)
            if update.status == MotionStatus.READY:
                self.request = None
                self.motion_started = False
                self.confirm_started = observation.time
                self.confirmed_since = None
                self._phase(Phase.CONFIRM_CONTACT, observation, 'motion_ready_requires_contact_confirmation')
                return self._emit('motion_ready_requires_contact_confirmation')
            if np.linalg.norm(update.velocity_world) > self.config.max_command_speed+1e-12:
                return self._fail(observation, 'motion_command_exceeds_speed_limit')
            self._phase(update.phase, observation, update.reason)
            return self._emit(update.reason, command=update.velocity_world)
        estimate = self.monitor.estimate(observation, self.target)
        if self.phase == Phase.CONFIRM_CONTACT:
            if self.monitor.matches(estimate, self.target):
                if self.confirmed_since is None:
                    self.confirmed_since = observation.time
                if observation.time-self.confirmed_since >= self.config.contact_confirm_seconds-1e-10:
                    self.active_target = self.target
                    self.push_controller.activate(self.target)
                    self.activated_at = observation.time
                    self.loss_since = None
                    self.progress.reset()
                    self._phase(Phase.PUSH, observation, 'contact_confirmed')
                    return self._emit('contact_confirmed', estimate)
            else:
                self.confirmed_since = None
            if observation.time-self.confirm_started >= self.config.contact_confirm_timeout:
                return self._request(observation, 'contact_confirmation_timeout')
            return self._emit('confirming_contact', estimate)
        if not estimate.in_bounds:
            return self._reselect(observation, 'contact_out_of_bounds')
        if estimate.gap < -self.config.max_contact_penetration:
            return self._reselect(observation, 'selected_face_geometry_invalid')
        if abs(estimate.measured_s-self.active_target.s) > self.config.max_s_drift:
            return self._reselect(observation, 'contact_location_drift')
        if not estimate.loaded and estimate.gap > self.config.loss_gap:
            if self.loss_since is None:
                self.loss_since = observation.time
                self.progress.reset()
                self.last_progress = None
            if observation.time-self.loss_since >= self.config.loss_seconds-1e-10:
                return self._reselect(observation, 'contact_lost')
            return self._emit('contact_loss_pending', estimate)  # zero while checking macroscopic loss
        self.loss_since = None
        self.last_progress = self.progress.update(observation, goal)
        if (self.last_progress.stalled
                and observation.time-self.activated_at >= self.config.min_contact_hold_seconds):
            return self._reselect(observation, 'stagnation')
        push = self.push_controller.compute(observation, estimate, goal)
        if not push.feasible:
            return replace(self._reselect(observation, 'solver_failure:'+push.status), push=push)
        if np.linalg.norm(push.velocity_world) > self.config.max_command_speed+1e-12:
            return self._fail(observation, 'push_command_exceeds_speed_limit', estimate)
        return self._emit('pushing', estimate, push, push.velocity_world)
