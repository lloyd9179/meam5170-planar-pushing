from dataclasses import replace
import numpy as np
import pytest
from planar_pushing.task import (WorldObservation, ContactSample, ContactTarget, RectangleGeometry,
    RuleContactSelector, ContactMonitor, TaskSupervisor, SupervisorConfig, ProgressConfig,
    PushResult, Phase, MotionUpdate, MotionStatus, RepositionPlan)


class FakePush:
    def __init__(self, feasible=True, speed=.01):
        self.activations = []
        self.calls = 0
        self.resets = 0
        self.feasible = feasible
        self.speed = speed

    def reset(self):
        self.resets += 1

    def activate(self, target):
        self.activations.append(target)

    def compute(self, observation, contact, goal_pose):
        self.calls += 1
        return PushResult((self.speed, 0.), self.feasible, 'solved' if self.feasible else 'primal infeasible')


class FakeMotion:
    def __init__(self):
        self.starts = []
        self.cancelled = 0
        self.next = MotionUpdate((0., 0.), Phase.APPROACH, MotionStatus.READY)

    def start(self, observation, request):
        self.starts.append(request)

    def update(self, observation):
        return self.next

    def cancel(self):
        self.cancelled += 1


def observation(time, *, target=None, pose=(0., 0., 0.), loaded=True, gap=0.):
    g = RectangleGeometry()
    target = target or ContactTarget('left', 0.)
    center = g.center_position(pose, target, gap=gap)
    r0, n, t = g.frame(target.face)
    # Rotation for nonzero poses is exercised separately by geometry integration tests.
    from planar_pushing.models.contact_geometry import rotation
    point = np.asarray(pose[:2])+rotation(pose[2]) @ (r0+target.s*t)
    contacts = (ContactSample(tuple(point), .5),) if loaded else ()
    return WorldObservation(pose, tuple(center), (0., 0., 0.), (0., 0.), time, contacts)


def make(push=None, motion=None, config=None):
    geometry = RectangleGeometry()
    push = push or FakePush()
    sup = TaskSupervisor(RuleContactSelector(geometry), push, ContactMonitor(geometry),
                         config=config, reposition_executor=motion)
    return sup, push


def activate(sup, goal=(.04, 0, 0)):
    for k in range(4):
        result = sup.tick(observation(k*.02), goal)
    assert result.phase == Phase.PUSH
    return result


def test_gate_contact_confirmation_and_goal_hold():
    sup, push = make()
    first = sup.tick(observation(0), (.04, 0, 0))
    assert first.phase == Phase.CONFIRM_CONTACT and first.velocity_world == (0., 0.)
    assert push.calls == 0 and not push.activations
    for k in range(1, 5):
        result = sup.tick(observation(k*.02), (.04, 0, 0))
    assert result.phase == Phase.PUSH and push.calls == 1
    for k in range(6, 18):
        result = sup.tick(observation(k*.02, pose=(.04, 0., 0.)), (.04, 0, 0))
        assert result.velocity_world == (0., 0.)
    assert result.phase == Phase.SUCCEEDED


def test_reposition_boundary_zero_no_controller_and_no_geometry_crash():
    sup, push = make()
    result = sup.tick(observation(0, target=ContactTarget('right', 0)), (.04, 0, 0))
    assert result.phase == Phase.REPOSITION_REQUIRED
    assert result.reposition_request.target.face == 'left'
    assert result.velocity_world == (0., 0.) and push.calls == 0
    for k in range(1, 60):
        result = sup.tick(observation(k*.02, target=ContactTarget('right', 0)), (.04, 0, 0))
    assert sup.attempts == 1 and not sup.progress.history
    # Estimate an arbitrary far-away pusher without requiring a valid model state s.
    far = replace(observation(2.), pusher_position=(-.1, .2))
    estimate = sup.monitor.estimate(far, ContactTarget('left', 0))
    assert not estimate.in_bounds


def test_stagnation_reselection_and_failure_context():
    sup, push = make()
    activate(sup)
    for k in range(4, 60):
        result = sup.tick(observation(k*.02), (.04, 0, 0))
        if result.phase == Phase.REPOSITION_REQUIRED:
            break
    assert result.reposition_request.trigger == 'stagnation'
    assert result.target.face != 'left'  # failed primary is temporarily blocked
    assert sup.attempts == 2 and len(sup.blocked) == 1
    assert result.velocity_world == (0., 0.)


def test_microgap_vs_sustained_contact_loss():
    sup, push = make()
    activate(sup)
    small = observation(.08, loaded=False, gap=.00001)
    assert sup.tick(small, (.04, 0, 0)).phase == Phase.PUSH
    for k in range(5, 10):
        result = sup.tick(observation(k*.02, loaded=False, gap=.002), (.04, 0, 0))
        if result.phase == Phase.REPOSITION_REQUIRED:
            break
        assert result.reason == 'contact_loss_pending' and result.velocity_world == (0., 0.)
    assert result.reposition_request.trigger == 'contact_lost'


def test_failed_qp_is_never_commanded():
    push = FakePush(feasible=False, speed=.04)
    sup, _ = make(push=push)
    activate(sup)
    result = sup.tick(observation(.08), (.04, 0, 0))
    assert result.phase == Phase.REPOSITION_REQUIRED
    assert 'solver_failure:primal infeasible' in result.reason
    assert result.push is not None and not result.push.feasible
    assert result.velocity_world == (0., 0.)


def test_motion_ready_is_not_physical_contact_and_same_face_reactivation():
    motion = FakeMotion()
    sup, push = make(motion=motion)
    first = sup.tick(observation(0, target=ContactTarget('right', 0)), (.04, 0, 0))
    assert len(motion.starts) == 1
    second = sup.tick(observation(.02, target=ContactTarget('right', 0)), (.04, 0, 0))
    assert second.phase == Phase.CONFIRM_CONTACT and not push.activations
    for k in range(2, 8):
        result = sup.tick(observation(k*.02, loaded=False), (.04, 0, 0))
    assert result.phase == Phase.CONFIRM_CONTACT and not push.activations
    for k in range(8, 12):
        result = sup.tick(observation(k*.02), (.04, 0, 0))
    assert result.phase == Phase.PUSH and len(push.activations) == 1
    assert len(motion.starts) == 1


def test_motion_running_speed_and_failure_are_explicit():
    motion = FakeMotion()
    sup, _ = make(motion=motion)
    sup.tick(observation(0, target=ContactTarget('right', 0)), (.04, 0, 0))
    motion.next = MotionUpdate((-.01, .005), Phase.RETRACT, MotionStatus.RUNNING, 'retreat')
    result = sup.tick(observation(.02), (.04, 0, 0))
    assert result.phase == Phase.RETRACT and result.velocity_world == (-.01, .005)
    motion.next = MotionUpdate((.2, 0.), Phase.TRANSIT, MotionStatus.RUNNING)
    result = sup.tick(observation(.04), (.04, 0, 0))
    assert result.phase == Phase.FAILED and result.velocity_world == (0., 0.)
    assert 'speed_limit' in result.reason


def test_goal_change_time_checks_timeout_and_reset():
    sup, push = make(config=SupervisorConfig(max_task_seconds=.3))
    activate(sup)
    with pytest.raises(ValueError, match='increasing'):
        sup.tick(observation(.06), (.04, 0, 0))
    result = sup.tick(observation(.08), (-.04, 0, 0))
    assert result.target.face == 'right' and sup.attempts == 1
    result = sup.tick(observation(.4), (-.04, 0, 0))
    assert result.phase == Phase.FAILED and result.reason == 'task_timeout'
    sup.reset()
    assert sup.phase == Phase.SELECT and sup.last_time is None
    assert not sup.events and not sup.blocked


def test_measured_s_and_wrong_face_loading():
    monitor = ContactMonitor(RectangleGeometry())
    target = ContactTarget('left', .02)
    measured = monitor.estimate(observation(0, target=ContactTarget('left', .024)), target)
    assert measured.measured_s == pytest.approx(.024)
    assert not monitor.matches(measured, target)
    wrong = monitor.estimate(observation(0, target=ContactTarget('right', 0)), target)
    # gap is signed against the selected face's supporting line, not global clearance.
    assert not wrong.loaded and wrong.gap < -.1
    assert monitor.estimate(replace(observation(0), pusher_velocity_valid=False), ContactTarget('left', 0)).slip_speed is None


def test_plan_length_and_interface_validation():
    plan = RepositionPlan(ContactTarget('left', 0), ((0., 0.), (.03, .04), (.06, .04)), (0., 0., .3))
    assert plan.length_m == pytest.approx(.08)
    with pytest.raises(ValueError):
        RepositionPlan(ContactTarget('left', 0), ((0., 0.),), (0., 0., 0.))
    with pytest.raises(ValueError):
        MotionUpdate((0., 0.), Phase.PUSH, MotionStatus.READY)
    with pytest.raises(ValueError):
        WorldObservation((0, 0, float('nan')), (0, 0), (0, 0, 0), (0, 0), 0.)


@pytest.mark.parametrize('s,trigger', [(.01, 'contact_location_drift'), (.039, 'contact_out_of_bounds')])
def test_contact_drift_and_edge_guards(s, trigger):
    sup, _ = make()
    activate(sup)
    if s == .039:
        base = observation(.08)
        value = replace(base, pusher_position=(-.07, s))
    else:
        value = observation(.08, target=ContactTarget('left', s))
    result = sup.tick(value, (.04, 0, 0))
    assert result.reposition_request.trigger == trigger
    assert result.velocity_world == (0., 0.)


def test_minimum_hold_and_selection_retry_cap():
    sup, push = make(config=SupervisorConfig(min_contact_hold_seconds=2., max_selection_attempts=1))
    activate(sup)
    for k in range(4, 104):
        result = sup.tick(observation(k*.02), (.04, 0, 0))
        if k*.02 < 2.06-1e-10:
            assert result.phase == Phase.PUSH
    assert result.phase == Phase.FAILED and 'selection_attempt_limit:stagnation' in result.reason
    assert result.velocity_world == (0., 0.)


def test_wrong_side_cannot_drive_active_contact_and_motion_failure():
    sup, _ = make()
    activate(sup)
    result = sup.tick(observation(.08, target=ContactTarget('right', 0)), (.04, 0, 0))
    assert result.reposition_request.trigger == 'selected_face_geometry_invalid'
    motion = FakeMotion()
    sup, _ = make(motion=motion)
    sup.tick(observation(0, target=ContactTarget('right', 0)), (.04, 0, 0))
    motion.next = MotionUpdate((.01, 0.), Phase.TRANSIT, MotionStatus.FAILED, 'no_collision_free_path')
    result = sup.tick(observation(.02), (.04, 0, 0))
    assert result.phase == Phase.FAILED and result.velocity_world == (0., 0.)
    assert 'no_collision_free_path' in result.reason
