from dataclasses import replace
import numpy as np
import pytest
from planar_pushing.models.contact_geometry import rotation
from planar_pushing.task import (RectangleGeometry, RuleContactSelector, RuleSelectionConfig,
    WorldObservation, SelectionContext, ContactTarget, ProgressConfig, StagnationDetector)


def obs(pose=(0., 0., 0.), time=0.):
    return WorldObservation(pose, (-.07, 0.), (0., 0., 0.), (0., 0.), time)


@pytest.mark.parametrize('goal,face', [((.04, 0, 0), 'left'), ((-.04, 0, 0), 'right'),
                                       ((0, .04, 0), 'bottom'), ((0, -.04, 0), 'top')])
def test_rule_faces_and_center(goal, face):
    result = RuleContactSelector(RectangleGeometry()).select(obs(), goal, SelectionContext())
    assert result.target.face == face
    assert result.target.s == 0
    assert 'primary' in result.reason


@pytest.mark.parametrize('yaw,sign', [(.2, -1), (-.2, 1)])
def test_rotation_rule_torque_sign_and_clamping(yaw, sign):
    g = RectangleGeometry()
    selector = RuleContactSelector(g)
    target = selector.select(obs(), (.04, 0, yaw), SelectionContext()).target
    r0, n, t = g.frame(target.face)
    r = r0+target.s*t
    torque = r[0]*n[1]-r[1]*n[0]
    assert np.sign(torque) == np.sign(yaw)
    assert np.sign(target.s) == sign
    assert abs(target.s) <= .6*g.bounds(target.face)[1]
    wrapped = selector.select(obs(), (.04, 0, yaw+2*np.pi), SelectionContext()).target
    assert wrapped.face == target.face and wrapped.s == pytest.approx(target.s)


def test_rule_rotation_invariance_and_tie_order():
    selector = RuleContactSelector(RectangleGeometry())
    angle = .7
    point = rotation(angle) @ [.04, 0]
    target = selector.select(obs((.1, -.1, angle)), (* (point+[.1, -.1]), angle), SelectionContext()).target
    assert target.face == 'left' and target.s == pytest.approx(0)
    target = selector.select(obs(), (0, 0, .2), SelectionContext(ContactTarget('right', 0))).target
    assert target.face == 'right'
    first = selector.select(obs(), (.04, .04, 0), SelectionContext())
    assert first.target.face == 'left'


def test_hysteresis_and_finite_rule_fallbacks():
    selector = RuleContactSelector(RectangleGeometry())
    context = SelectionContext(ContactTarget('bottom', 0))
    assert selector.select(obs(), (.04, .039, 0), context).target.face == 'bottom'
    blocked = []
    for _ in range(4):
        result = selector.select(obs(), (.04, 0, 0), SelectionContext(blocked=tuple(blocked), trigger='stagnation'))
        assert result.target not in blocked
        blocked.append(result.target)
    assert selector.select(obs(), (.04, 0, 0), SelectionContext(blocked=tuple(blocked))).target is None


def test_stagnation_actual_time_window_noise_progress_and_reset():
    detector = StagnationDetector()
    for k in range(51):
        report = detector.update(obs((.00001*np.sin(k), 0., 0.), k*.02), (.04, 0, 0))
        if k < 50:
            assert not report.ready
    assert report.stalled
    detector.reset()
    for k in range(51):
        report = detector.update(obs((k*.0004, 0., 0.), k*.02), (.04, 0, 0))
    assert report.ready and not report.stalled and report.decrease > report.threshold
    with pytest.raises(ValueError, match='timestamps'):
        detector.update(obs(time=1.), (.04, 0, 0))
    detector.reset()
    # A different sampling frequency still requires a full simulation-time window.
    for k in range(21):
        report = detector.update(obs(time=k*.05), (.04, 0, 0))
    assert report.stalled


@pytest.mark.parametrize('kwargs', [{'max_offset_fraction': 2}, {'rotation_gain': -.1}, {'face_hysteresis': np.nan}])
def test_invalid_rule_settings(kwargs):
    with pytest.raises(ValueError):
        RuleSelectionConfig(**kwargs)
