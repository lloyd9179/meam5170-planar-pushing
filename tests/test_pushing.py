import numpy as np
import pytest

from planar_pushing.demo import run_demo


@pytest.fixture(scope="module")
def experiments():
    return {mode: run_demo(mode) for mode in ("center", "offcenter")}


def assert_stable(states, summary):
    assert summary["completed"]
    assert summary["pusher_contact_steps"] > 50
    assert summary["peak_pusher_normal_force_N"] > .1
    assert summary["minimum_contact_distance_m"] > -.001  # < 1 mm vs 20 mm slider height
    assert summary["max_vertical_drift_m"] < .001
    assert summary["max_tilt_rad"] < .05  # < 2.9 deg: remains planar in practice
    assert all(np.isfinite([*s.slider_pose, *s.slider_linear_velocity,
                           *s.slider_angular_velocity]).all() for s in states)


def test_center_push(experiments):
    states, summary = experiments["center"]
    assert_stable(states, summary)
    assert .04 < summary["delta_x_m"] < .12  # meaningful part of 100 mm commanded travel
    assert abs(summary["delta_y_m"]) < .003
    assert abs(summary["yaw_change_rad"]) < .03  # < 1.7 deg


def test_offcenter_push(experiments):
    states, summary = experiments["offcenter"]
    assert_stable(states, summary)
    assert summary["translation_m"] > .02
    assert -.0 > summary["yaw_change_rad"] > -2.5  # +X force at +Y => negative yaw
    assert abs(summary["yaw_change_rad"]) > .15  # > 8.6 deg, clearly observable
    assert abs(summary["yaw_change_rad"]) > abs(experiments["center"][1]["yaw_change_rad"]) + .12
