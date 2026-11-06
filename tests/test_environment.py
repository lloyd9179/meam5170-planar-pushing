import os
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from planar_pushing import EnvConfig, PlanarPushingEnv


def flattened(state):
    return np.array([state.time, *state.slider_position, state.theta, *state.pusher_position,
                     *state.slider_linear_velocity, *state.slider_angular_velocity])


def test_initialization():
    env = PlanarPushingEnv()
    assert env.model.nq == 7  # one free body, no slider trajectory constraints
    assert env.model.npair == 2
    assert np.all(np.isfinite(flattened(env.get_state())))
    assert env.get_state().time == 0
    assert env.get_state().slider_position[2] - env.config.slider_size[2]/2 == pytest.approx(.0002)
    assert all(c["distance"] >= -1e-8 for c in env.contact_info())
    # Settle onto the actual table under gravity, not a hidden planar constraint.
    for _ in range(30):
        state, info = env.step([0, 0])
    assert state.slider_position[2] == pytest.approx(.01, abs=2e-4)
    contacts = [c for c in info["contacts"] if "table" in c["geoms"]]
    assert contacts
    assert sum(c["normal_force"] for c in contacts) == pytest.approx(.2*9.81, rel=.02)


def test_reset_reproducibility():
    env = PlanarPushingEnv()
    commands = [np.array([.025, .003*np.sin(i*.1)]) for i in range(170)]
    trajectories = []
    for _ in range(2):
        state = env.reset(23, pusher_position=(-.073, .025))
        assert state.time == 0
        assert np.linalg.norm(env.data.qvel) == 0
        states = [flattened(state)]
        for u in commands:
            states.append(flattened(env.step(u)[0]))
        trajectories.append(np.array(states))
    np.testing.assert_allclose(*trajectories, rtol=0, atol=1e-12)
    fresh = PlanarPushingEnv()
    fresh.reset(23, pusher_position=(-.073, .025))
    fresh_states = [flattened(fresh.get_state())]
    for u in commands:
        fresh_states.append(flattened(fresh.step(u)[0]))
    np.testing.assert_allclose(trajectories[0], fresh_states, rtol=0, atol=1e-12)


def test_velocity_and_time():
    env = PlanarPushingEnv()
    initial = env.reset(pusher_position=(-.2, -.2))
    u = np.array([.02, -.01])
    state, info = env.step(u)
    np.testing.assert_allclose(np.subtract(state.pusher_position, initial.pusher_position), u*.02, atol=1e-14)
    assert state.time == pytest.approx(.02)
    assert not info["speed_clipped"]
    assert np.linalg.norm(state.slider_position[:2]) < 1e-10  # no contact, no fake motion


@pytest.mark.parametrize("u", [[1e308, -1e308], [3., 4.]])
def test_speed_saturation(u):
    env = PlanarPushingEnv()
    initial = env.reset(pusher_position=(-.2, -.2))
    state, info = env.step(u)
    assert info["speed_clipped"]
    assert np.linalg.norm(info["applied_velocity"]) == pytest.approx(.08)
    assert np.linalg.norm(np.subtract(state.pusher_position, initial.pusher_position)) == pytest.approx(.08*.02)


@pytest.mark.parametrize("u", [[1], [1, 2, 3], [[0, 0]], [np.nan, 0], [np.inf, 0]])
def test_invalid_control_is_atomic(u):
    env = PlanarPushingEnv()
    before = env.get_state()
    with pytest.raises(ValueError):
        env.step(u)
    assert before == env.get_state()


def test_configurable_geometry_and_reset():
    env = PlanarPushingEnv(EnvConfig(slider_size=(.16, .1, .03), slider_mass=.35,
                                    slider_pose=(.1, -.03, .6), pusher_height=.02))
    np.testing.assert_allclose(env.model.geom("slider_geom").size, [.08, .05, .015])
    assert env.model.body_mass[env.model.body("slider").id] == pytest.approx(.35)
    np.testing.assert_allclose(env.get_state().slider_pose, [.1, -.03, .6])
    state = env.reset(slider_pose=(.2, .1, -.3))
    np.testing.assert_allclose(state.slider_pose, [.2, .1, -.3])
    assert all(c["distance"] >= -1e-8 for c in env.contact_info())
    with pytest.raises(ValueError, match="overlaps"):
        env.reset(pusher_position=(.1, -.03))


@pytest.mark.parametrize("kwargs", [{"dt_control": .0205}, {"dt_physics": .003, "dt_control": .03},
                                         {"slider_mass": 0}, {"mu_ground": -1},
                                         {"pusher_height": .03}, {"slider_size": (.1, -.1, .02)}])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        EnvConfig(**kwargs)


def test_cli_headless_without_display(tmp_path):
    variables = dict(os.environ)
    for name in ("DISPLAY", "WAYLAND_DISPLAY", "MUJOCO_GL"):
        variables.pop(name, None)
    result = subprocess.run([sys.executable, "-m", "planar_pushing.demo", "--mode", "both",
                             "--headless", "--output-dir", str(tmp_path)], env=variables,
                            text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert (tmp_path / "center.csv").is_file()
    assert (tmp_path / "offcenter.csv").is_file()
    assert (tmp_path / "trajectories.png").stat().st_size > 10000
