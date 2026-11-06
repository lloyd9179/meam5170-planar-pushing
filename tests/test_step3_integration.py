from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from planar_pushing import EnvConfig, PlanarPushingEnv
from planar_pushing.models import ModelConfig, QuasiStaticPushingModel
from planar_pushing.control import LocalQPMPC
from planar_pushing.rule_demo import run_rule_demo
from planar_pushing.task import ContactTarget, RectangleGeometry
from planar_pushing.task.adapters import MuJoCoObservationAdapter, LocalMPCPushAdapter
from planar_pushing.task.monitor import ContactMonitor


@pytest.fixture(scope='module')
def demos():
    return {key: run_rule_demo(key) for key in ('translation', 'rotation', 'right', 'rotated', 'reposition', 'stagnant')}


@pytest.mark.parametrize('task', ['translation', 'rotation', 'right', 'rotated'])
def test_rule_mpc_mujoco_success(demos, task):
    case = demos[task]
    s = case['summary']
    assert s['termination'] == 'succeeded'
    assert s['final_position_error_m'] <= .005
    assert s['final_position_error_m'] < .2*s['initial_position_error_m']
    assert s['final_yaw_error_rad'] <= np.deg2rad(3)
    assert s['solve_count'] > 20 and s['solver_failures'] == 0
    assert s['max_constraint_violation'] < 5e-6
    assert max(np.linalg.norm(case['commands'], axis=1)) < .08
    assert any(e.get('reason') == 'contact_confirmed' for e in s['events'])


def test_reposition_request_is_honest_and_no_teleport(demos):
    case = demos['reposition']
    s = case['summary']
    assert s['termination'] == 'reposition_required_step4_unavailable'
    assert s['solve_count'] == 0 and s['steps'] == 1
    assert s['reposition_request']['target']['face'] == 'left'
    np.testing.assert_allclose(case['poses'][0], case['poses'][-1], atol=1e-8)
    p = np.array([o.pusher_position for o in case['observations']])
    np.testing.assert_allclose(p[0], p[-1], atol=1e-8)
    np.testing.assert_array_equal(case['commands'], [[0, 0]])


def test_physical_stagnation_reselects_without_changing_slider(demos):
    case = demos['stagnant']
    s = case['summary']
    assert s['termination'] == 'reposition_required_step4_unavailable'
    assert s['reposition_request']['trigger'] == 'stagnation'
    assert s['selection_attempts'] == 2
    assert s['solve_count'] > 50
    assert s['final_position_error_m'] > s['configs']['supervisor']['position_tolerance']
    # Final request sends zero; no reset/teleport at selection boundaries.
    np.testing.assert_array_equal(case['commands'][-1], [0, 0])
    assert np.max(np.linalg.norm(np.diff(case['poses'][:, :2], axis=0), axis=1)) < .002


def test_true_parameters_independent_and_same_face_activation_resets_warmstart():
    env = PlanarPushingEnv(EnvConfig(mu_ground=.65, mu_pusher=.8, pusher_mode='velocity_servo', dt_physics=.0002))
    model = QuasiStaticPushingModel(ModelConfig(mu_ground_nominal=.1, mu_pusher=.2))
    local = LocalMPCPushAdapter(LocalQPMPC(model))
    before = env.get_friction()
    local.activate(ContactTarget('left', 0))
    # Populate warm-start explicitly, then change only s on the same face.
    assert local.mpc.solve([0, 0, 0, 0], [.04, 0, 0], 'left').feasible
    assert local.mpc._controls is not None
    local.activate(ContactTarget('left', .02))
    assert local.mpc._controls is None and local.theta is None
    assert env.get_friction() == before and before['mu_ground'] == .65
    assert model.config.mu_ground_nominal == .1 and model.config.mu_pusher == .2
    obs = MuJoCoObservationAdapter(env).observe()
    assert obs.pusher_velocity_valid
    # Nominal contact bounds and physical dimensions remain separate.
    physical = RectangleGeometry(env.config.slider_size[0], env.config.slider_size[1], env.config.pusher_radius)
    assert physical.length == .12


def test_cli_without_display_and_unique_run_directories(tmp_path):
    variables = dict(os.environ)
    for key in ('DISPLAY', 'WAYLAND_DISPLAY', 'MUJOCO_GL'):
        variables.pop(key, None)
    for _ in range(2):
        result = subprocess.run([sys.executable, '-m', 'planar_pushing.rule_demo', '--task', 'reposition',
                                 '--headless', '--output-dir', str(tmp_path)],
                                env=variables, text=True, capture_output=True, timeout=30)
        assert result.returncode == 0, result.stderr
    runs = list(tmp_path.glob('step3_*'))
    assert len(runs) == 2
    for path in runs:
        assert json.loads((path/'summary.json').read_text())['reposition']['step4_implemented'] is False
        assert (path/'step3_tracking.png').stat().st_size > 10000


def test_partial_configuration_does_not_couple_true_and_nominal(tmp_path):
    from planar_pushing.rule_demo import load_configs
    path = tmp_path/'config.json'
    path.write_text(json.dumps({'env': {'mu_pusher': .8}, 'model': {'mu_pusher': .2},
                               'mpc': {'horizon': 5}, 'rule': {'rotation_gain': .12}}))
    config = load_configs(path)
    assert config['env_config'].pusher_mode == 'velocity_servo'
    assert config['env_config'].mu_pusher == .8 and config['model_config'].mu_pusher == .2
    assert config['mpc_config'].horizon == 5 and config['rule_config'].rotation_gain == .12
    path.write_text('{"modle": {}}')
    with pytest.raises(ValueError, match='unknown'):
        load_configs(path)


def test_benchmark_goal_independent_of_nominal_ell_and_clock_contract():
    a = run_rule_demo('rotation', duration=.02)
    b = run_rule_demo('rotation', duration=.02, model_config=ModelConfig(ell=.06))
    np.testing.assert_array_equal(a['summary']['goal'], b['summary']['goal'])
    assert a['summary']['initial_selection']['target'] == b['summary']['initial_selection']['target']
    from planar_pushing.control import MPCConfig
    with pytest.raises(ValueError, match='dt must equal'):
        run_rule_demo(env_config=EnvConfig(pusher_mode='velocity_servo'), mpc_config=MPCConfig(dt=.04))
