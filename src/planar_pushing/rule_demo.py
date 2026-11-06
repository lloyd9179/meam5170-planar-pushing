"""Rule controller demo; optional Step 4 physical repositioning port."""
import argparse
from contextlib import nullcontext
from dataclasses import asdict, replace
import csv
import time
import numpy as np
from .env import PlanarPushingEnv, EnvConfig
from .models import QuasiStaticPushingModel, ModelConfig
from .models.contact_geometry import rotation
from .control import LocalQPMPC, MPCConfig
from .experiment_io import new_run, save_json, pyplot
from .task import (RectangleGeometry, RuleContactSelector, RuleSelectionConfig,
    ContactMonitor, ContactMonitorConfig, TaskSupervisor, SupervisorConfig,
    ProgressConfig, WorldObservation, SelectionContext, ContactTarget, Phase)
from .task.adapters import MuJoCoObservationAdapter, LocalMPCPushAdapter
from .task.selection import angle_error
from .task.contracts import CONTRACT_VERSION


def run_rule_demo(task='translation', *, duration=8., env_config=None, model_config=None,
                  mpc_config=None, rule_config=None, monitor_config=None,
                  supervisor_config=None, progress_config=None, geometry_config=None,
                  goal_pose=None, viewer=False, enable_reposition=False,
                  planner_config=None, motion_config=None, travel_cost_config=None,
                  reposition_executor=None):
    if task not in ('translation', 'rotation', 'right', 'rotated', 'reposition', 'stagnant'):
        raise ValueError('unknown task')
    mcfg = mpc_config or MPCConfig()
    # Execution parameters are independent of nominal model friction/geometry.
    ecfg = env_config or EnvConfig(dt_physics=.0002, dt_control=mcfg.dt,
                                   pusher_mode='velocity_servo', pusher_servo_gain=500.)
    pcfg = model_config or ModelConfig()
    scfg = supervisor_config or SupervisorConfig(max_command_speed=ecfg.max_pusher_speed)
    if task == 'stagnant' and supervisor_config is None:
        scfg = replace(scfg, position_tolerance=.0005, angle_tolerance=.005)
    if not np.isfinite(duration) or duration <= 0 or not np.isclose(round(duration/ecfg.dt_control)*ecfg.dt_control, duration):
        raise ValueError('duration must be a positive multiple of control dt')
    if ecfg.pusher_mode != 'velocity_servo':
        raise ValueError('Step 3 requires measured physical pusher velocity; select velocity_servo')
    if not np.isclose(mcfg.dt, ecfg.dt_control, atol=1e-12, rtol=0):
        raise ValueError('MPC dt must equal environment control dt')
    if scfg.max_command_speed > ecfg.max_pusher_speed+1e-12:
        raise ValueError('supervisor speed limit exceeds environment limit')
    model = QuasiStaticPushingModel(pcfg)
    geometry = geometry_config or RectangleGeometry(ecfg.slider_size[0], ecfg.slider_size[1], ecfg.pusher_radius)
    if not np.allclose([geometry.length, geometry.width, geometry.pusher_radius],
                       [*ecfg.slider_size[:2], ecfg.pusher_radius], atol=1e-12, rtol=0):
        raise ValueError('physical geometry must match the actual scene dimensions')
    selector = RuleContactSelector(geometry, rule_config)
    monitor = ContactMonitor(geometry, monitor_config)
    local = LocalMPCPushAdapter(LocalQPMPC(model, mcfg))
    motion_parameters = None
    if enable_reposition and reposition_executor is None:
        from .motion import (VisibilityRepositionPlanner, FeedbackRepositionExecutor,
                             MotionConfig, PlannerConfig, NominalTimeCost, TravelCostConfig)
        motion_cfg = motion_config or MotionConfig(dt=ecfg.dt_control)
        if not np.isclose(motion_cfg.dt, ecfg.dt_control, atol=1e-12, rtol=0):
            raise ValueError('motion dt must equal environment control dt')
        if max(motion_cfg.retreat_speed, motion_cfg.transit_speed,
               np.hypot(motion_cfg.approach_speed, .002)) > scfg.max_command_speed:
            raise ValueError('motion speeds exceed the shared command limit')
        planner_cfg = planner_config or PlannerConfig()
        cost_cfg = travel_cost_config or TravelCostConfig(
            retreat_speed=motion_cfg.retreat_speed, transit_speed=motion_cfg.transit_speed,
            approach_speed=motion_cfg.approach_speed, fixed_settle_seconds=motion_cfg.settle_seconds)
        planner = VisibilityRepositionPlanner(geometry, planner_cfg, NominalTimeCost(geometry, cost_cfg))
        reposition_executor = FeedbackRepositionExecutor(geometry, planner, monitor, motion_cfg)
        motion_parameters = dict(planner=asdict(planner_cfg), motion=asdict(motion_cfg), travel_cost=asdict(cost_cfg))
    motion_enabled = reposition_executor is not None
    supervisor = TaskSupervisor(selector, local, monitor, config=scfg, progress_config=progress_config,
                                reposition_executor=reposition_executor)
    pose = np.array(ecfg.slider_pose)
    # Fixed task fixture from the Step 2 nominal reachable-pose experiment.
    # Changing nominal ell/friction must NOT silently change the benchmark goal.
    local_goal = (np.array([.03293194288959019, -.009601306146257595, -.1255813953488375])
                  if task in ('rotation', 'stagnant') else np.array([.04, 0, 0.]))
    if task == 'right':
        local_goal = np.array([-.04, 0., 0.])
    if task == 'rotated' and env_config is None:
        pose[2] = .6
    goal = np.r_[pose[:2]+rotation(pose[2]) @ local_goal[:2], pose[2]+local_goal[2]]
    if goal_pose is not None:
        from .task.contracts import finite_tuple
        goal = np.array(finite_tuple(goal_pose, 3, 'goal_pose'))
    initial = WorldObservation(tuple(pose), (0., 0.), (0., 0., 0.), (0., 0.), 0.)
    initial_selection = selector.select(initial, tuple(goal), SelectionContext())
    target = initial_selection.target
    start_target = ContactTarget('right', 0.) if task == 'reposition' else target
    ecfg = replace(ecfg, slider_pose=tuple(pose), goal_pose=tuple(goal))
    env = PlanarPushingEnv(ecfg)
    # Initial conditions only; no runtime code in selector/supervisor can reset env.
    env.reset(seed=0, pusher_position=geometry.center_position(pose, start_target))
    observation_adapter = MuJoCoObservationAdapter(env)
    for _ in range(round(.2/ecfg.dt_control)):
        env.step([0., 0.])
    if task != 'reposition':
        # Initialization approach uses actual velocity commands; no slider writes.
        for _ in range(round(.6/ecfg.dt_control)):
            observation = observation_adapter.observe()
            _, n, _ = geometry.frame(target.face)
            env.step(rotation(observation.slider_pose[2]) @ (.002*n))
            if monitor.matches(monitor.estimate(observation_adapter.observe(), target), target):
                break
        else:
            raise RuntimeError('initial loaded contact could not be established')
    observation = observation_adapter.observe()
    initial_observation = observation
    observations = [observation]
    rows = []
    commands = []
    run_started = observation.time
    window_context = env.launch_viewer() if viewer else nullcontext()
    termination = 'duration_limit'
    with window_context as window:
        if window is not None:
            window.cam.lookat[:] = (*pose[:2], .01)
            window.cam.distance = .5
            window.cam.elevation = -65
        for _ in range(round(duration/ecfg.dt_control)):
            start = time.perf_counter()
            if window is not None and not window.is_running():
                termination = 'viewer_closed'
                break
            decision = supervisor.tick(observation, tuple(goal))
            estimate = decision.contact
            push = decision.push
            row = dict(time_s=observation.time-run_started, phase=decision.phase.value,
                       reason=decision.reason, face=decision.target.face.value if decision.target else '',
                       desired_s=decision.target.s if decision.target else None,
                       measured_s=estimate.measured_s if estimate else None,
                       gap=estimate.gap if estimate else None, loaded=estimate.loaded if estimate else None,
                       slip_speed=estimate.slip_speed if estimate else None,
                       slip_flag=bool(estimate and estimate.slip_speed is not None
                                      and abs(estimate.slip_speed) > monitor.config.slip_flag_speed),
                       vx=decision.velocity_world[0], vy=decision.velocity_world[1],
                       solver_status=push.status if push else '', solve_time_s=push.solve_time if push else None,
                       solver_feasible=push.feasible if push else None,
                       iterations=push.iterations if push else None,
                       constraint_violation=push.constraint_violation if push else None,
                       objective=push.objective if push else None,
                       progress_metric=supervisor.last_progress.metric if supervisor.last_progress else None,
                       progress_decrease=supervisor.last_progress.decrease if supervisor.last_progress else None)
            # Raw physical diagnostics remain available during motion phases,
            # where there is deliberately no active pushing contact estimate.
            from .motion.collision import body_point, circle_clearance
            row['physical_clearance_m'] = circle_clearance(
                body_point(observation.pusher_position, observation.slider_pose), geometry)
            row['physical_normal_force_N'] = sum(c.normal_force for c in observation.contacts)
            rows.append(row)
            commands.append(decision.velocity_world)
            env.step(decision.velocity_world)
            observation = observation_adapter.observe()
            observations.append(observation)
            if window is not None:
                window.sync()
                time.sleep(max(0., ecfg.dt_control-(time.perf_counter()-start)))
            if decision.terminal:
                termination = decision.phase.value
                break
            if decision.phase == Phase.REPOSITION_REQUIRED and not motion_enabled:
                termination = 'reposition_required_step4_unavailable'
                break
    poses = np.array([o.slider_pose for o in observations])
    errors = np.linalg.norm(poses[:, :2]-goal[:2], axis=1)
    angular = abs(np.arctan2(np.sin(poses[:, 2]-goal[2]), np.cos(poses[:, 2]-goal[2])))
    solved = [r for r in rows if r['solver_status']]
    summary = dict(task=task, contract_version=CONTRACT_VERSION, termination=termination, initial_pose=initial_observation.slider_pose,
        final_pose=observation.slider_pose, goal=goal, initial_position_error_m=float(errors[0]),
        final_position_error_m=float(errors[-1]), final_yaw_error_rad=float(angular[-1]),
        steps=len(rows), solve_count=len(solved), solver_failures=sum(not r['solver_feasible'] for r in solved),
        mean_solve_time_ms=float(np.mean([r['solve_time_s'] for r in solved])*1000) if solved else None,
        max_constraint_violation=max((r['constraint_violation'] for r in solved if r['constraint_violation'] is not None), default=None),
        slip_flag_steps=sum(r['slip_flag'] for r in rows), initial_selection=asdict(initial_selection),
        unloaded_contact_samples=sum(r['loaded'] is False for r in rows),
        max_s_drift_m=max((abs(r['measured_s']-r['desired_s']) for r in rows
                          if r['measured_s'] is not None and r['desired_s'] is not None), default=None),
        events=supervisor.events, selection_attempts=supervisor.attempts,
        reposition_request=asdict(supervisor.request) if supervisor.request else None,
        configs=dict(env=asdict(ecfg), model=asdict(pcfg), mpc=asdict(mcfg),
                     geometry=asdict(geometry),
                     rule=asdict(selector.config), monitor=asdict(monitor.config),
                     supervisor=asdict(scfg), progress=asdict(supervisor.progress.config)),
        motion_configs=motion_parameters,
        motion_records=getattr(reposition_executor, 'records', []),
        motion_events=getattr(reposition_executor, 'events', []),
        viewer=viewer, step4_implemented=motion_enabled, step4_enabled=motion_enabled)
    return dict(summary=summary, observations=observations, rows=rows, poses=poses,
                errors=errors, angular_errors=angular, commands=np.array(commands), goal=goal)


def save_results(cases, out):
    plt = pyplot()
    for key, case in cases.items():
        np.savez_compressed(out/f'{key}.npz', poses=case['poses'],
            pusher_positions=np.array([o.pusher_position for o in case['observations']]),
            times=np.array([o.time for o in case['observations']]), commands=case['commands'],
            position_errors=case['errors'], angular_errors=case['angular_errors'], goal=case['goal'])
        with (out/f'{key}.csv').open('w') as f:
            writer = csv.DictWriter(f, fieldnames=case['rows'][0].keys())
            writer.writeheader()
            writer.writerows(case['rows'])
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
    for index, (key, case) in enumerate(cases.items()):
        color = plt.get_cmap('tab10')(index)
        times = np.array([o.time for o in case['observations']])
        times -= times[0]
        axes[0, 0].plot(case['poses'][:, 0], case['poses'][:, 1], label=key, color=color)
        axes[0, 0].plot(*case['goal'][:2], 'x', color=color)
        axes[0, 1].plot(times, case['errors'], label=key, color=color)
        axes[1, 0].plot(times, case['angular_errors'], label=key, color=color)
        phases = [list(Phase).index(Phase(r['phase'])) for r in case['rows']]
        axes[1, 1].step(times[:-1], phases, where='post', label=key, color=color)
        axes[1, 1].scatter(times[-2], phases[-1], s=18, color=color)
    for ax, label in zip(axes.flat, ['slider trajectory x/y (m)', 'position error (m)', '|yaw error| (rad)', 'execution phase']):
        ax.set_title(label)
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    axes[1, 1].set_yticks(range(len(Phase)), [p.value for p in Phase], fontsize=7)
    for ax in [axes[0, 1], axes[1, 0], axes[1, 1]]:
        ax.set_xlabel('simulation time (s)')
    fig.suptitle('Step 3 rule policy + shared supervisor + local QP-MPC')
    fig.savefig(out/'step3_tracking.png', dpi=160)
    plt.close(fig)
    save_json(out/'summary.json', {key: case['summary'] for key, case in cases.items()})


def load_configs(path):
    """Partial JSON overrides; unknown sections/fields fail instead of being ignored."""
    import json
    from pathlib import Path
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('configuration must be a JSON object')
    from .motion import PlannerConfig, MotionConfig, TravelCostConfig
    factories = dict(env=EnvConfig, model=ModelConfig, mpc=MPCConfig, rule=RuleSelectionConfig,
                     monitor=ContactMonitorConfig, supervisor=SupervisorConfig, progress=ProgressConfig,
                     geometry=RectangleGeometry, planner=PlannerConfig, motion=MotionConfig,
                     travel_cost=TravelCostConfig)
    unknown = set(data)-set(factories)-{'goal_pose'}
    if unknown:
        raise ValueError(f'unknown configuration sections: {sorted(unknown)}')
    def immutable(value):
        return tuple(immutable(v) for v in value) if isinstance(value, list) else value
    result = {}
    for section, cls in factories.items():
        if section not in data:
            continue
        if not isinstance(data[section], dict):
            raise ValueError(f'{section} must be an object')
        values = {key: immutable(value) for key, value in data[section].items()}
        if section == 'env':
            values = {'pusher_mode': 'velocity_servo', 'dt_physics': .0002,
                      'pusher_servo_gain': 500., **values}
        if section == 'geometry':
            env = result.get('env_config', EnvConfig())
            values = {'length': env.slider_size[0], 'width': env.slider_size[1],
                      'pusher_radius': env.pusher_radius, **values}
        result[section+'_config'] = cls(**values)
    if 'goal_pose' in data:
        from .task.contracts import finite_tuple
        result['goal_pose'] = finite_tuple(data['goal_pose'], 3, 'goal_pose')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['translation', 'rotation', 'right', 'rotated', 'reposition', 'stagnant', 'all'], default='all')
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--with-reposition', action='store_true', help='enable the physical Step 4 motion port')
    parser.add_argument('--config', help='JSON partial overrides for independent component configs')
    parser.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'YAW'), help='world-frame goal pose')
    display = parser.add_mutually_exclusive_group()
    display.add_argument('--viewer', action='store_true')
    display.add_argument('--headless', action='store_true')
    parser.add_argument('--output-dir', default='results')
    args = parser.parse_args()
    try:
        configs = load_configs(args.config) if args.config else {}
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    if args.goal is not None:
        configs['goal_pose'] = tuple(args.goal)
    if 'goal_pose' in configs and args.task == 'all':
        parser.error('select a single --task when supplying a goal')
    tasks = ['translation', 'rotation', 'right', 'rotated', 'reposition', 'stagnant'] if args.task == 'all' else [args.task]
    out = new_run(args.output_dir, 'step3')
    cases = {}
    for task in tasks:
        case = run_rule_demo(task, duration=args.duration, viewer=args.viewer,
                             enable_reposition=args.with_reposition, **configs)
        cases[task] = case
        s = case['summary']
        print(task, s['termination'], s['final_position_error_m'], s['final_yaw_error_rad'], flush=True)
    save_results(cases, out)
    print(out)


if __name__ == '__main__':
    main()
