"""Step 4 physical repositioning and rule-controller switching demonstrations."""
import argparse
from contextlib import nullcontext
from dataclasses import asdict
import csv
import time
import numpy as np
from .env import EnvConfig, PlanarPushingEnv
from .models.contact_geometry import rotation
from .task import RectangleGeometry, ContactTarget, ContactMonitor, RepositionRequest, MotionStatus, Phase
from .task.adapters import MuJoCoObservationAdapter
from .motion import (VisibilityRepositionPlanner, FeedbackRepositionExecutor, PlannerConfig,
                     MotionConfig, NominalTimeCost, TravelCostConfig)
from .motion.collision import body_point, circle_clearance
from .experiment_io import new_run, save_json, pyplot


def run_motion_case(start_face='right', target_face='left', *, start_s=0., target_s=0.,
                    theta=0., free_start=None, duration=30., viewer=False, env_config=None,
                    planner_config=None, motion_config=None, cost_config=None):
    ecfg = env_config or EnvConfig(pusher_mode='velocity_servo', pusher_servo_gain=500.,
                                   dt_physics=.0002, slider_pose=(0.,0.,theta))
    mc = motion_config or MotionConfig(dt=ecfg.dt_control)
    if not np.isclose(mc.dt, ecfg.dt_control, rtol=0, atol=1e-12):
        raise ValueError('motion dt must match execution dt')
    if ecfg.pusher_mode != 'velocity_servo':
        raise ValueError('physical measured pusher velocity is required')
    if max(mc.retreat_speed, mc.transit_speed, np.hypot(mc.approach_speed,.002)) > ecfg.max_pusher_speed:
        raise ValueError('motion speeds exceed execution limits')
    if (not np.isfinite(duration) or duration < ecfg.dt_control
            or not np.isclose(round(duration/ecfg.dt_control)*ecfg.dt_control, duration)):
        raise ValueError('duration must be a positive integer multiple of execution dt')
    g = RectangleGeometry(*ecfg.slider_size[:2], ecfg.pusher_radius)
    monitor = ContactMonitor(g)
    planner = VisibilityRepositionPlanner(g, planner_config, NominalTimeCost(g, cost_config))
    executor = FeedbackRepositionExecutor(g, planner, monitor, mc)
    start, target = ContactTarget(start_face, start_s), ContactTarget(target_face, target_s)
    env = PlanarPushingEnv(ecfg)
    position = g.center_position(ecfg.slider_pose, start) if free_start is None else free_start
    env.reset(seed=0, pusher_position=position)
    adapter = MuJoCoObservationAdapter(env)
    for _ in range(round(.2/ecfg.dt_control)):
        env.step([0,0])
    if free_start is None:
        for _ in range(round(.6/ecfg.dt_control)):
            obs = adapter.observe()
            _, n, _ = g.frame(start.face)
            env.step(rotation(obs.slider_pose[2]) @ (.002*n))
            if monitor.matches(monitor.estimate(adapter.observe(), start), start):
                break
        else:
            raise RuntimeError('initial contact preload failed')
    obs = adapter.observe()
    initial = obs
    request = RepositionRequest(target, start if free_start is None else None, 'demo', obs.time, obs.slider_pose)
    executor.start(obs, request)
    observations = [obs]
    rows, commands = [], []
    transit_reference = None
    max_transit_drift = 0.
    termination = 'duration_limit'
    context = env.launch_viewer() if viewer else nullcontext()
    with context as window:
        if window is not None:
            window.cam.lookat[:] = (*obs.slider_pose[:2], .01)
            window.cam.distance = .55
            window.cam.elevation = -65
        for _ in range(round(duration/ecfg.dt_control)):
            tick_started = time.perf_counter()
            if window is not None and not window.is_running():
                termination = 'viewer_closed'
                break
            update = executor.update(obs)
            before_clear = circle_clearance(body_point(obs.pusher_position, obs.slider_pose), g)
            env.step(update.velocity_world)
            new_obs = adapter.observe()
            after_clear = circle_clearance(body_point(new_obs.pusher_position, new_obs.slider_pose), g)
            if update.phase == Phase.TRANSIT:
                if transit_reference is None:
                    transit_reference = np.asarray(obs.slider_pose[:2])
                max_transit_drift = max(max_transit_drift, float(np.linalg.norm(np.asarray(new_obs.slider_pose[:2])-transit_reference)))
            estimate = monitor.estimate(new_obs, target)
            rows.append(dict(time_s=obs.time-initial.time, phase=update.phase.value, status=update.status.value,
                reason=update.reason, vx=update.velocity_world[0], vy=update.velocity_world[1],
                clearance_before_m=before_clear, clearance_after_m=after_clear,
                normal_force_N=sum(c.normal_force for c in new_obs.contacts), target_gap_m=estimate.gap,
                target_s_m=estimate.measured_s, target_loaded=estimate.loaded))
            commands.append(update.velocity_world)
            observations.append(new_obs)
            obs = new_obs
            if window is not None:
                window.sync()
                time.sleep(max(0, ecfg.dt_control-(time.perf_counter()-tick_started)))
            if update.status != MotionStatus.RUNNING:
                termination = update.status.value+':'+update.reason
                break
    points = np.array([o.pusher_position for o in observations])
    final = monitor.estimate(obs, target)
    transits = [r for r in rows if r['phase'] == 'transit']
    summary = dict(start=asdict(start), target=asdict(target), theta=ecfg.slider_pose[2],
        initial_pusher_position=initial.pusher_position, final_pusher_position=obs.pusher_position,
        termination=termination, steps=len(rows), duration_s=obs.time-initial.time,
        actual_length_m=float(sum(np.linalg.norm(np.diff(points,axis=0),axis=1))),
        final_contact=asdict(final), final_contact_matches=monitor.matches(final,target),
        min_transit_clearance_m=min((min(r['clearance_before_m'],r['clearance_after_m']) for r in transits),default=None),
        transit_loaded_samples=sum(r['normal_force_N']>=monitor.config.load_threshold for r in transits),
        max_transit_slider_translation_m=max_transit_drift,
        total_slider_translation_m=float(np.linalg.norm(np.subtract(obs.slider_pose[:2],initial.slider_pose[:2]))),
        max_speed_m_s=float(max(np.linalg.norm(v) for v in commands)),
        plans=executor.records, events=executor.events, viewer=viewer,
        configs=dict(env=asdict(ecfg),motion=asdict(mc),planner=asdict(planner.config),
                     cost=asdict(planner.cost_evaluator.config)))
    return dict(summary=summary,observations=observations,rows=rows,commands=np.array(commands),geometry=g)


def save_motion(cases,out):
    plt = pyplot()
    columns = min(3, len(cases))
    rows_count = int(np.ceil(len(cases)/columns))
    fig, grid = plt.subplots(rows_count, columns, figsize=(5*columns, 4.5*rows_count),
                             squeeze=False, constrained_layout=True)
    clearance_fig, clearance_ax = plt.subplots(figsize=(10,5), constrained_layout=True)
    cost_fig, cost_ax = plt.subplots(figsize=(10,5), constrained_layout=True)
    observed_times, estimated_costs = [], []
    for i,(name,case) in enumerate(cases.items()):
        points = np.array([o.pusher_position for o in case['observations']])
        poses = np.array([o.slider_pose for o in case['observations']])
        times = np.array([o.time for o in case['observations']]);times-=times[0]
        np.savez_compressed(out/f'{name}.npz',pusher=points,slider=poses,times=times,commands=case['commands'])
        with (out/f'{name}.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=case['rows'][0].keys());writer.writeheader();writer.writerows(case['rows'])
        color=plt.get_cmap('tab10')(i)
        ax=grid.flat[i]
        g=case['geometry'];theta=poses[0,2];p=poses[0,:2]
        half=np.array([g.length,g.width])/2
        for size,style,label in [(half,'k-','physical slider'),
                                (half+g.pusher_radius+case['summary']['configs']['planner']['clearance'],'k--','planning envelope')]:
            corners=np.array([[-size[0],-size[1]],[size[0],-size[1]],
                              [size[0],size[1]],[-size[0],size[1]],[-size[0],-size[1]]])
            world=corners@rotation(theta).T+p
            ax.plot(world[:,0],world[:,1],style,alpha=.6,label=label)
        for record in case['summary']['plans']:
            plan=np.array(record['plan']['waypoints_world'])
            ax.plot(plan[:,0],plan[:,1],':',color='gray',label='planned to precontact')
        for phase,phase_color in [('retract','tab:orange'),('transit','tab:blue'),('approach','tab:green')]:
            selected=np.array([j for j,r in enumerate(case['rows']) if r['phase']==phase],dtype=int)
            if len(selected):
                ax.plot(points[selected,0],points[selected,1],color=phase_color,label=phase)
        ax.plot(*points[0],'o',color='black',label='start')
        ax.plot(*points[-1],'x',color='red',label='contact')
        ax.set(title=name,xlabel='world x (m)',ylabel='world y (m)')
        ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend(fontsize=7)
        clearance_ax.plot(times[:-1],[r['clearance_after_m'] for r in case['rows']],color=color,label=name)
        observed_times.append(case['summary']['duration_s'])
        plans = case['summary']['plans']
        estimated_costs.append(plans[0]['cost']['total'] if plans and plans[0]['cost'] else np.nan)
    for ax in list(grid.flat)[len(cases):]:ax.set_visible(False)
    fig.suptitle('Step 4: actual Pusher center paths and conservative planning geometry')
    fig.savefig(out/'step4_paths.png',dpi=170);plt.close(fig)
    clearance_ax.axhline(0.,color='black',linestyle='--',label='physical contact')
    clearance_ax.set(xlabel='simulation time (s)',ylabel='circle-to-slider clearance (m)',
                     title='Transit stays separated; contact is allowed during retreat / approach')
    clearance_ax.legend(fontsize=8);clearance_ax.grid(alpha=.25)
    clearance_fig.savefig(out/'step4_clearance.png',dpi=170);plt.close(clearance_fig)
    positions=np.arange(len(cases))
    cost_ax.bar(positions-.18,estimated_costs,width=.36,label='nominal route cost (s, heuristic)')
    cost_ax.bar(positions+.18,observed_times,width=.36,label='observed motion duration (s)')
    cost_ax.set_xticks(positions,list(cases));cost_ax.set_ylabel('seconds');cost_ax.legend();cost_ax.grid(axis='y',alpha=.2)
    cost_fig.savefig(out/'step4_cost.png',dpi=170);plt.close(cost_fig)
    save_json(out/'summary.json',{k:v['summary'] for k,v in cases.items()})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=['opposite','adjacent','same_face','rotated','free_start','all'],default='all')
    p.add_argument('--viewer',action='store_true');p.add_argument('--headless',action='store_true')
    p.add_argument('--output-dir',default='results')
    a=p.parse_args()
    if a.viewer and a.headless:p.error('--viewer and --headless are mutually exclusive')
    out=new_run(a.output_dir,'step4')
    definitions=dict(opposite=dict(start_face='right',target_face='left'),
        adjacent=dict(start_face='left',target_face='bottom',target_s=.018),
        same_face=dict(start_face='left',start_s=-.02,target_face='left',target_s=.02),
        rotated=dict(start_face='top',target_face='right',target_s=-.018,theta=.7),
        free_start=dict(start_face='left',target_face='top',target_s=.02,free_start=(.14,-.12)))
    cases={}
    for name in (definitions if a.case=='all' else [a.case]):
        case=run_motion_case(**definitions[name],viewer=a.viewer);cases[name]=case
        print(name,case['summary']['termination'],'clearance',case['summary']['min_transit_clearance_m'],flush=True)
    save_motion(cases,out);print(out)

if __name__=='__main__':main()
