"""Step 2 closed loop, identical MPC with model-only or MuJoCo execution."""
import argparse
from contextlib import nullcontext
from dataclasses import asdict
import csv
import time
import numpy as np
from .models import QuasiStaticPushingModel
from .control import LocalQPMPC,MPCConfig
from .integration import prepare_contact,model_state,world_velocity,contact_diagnostics,wrap_angle
from .experiment_io import new_run,save_json,pyplot


def task_setup(task,model):
    if task=='translation': return 0.,np.array([.04,0,0])
    if task!='rotation': raise ValueError('task must be translation or rotation')
    # Reachable-pose construction only: open-loop nominal model defines the goal.
    # The resulting goal is fixed; no reference controls/trajectory are sent to execution.
    x=np.array([0.,0,0,.02])
    for _ in range(150): x=model.discrete_dynamics(x,[.012,0],.02,'left')
    return .02,x[:3]


def run_closed_loop(task='translation',backend='mujoco',*,duration=6.,mpc_config=None,viewer=False):
    if backend not in ('model','mujoco'): raise ValueError('backend')
    if viewer and backend!='mujoco': raise ValueError('viewer is only available for MuJoCo')
    if not np.isfinite(duration) or duration<=0: raise ValueError('duration')
    model=QuasiStaticPushingModel();cfg=mpc_config or MPCConfig();mpc=LocalQPMPC(model,cfg)
    if not np.isclose(round(duration/cfg.dt)*cfg.dt,duration):
        raise ValueError('duration must be a multiple of MPC dt')
    s,goal=task_setup(task,model);face='left';env=None
    if backend=='mujoco': env,x=prepare_contact(model,face,s,dt=cfg.dt,goal=goal)
    else: x=np.array([0.,0,0,s])
    xs=[x.copy()];controls=[];predictions=[];logs=[];reason='duration_complete';missing=0
    initial=x.copy();start=time.perf_counter();window_context=env.launch_viewer() if viewer else nullcontext()
    with window_context as window:
        if window is not None:
            window.cam.lookat[:]=[.02,0,.01];window.cam.distance=.5;window.cam.elevation=-65
        for k in range(round(duration/cfg.dt)):
            iteration_start=time.perf_counter()
            if window is not None and not window.is_running(): reason='viewer_closed';break
            r=mpc.solve(x,goal,face)
            row=dict(step=k,time_s=k*cfg.dt,status=r.status,feasible=r.feasible,
                     solve_time_s=r.solve_time,solver_time_s=r.solver_time,iterations=r.iterations,
                     objective=r.objective,constraint_violation=r.constraint_violation,
                     fallback=r.fallback_used,warm_started=r.warm_started,
                     vn_m_s=r.control[0],vt_m_s=r.control[1],
                     sticking_feasible=model.is_sticking_feasible(x,r.control,face,cfg.acceptance_tolerance))
            if not r.feasible:
                # Zero fallback is explicit; do not execute invalid optimization iterates.
                if env is not None: env.step([0,0])
                row.update(contact=False,slip_flag=False,contact_point_error_m=None,slip_speed_m_s=None)
                logs.append(row);reason='solver_fallback';break
            command=world_velocity(model,x,r.control,face)
            if env is not None:
                state,info=env.step(command)
                x_next=model_state(env,model,face,previous_theta=x[2])
                d=contact_diagnostics(env,model,x_next,face,info['applied_velocity'])
                row.update(contact=d['contact'],gap_m=d['gap'],slip_flag=d['contact'] and abs(d['slip_speed'])>.002,
                           contact_point_error_m=d['contact_point_error'],slip_speed_m_s=d['slip_speed'])
                row['world_vx_m_s'],row['world_vy_m_s']=info['applied_velocity']
                row['speed_clipped']=info['speed_clipped']
                missing=missing+1 if not d['contact'] and d['gap']>.0005 else 0
            else:
                x_next=model.discrete_dynamics(x,r.control,cfg.dt,face)
                row.update(contact=True,slip_flag=False,contact_point_error_m=0.,slip_speed_m_s=0.)
                row['world_vx_m_s'],row['world_vy_m_s']=command;row['speed_clipped']=False
            controls.append(r.control);predictions.append(r.predicted_states);x=x_next;xs.append(x.copy());logs.append(row)
            if window is not None:
                window.sync();time.sleep(max(0,cfg.dt-(time.perf_counter()-iteration_start)))
            if missing>=3:
                env.step([0,0]);reason='contact_lost';break
            low,high=model.contact_bounds(face)
            if not low<=x[3]<=high:
                if env is not None: env.step([0,0])
                reason='contact_location_out_of_bounds';break
    xs=np.array(xs);controls=np.array(controls).reshape(-1,2)
    error=np.linalg.norm(xs[:,:2]-goal[:2],axis=1);angle=abs(wrap_angle(xs[:,2]-goal[2]))
    solved=[l for l in logs if l['feasible']]
    summary=dict(task=task,backend=backend,goal=goal,initial=initial,final=xs[-1],
        initial_position_error_m=float(error[0]),final_position_error_m=float(error[-1]),
        initial_yaw_error_rad=float(angle[0]),final_yaw_error_rad=float(angle[-1]),
        termination=reason,execution_steps=len(controls),solve_count=len(logs),
        fallbacks=sum(l['fallback'] for l in logs),slip_flag_steps=sum(l['slip_flag'] for l in logs),
        contact_lost_steps=sum(not l['contact'] for l in logs),
        sticking_infeasible_commands=sum(not l['sticking_feasible'] for l in logs),
        max_s_drift_m=float(np.max(abs(xs[:,3]-initial[3]))),
        mean_solve_time_ms=float(np.mean([l['solve_time_s'] for l in logs])*1000),
        max_solve_time_ms=float(np.max([l['solve_time_s'] for l in logs])*1000),
        max_constraint_violation=max((l['constraint_violation'] for l in solved),default=None),
        env_config=asdict(env.config) if env is not None else None,
        total_wall_s=time.perf_counter()-start,model_config=asdict(model.config),mpc_config=asdict(cfg))
    return dict(states=xs,controls=controls,predictions=np.array(predictions),logs=logs,summary=summary,
                position_error=error,angle_error=angle,dt=cfg.dt)


def save_and_plot(cases,out):
    plt=pyplot()
    for key,d in cases.items():
        np.savez_compressed(out/f'{key}.npz',states=d['states'],controls=d['controls'],
                            predictions=d['predictions'],position_error=d['position_error'],angle_error=d['angle_error'])
        with (out/f'{key}_solver_contact.csv').open('w') as f:
            keys=list(dict.fromkeys(key for row in d['logs'] for key in row))
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(d['logs'])
    for task in ('translation','rotation'):
        selected={k:d for k,d in cases.items() if d['summary']['task']==task}
        if not selected:continue
        fig,axs=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
        for key,d in selected.items():
            s=d['states'];t=np.arange(len(s))*d['dt'];goal=d['summary']['goal']
            axs[0,0].plot(s[:,0],s[:,1],label=key)
            if len(d['predictions']):
                prediction=d['predictions'][min(10,len(d['predictions'])-1)]
                axs[0,0].plot(prediction[:,0],prediction[:,1],'--',alpha=.7,label=f'{key} horizon @ k<=10')
            axs[0,1].plot(t,d['position_error'],label=key)
            axs[1,0].plot(t,s[:,2],label=key);axs[1,0].axhline(goal[2],color='k',linestyle=':')
            axs[1,1].plot(t,d['angle_error'],label=key)
        axs[0,0].plot(goal[0],goal[1],'kx',ms=9,label='goal')
        for ax,title in zip(axs.flat,['trajectory x/y (m)','position error (m)','yaw (rad)','|yaw error| (rad)']):
            ax.set_title(title);ax.legend(fontsize=7);ax.grid(alpha=.25)
        for ax in (axs[0,1],axs[1,0],axs[1,1]):ax.set_xlabel('time (s)')
        fig.suptitle(f'Fixed LEFT face: {task}; model-only vs MuJoCo')
        fig.savefig(out/f'step2_{task}_tracking.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
    for key,d in cases.items():
        t=np.arange(len(d['controls']))*d['dt']
        for j in range(2):axs[j].plot(t,d['controls'][:,j],label=key)
    for ax,label in zip(axs,['vn (m/s)','vt (m/s)']):ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend(fontsize=8)
    axs[-1].set_xlabel('time (s)');fig.savefig(out/'step2_control_inputs.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5),constrained_layout=True)
    for key,d in cases.items():ax.plot([r['time_s'] for r in d['logs']],[r['solve_time_s']*1000 for r in d['logs']],label=key,alpha=.8)
    ax.axhline(20,color='k',linestyle='--',label='20 ms control interval (not real-time guarantee)')
    ax.set(xlabel='time (s)',ylabel='build + setup + solve wall time (ms)');ax.legend(fontsize=8);ax.grid(alpha=.25)
    fig.savefig(out/'step2_solver_time.png',dpi=160);plt.close(fig)
    save_json(out/'summary.json',{k:d['summary'] for k,d in cases.items()})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['translation','rotation','both'],default='both')
    p.add_argument('--backend',choices=['model','mujoco','both'],default='both')
    p.add_argument('--duration',type=float,default=6.)
    p.add_argument('--horizon',type=int,default=15);p.add_argument('--dt',type=float,default=.02)
    p.add_argument('--viewer',action='store_true');p.add_argument('--headless',action='store_true')
    p.add_argument('--output-dir',default='results')
    a=p.parse_args()
    if a.viewer and a.headless:p.error('--viewer and --headless are mutually exclusive')
    if a.viewer and a.backend!='mujoco':p.error('--viewer requires --backend mujoco')
    tasks=['translation','rotation'] if a.task=='both' else [a.task]
    backends=['model','mujoco'] if a.backend=='both' else [a.backend]
    out=new_run(a.output_dir,'step2');cases={}
    for backend in backends:
        for task in tasks:
            key=f'{backend}_{task}'
            cases[key]=run_closed_loop(task,backend,duration=a.duration,viewer=a.viewer,
                                       mpc_config=MPCConfig(horizon=a.horizon,dt=a.dt))
            print(key,{k:cases[key]['summary'][k] for k in ['final_position_error_m','final_yaw_error_rad','termination','fallbacks']},flush=True)
    save_and_plot(cases,out);print(out)

if __name__=='__main__':main()
