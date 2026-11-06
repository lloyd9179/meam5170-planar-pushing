"""Step 1: measured-state one-step and realized-world-input open-loop comparison."""
import argparse
from dataclasses import asdict
import numpy as np
from .models import ModelConfig, QuasiStaticPushingModel
from .integration import prepare_contact, model_state, world_velocity, local_velocity, contact_diagnostics, wrap_angle
from .experiment_io import new_run, save_json, pyplot


def compare(model, s, speed, duration=3., dt=.02, face='left'):
    if (not np.isfinite([speed,duration,dt]).all() or min(speed,duration,dt)<=0
            or not np.isclose(round(duration/dt)*dt,duration)):
        raise ValueError('speed/duration/dt must be positive; duration must be a multiple of dt')
    env, x = prepare_contact(model,face,s,dt=dt)
    initial=x.copy(); predicted=x.copy()
    actual=[x.copy()]; openloop=[x.copy()]; onestep=[]; diagnostics=[]; commands=[]
    for _ in range(round(duration/dt)):
        cmd=world_velocity(model,x,[speed,0],face)
        one=model.discrete_dynamics(x,[speed,0],dt,face)
        # Replay identical world center commands into the open-loop model.
        predicted=model.discrete_dynamics(predicted, local_velocity(model,predicted,cmd,face),dt,face)
        new_state,info=env.step(cmd)
        updated=model_state(env,model,face,previous_theta=x[2])
        diagnostics.append(contact_diagnostics(env,model,updated,face,info['applied_velocity']))
        onestep.append(one); x=updated
        actual.append(x.copy()); openloop.append(predicted.copy()); commands.append(cmd)
    actual,openloop,onestep=np.array(actual),np.array(openloop),np.array(onestep)
    err1=onestep-actual[1:]; err1[:,2]=wrap_angle(err1[:,2])
    errN=openloop-actual; errN[:,2]=wrap_angle(errN[:,2])
    summary=dict(s=s,speed_m_s=speed,duration_s=duration,pusher_mode=env.config.pusher_mode,
        dt_physics=env.config.dt_physics,pusher_servo_gain=env.config.pusher_servo_gain,
        one_step_position_rmse_m=float(np.sqrt(np.mean(np.sum(err1[:,:2]**2,axis=1)))),
        one_step_yaw_rmse_rad=float(np.sqrt(np.mean(err1[:,2]**2))),
        open_loop_final_position_error_m=float(np.linalg.norm(errN[-1,:2])),
        open_loop_final_yaw_error_rad=float(abs(errN[-1,2])),
        actual_delta=(actual[-1]-initial).tolist(), predicted_delta=(openloop[-1]-initial).tolist(),
        contact_lost_steps=sum(not d['contact'] for d in diagnostics),
        slip_flag_steps=sum(d['contact'] and abs(d['slip_speed'])>.002 for d in diagnostics),
        max_s_drift_m=float(np.max(np.abs(actual[:,3]-initial[3]))),
        max_contact_point_error_m=max((d['contact_point_error'] for d in diagnostics if d['contact']),default=None),
        assumptions='Point velocity approximation; slip flag threshold 2 mm/s; no trajectory filtering')
    return dict(actual=actual,openloop=openloop,onestep=onestep,commands=np.array(commands),
                err1=err1,errN=errN,diagnostics=diagnostics,summary=summary,dt=dt)


def run_validation(base='results', duration=3.):
    model=QuasiStaticPushingModel(); out=new_run(base,'step1')
    cases={}
    for label,s in [('center',0.),('offcenter',.02)]:
        for speed in [.005,.01,.02]:
            key=f'{label}_{speed:.3f}'; d=compare(model,s,speed,duration)
            cases[key]=d
            np.savez_compressed(out/f'{key}.npz', **{k:v for k,v in d.items() if isinstance(v,np.ndarray)})
            # Full contact diagnostics preserve flagged/failed-assumption samples.
            import csv
            with (out/f'{key}_contact.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=d['diagnostics'][0].keys());writer.writeheader();writer.writerows(d['diagnostics'])
    plt=pyplot()
    for label in ['center','offcenter']:
        d=cases[f'{label}_0.010']; t=np.arange(len(d['actual']))*d['dt']
        fig,axs=plt.subplots(3,2,figsize=(11,9),constrained_layout=True)
        for j,name in enumerate(['x (m)','y (m)','yaw (rad)']):
            axs[j,0].plot(t,d['actual'][:,j],label='MuJoCo')
            axs[j,0].plot(t,d['openloop'][:,j],'--',label='open-loop: same world input')
            axs[j,0].plot(t[1:],d['onestep'][:,j],':',label='one-step from measured state')
            axs[j,0].set_ylabel(name);axs[j,0].legend(fontsize=8)
        axs[0,1].plot(d['actual'][:,0],d['actual'][:,1],label='MuJoCo')
        axs[0,1].plot(d['openloop'][:,0],d['openloop'][:,1],'--',label='prediction')
        axs[0,1].set(xlabel='x (m)',ylabel='y (m)');axs[0,1].legend()
        axs[1,1].plot(t[1:],np.linalg.norm(d['err1'][:,:2],axis=1),label='one-step')
        axs[1,1].plot(t,np.linalg.norm(d['errN'][:,:2],axis=1),label='open-loop')
        axs[1,1].set_ylabel('position error (m)');axs[1,1].legend()
        axs[2,1].plot(t[1:],abs(d['err1'][:,2]),label='one-step')
        axs[2,1].plot(t,abs(d['errN'][:,2]),label='open-loop')
        axs[2,1].set_ylabel('|yaw error| (rad)');axs[2,1].legend()
        for ax in axs.flat: ax.grid(alpha=.25)
        fig.suptitle(f'{label}: 0.010 m/s; all samples retained')
        fig.savefig(out/f'step1_{label}_push.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(10,7),constrained_layout=True)
    for label in ['center','offcenter']:
        ds=[cases[f'{label}_{v:.3f}']['summary'] for v in [.005,.01,.02]]
        for ax,key in zip(axs.flat,['one_step_position_rmse_m','one_step_yaw_rmse_rad',
                                    'open_loop_final_position_error_m','open_loop_final_yaw_error_rad']):
            ax.plot([.005,.01,.02],[d[key] for d in ds],'o-',label=label);ax.set_title(key,fontsize=9);ax.set_xlabel('speed (m/s)');ax.legend();ax.grid(alpha=.25)
    fig.savefig(out/'step1_prediction_error.png',dpi=160);plt.close(fig)
    save_json(out/'summary.json',dict(model_config=asdict(model.config),dt=.02,cases={k:v['summary'] for k,v in cases.items()}))
    print(out)
    return out,cases


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',default='results');p.add_argument('--duration',type=float,default=3.)
    a=p.parse_args();run_validation(a.output_dir,a.duration)

if __name__=='__main__': main()
