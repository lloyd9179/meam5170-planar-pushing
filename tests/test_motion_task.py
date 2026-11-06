import json
import os
import subprocess
import sys
import numpy as np
import pytest
from planar_pushing.rule_demo import run_rule_demo


@pytest.mark.parametrize('task,goal',[('reposition',None),('translation',(.04,.04,0.))])
def test_complete_handoff_reacquisition_and_runtime_face_switch(task,goal):
    case=run_rule_demo(task,goal_pose=goal,enable_reposition=True,duration=30.)
    s=case['summary']
    assert s['termination']=='succeeded'
    assert s['final_position_error_m']<.005 and s['final_yaw_error_rad']<np.deg2rad(3)
    assert s['solver_failures']==0 and s['step4_enabled']
    phases={row['phase'] for row in case['rows']}
    assert {'retract','transit','approach','confirm_contact','push','goal_hold','succeeded'}<=phases
    assert s['motion_records'] and all(r['cost']['unit']=='s' for r in s['motion_records'])
    assert all(event['reason']=='loaded_target_contact' for event in s['motion_events'] if event['event']=='ready')
    # One continuous episode: no reset or nonphysical Pusher/Slider jumps.
    times=np.array([o.time for o in case['observations']])
    np.testing.assert_allclose(np.diff(times),.02,atol=1e-11,rtol=0)
    centers=np.array([o.pusher_position for o in case['observations']])
    assert np.max(np.linalg.norm(np.diff(centers,axis=0),axis=1))<.001
    assert np.max(np.linalg.norm(np.diff(case['poses'][:,:2],axis=0),axis=1))<.002
    if goal is not None:
        selected=[event['face'] for event in s['events'] if event.get('event')=='selection']
        assert len(set(selected))>=2
        assert len(s['motion_records'])>=1


def test_headless_motion_cli(tmp_path):
    env=dict(os.environ)
    for key in ('DISPLAY','WAYLAND_DISPLAY','MUJOCO_GL'):env.pop(key,None)
    result=subprocess.run([sys.executable,'-m','planar_pushing.motion_demo','--case','opposite','--headless',
                           '--output-dir',str(tmp_path)],env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stderr
    folder=next(tmp_path.glob('step4_*'))
    data=json.loads((folder/'summary.json').read_text())['opposite']
    assert data['termination']=='ready:loaded_target_contact'
    assert data['transit_loaded_samples']==0
    assert (folder/'step4_paths.png').stat().st_size>10000
