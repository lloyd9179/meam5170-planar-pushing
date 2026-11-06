from dataclasses import replace
import itertools
import numpy as np
import pytest
from planar_pushing.models.contact_geometry import ContactFace
from planar_pushing.task import (RectangleGeometry,ContactMonitor,ContactTarget,RepositionRequest,
    RepositionPlan,PlanningResult,WorldObservation,ContactSample,MotionStatus,Phase)
from planar_pushing.motion import VisibilityRepositionPlanner,FeedbackRepositionExecutor,MotionConfig
from planar_pushing.motion_demo import run_motion_case


def obs(time,position=(.1,0.),pose=(0.,0.,0.),twist=(0.,0.,0.),contacts=()):
    return WorldObservation(pose,position,twist,(0.,0.),time,contacts)


def executor(config=None,planner=None):
    g=RectangleGeometry()
    return FeedbackRepositionExecutor(g,planner or VisibilityRepositionPlanner(g),ContactMonitor(g),config)


def start(ex,o=None):
    o=o or obs(0.)
    ex.start(o,RepositionRequest(ContactTarget('left',0),None,'test',o.time,o.slider_pose))
    return o


def advance_to_transit(ex):
    o=start(ex)
    for k in range(10):
        update=ex.update(obs(k*.02))
        if update.phase==Phase.TRANSIT:return k
    raise AssertionError('did not finish settle phase')


def test_replanning_pose_anchor_and_bounded_retry():
    ex=executor(MotionConfig(max_replans=1));start(ex)
    ex.update(obs(0.))
    value=ex.update(obs(.02,pose=(.002,0.,0.)))
    assert value.status==MotionStatus.RUNNING and value.velocity_world==(0.,0.)
    assert ex.records[-1]['plan']['slider_pose_anchor']==(.002,0.,0.)
    value=ex.update(obs(.04,pose=(.004,0.,0.)))
    assert value.status==MotionStatus.FAILED and value.velocity_world==(0.,0.)
    assert 'replan_limit' in value.reason


def test_unsafe_replacement_plan_rejected_before_motion():
    class UnsafePlanner:
        def plan(self,o,target):
            return PlanningResult(RepositionPlan(target,(o.pusher_position,(-.078,0.)),o.slider_pose,
                                                    0,.008,.005),'unsafe')
    ex=executor(planner=UnsafePlanner());start(ex)
    update=ex.update(obs(0.))
    assert update.status==MotionStatus.FAILED and update.velocity_world==(0.,0.)
    assert update.reason=='planner_transit_segment_blocked'


def test_scene_guard_replans_before_crossing_and_load_fails():
    ex=executor();k=advance_to_transit(ex)
    update=ex.update(obs((k+1)*.02,position=(.071,0.)))
    assert 'transit_swept_segment_blocked' in update.reason and update.velocity_world==(0.,0.)
    ex=executor();k=advance_to_transit(ex)
    update=ex.update(obs((k+1)*.02,contacts=(ContactSample((.06,0),.2),)))
    assert update.status==MotionStatus.FAILED and update.reason=='unexpected_transit_contact'
    assert update.velocity_world==(0.,0.)


def test_clock_velocity_and_settle_failure():
    ex=executor();start(ex);ex.update(obs(0.))
    update=ex.update(obs(.04))
    assert update.status==MotionStatus.FAILED and update.reason=='control_timestep_mismatch'
    ex=executor();o=start(ex)
    update=ex.update(replace(o,pusher_velocity_valid=False))
    assert update.status==MotionStatus.FAILED
    ex=executor(MotionConfig(settle_seconds=.04,settle_timeout=.1));start(ex)
    for k in range(10):
        update=ex.update(obs(k*.02,twist=(.03,0,0)))
        if update.status==MotionStatus.FAILED:break
    assert update.reason=='slider_did_not_settle' and update.velocity_world==(0.,0.)


def test_deep_invalid_start_and_motion_timeout_zero_fallback():
    ex=executor();o=obs(0.,position=(0.,0.));start(ex,o)
    update=ex.update(o)
    assert update.status==MotionStatus.FAILED and 'interpenetrating' in update.reason
    ex=executor(MotionConfig(max_seconds=.04));start(ex);ex.update(obs(0.));ex.update(obs(.02))
    update=ex.update(obs(.04))
    assert update.reason=='motion_timeout' and update.velocity_world==(0.,0.)


@pytest.mark.parametrize('start_face,target_face',list(itertools.product(ContactFace,ContactFace)))
def test_real_physics_all_face_pairs(start_face,target_face):
    # Rotated variants, different s, actual contact dynamics; no model or QP needed.
    theta=.6 if target_face in (ContactFace.TOP,ContactFace.RIGHT) else 0.
    case=run_motion_case(start_face,target_face,start_s=-.018,target_s=.018,theta=theta)
    summary=case['summary']
    assert summary['termination']=='ready:loaded_target_contact'
    assert summary['final_contact_matches']
    assert summary['transit_loaded_samples']==0
    assert summary['min_transit_clearance_m']>.004
    assert summary['max_transit_slider_translation_m']<.0002
    assert summary['total_slider_translation_m']<.0005
    assert summary['max_speed_m_s']<=.030001
    points=np.array([o.pusher_position for o in case['observations']])
    assert np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))<.0008
    assert {'retract','transit','approach'} <= {r['phase'] for r in case['rows']}


def test_real_physics_free_start():
    case=run_motion_case('left','top',free_start=(.15,-.12),target_s=.02)
    assert case['summary']['termination']=='ready:loaded_target_contact'
    assert case['summary']['transit_loaded_samples']==0


def test_collision_checker_is_injectable_independently_of_planner_cost():
    class RejectTransit:
        def clearance(self,p,pose):return .01
        def segment_free(self,a,b,pose,margin):return False
    ex=executor()
    ex.collision=RejectTransit()
    start(ex)
    update=ex.update(obs(0.))
    assert update.status==MotionStatus.FAILED and update.velocity_world==(0.,0.)
    assert update.reason=='planner_transit_segment_blocked'
