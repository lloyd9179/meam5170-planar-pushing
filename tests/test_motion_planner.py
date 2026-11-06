from dataclasses import replace
import numpy as np
import pytest
from planar_pushing.models.contact_geometry import rotation,ContactFace
from planar_pushing.task import RectangleGeometry,WorldObservation,ContactTarget,MotionCost
from planar_pushing.motion import VisibilityRepositionPlanner,PlannerConfig,NominalTimeCost,TravelCostConfig
from planar_pushing.motion.collision import segment_hits_open_box,body_point,circle_clearance


def observation(position,pose=(0.,0.,0.),time=0.):
    return WorldObservation(pose,tuple(position),(0.,0.,0.),(0.,0.),time)


def independent_clearance(point,g):
    nearest=np.clip(point,[-g.length/2,-g.width/2],[g.length/2,g.width/2])
    return np.linalg.norm(point-nearest)-g.pusher_radius


@pytest.mark.parametrize('a,b,hit', [((-2,0),(2,0),True),((-2,1),(2,1),False),
    ((-2,1.1),(2,1.1),False),((-2,2),(2,-2),True),((2,0),(2,2),False),
    ((0,0),(0,0),True),((1,1),(1,1),False)])
def test_slab_segment_interior(a,b,hit):
    assert segment_hits_open_box(a,b,[1,1])==hit


@pytest.mark.parametrize('face',list(ContactFace))
@pytest.mark.parametrize('theta',[0.,.73])
def test_plan_swept_geometry_and_coordinates(face,theta):
    g=RectangleGeometry();pose=(.1,-.05,theta)
    start=g.center_position(pose,ContactTarget('right',-.02))
    planner=VisibilityRepositionPlanner(g)
    result=planner.plan(observation(start,pose),ContactTarget(face,.02))
    assert result.plan is not None and result.cost.unit=='s'
    plan=result.plan
    np.testing.assert_allclose(plan.waypoints_world[0],start)
    np.testing.assert_allclose(plan.waypoints_world[-1],g.center_position(pose,plan.target,.008))
    for index,(a,b) in enumerate(zip(plan.waypoints_world,plan.waypoints_world[1:])):
        clear=[independent_clearance(body_point(a+(np.array(b)-a)*t,pose),g) for t in np.linspace(0,1,151)]
        if index<plan.retreat_end_index:
            assert min(clear)>-1e-10 and np.min(np.diff(clear))>-1e-10
        else:
            assert min(clear)>=planner.config.clearance-1e-10
    assert result.cost.total==min(item.cost.total for item in planner.last_candidates)


def test_random_valid_starts_all_faces_and_finite_radius_corner():
    g=RectangleGeometry();planner=VisibilityRepositionPlanner(g)
    rng=np.random.default_rng(14)
    points=[(.068,.048),(-.07,0),(.07,0),(.2,-.15)]
    while len(points)<64:
        p=rng.uniform(-.2,.2,2)
        if independent_clearance(p,g)>=0:points.append(p)
    for i,p in enumerate(points):
        target=ContactTarget(list(ContactFace)[i%4],.015*(-1)**i)
        result=planner.plan(observation(p),target)
        assert result.plan is not None
        for j,(a,b) in enumerate(zip(result.plan.waypoints_world,result.plan.waypoints_world[1:])):
            minimum=min(independent_clearance(np.array(a)+(np.array(b)-a)*t,g) for t in np.linspace(0,1,41))
            assert minimum>=(-1e-10 if j<result.plan.retreat_end_index else .005-1e-10)


def test_cost_units_breakdown_and_rigid_transform_invariance():
    g=RectangleGeometry();planner=VisibilityRepositionPlanner(g)
    a=planner.plan(observation((.07,0)),ContactTarget('left',0))
    parts=dict(a.cost.components)
    assert parts['Jtravel_m']==pytest.approx(a.plan.length_m)
    assert a.cost.total==pytest.approx(parts['nominal_motion_s']+parts['turn_penalty_s']+parts['settle_s'])
    pose=(.2,-.3,.9);start=np.array(pose[:2])+rotation(pose[2]) @ [.07,0]
    b=planner.plan(observation(start,pose),ContactTarget('left',0))
    assert b.cost.total==pytest.approx(a.cost.total)
    assert b.plan.length_m==pytest.approx(a.plan.length_m)
    slower=NominalTimeCost(g,TravelCostConfig(transit_speed=.015)).evaluate(a.plan,observation((.07,0)))
    assert slower.total>a.cost.total


def test_replace_cost_without_touching_motion_model_or_safety():
    class PreferUpperRoute:
        def evaluate(self,plan,obs):
            # A deliberately different, dimensionless policy to test injection.
            return MotionCost(1. if max(p[1] for p in plan.waypoints_world)>.05 else 10.,'score')
    g=RectangleGeometry();planner=VisibilityRepositionPlanner(g,cost_evaluator=PreferUpperRoute())
    result=planner.plan(observation((.07,0)),ContactTarget('left',0))
    assert result.cost.unit=='score' and result.cost.total==1.
    assert max(p[1] for p in result.plan.waypoints_world)>.05
    for index,(a,b) in enumerate(zip(result.plan.waypoints_world,result.plan.waypoints_world[1:])):
        if index>=result.plan.retreat_end_index:
            assert not segment_hits_open_box(a,b,[.075,.055])


def test_invalid_initial_pose_target_and_cost_report_failure():
    g=RectangleGeometry();planner=VisibilityRepositionPlanner(g)
    assert planner.plan(observation((0,0)),ContactTarget('left',0)).plan is None
    assert planner.plan(observation((-.07,0)),ContactTarget('left',.04)).plan is None
    class BadCost:
        def evaluate(self,p,o):return MotionCost(float('nan'),'s')
    result=VisibilityRepositionPlanner(g,cost_evaluator=BadCost()).plan(observation((.07,0)),ContactTarget('left',0))
    assert result.plan is None and 'invalid_cost' in result.reason


@pytest.mark.parametrize('kwargs',[{'clearance':0},{'precontact_gap':.005},{'vertex_padding':float('nan')}])
def test_invalid_planner_config(kwargs):
    with pytest.raises(ValueError):PlannerConfig(**kwargs)
