"""Controller-independent, SI-unit contracts. No simulator or optimizer handles."""
from dataclasses import dataclass
from enum import Enum
from typing import Protocol
import math
from ..models.contact_geometry import ContactFace

CONTRACT_VERSION = 1
CONTRACT_REVISION = 1  # additive Step 4 fields; original constructor/protocols stay valid


def finite_tuple(value, size, name):
    result = tuple(float(v) for v in value)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must contain {size} finite values')
    return result


@dataclass(frozen=True)
class ContactTarget:
    face: ContactFace
    s: float  # body-face tangent coordinate, m; desired, never a measured state

    def __post_init__(self):
        object.__setattr__(self, 'face', ContactFace(self.face))
        if not math.isfinite(self.s):
            raise ValueError('s must be finite')


@dataclass(frozen=True)
class ContactSample:
    position_world: tuple[float, float]
    normal_force: float

    def __post_init__(self):
        object.__setattr__(self, 'position_world', finite_tuple(self.position_world, 2, 'position'))
        if not math.isfinite(self.normal_force) or self.normal_force < 0:
            raise ValueError('normal force must be finite and nonnegative')


@dataclass(frozen=True)
class WorldObservation:
    slider_pose: tuple[float, float, float]  # world x/y/yaw, m/m/rad
    pusher_position: tuple[float, float]    # world center, m
    slider_twist: tuple[float, float, float]  # world vx/vy/wz, m/s,m/s,rad/s
    pusher_velocity: tuple[float, float]     # actual world velocity, m/s
    time: float
    contacts: tuple[ContactSample, ...] = ()
    pusher_velocity_valid: bool = True

    def __post_init__(self):
        for key, n in [('slider_pose', 3), ('pusher_position', 2), ('slider_twist', 3), ('pusher_velocity', 2)]:
            object.__setattr__(self, key, finite_tuple(getattr(self, key), n, key))
        object.__setattr__(self, 'contacts', tuple(self.contacts))
        if not all(isinstance(c, ContactSample) for c in self.contacts):
            raise ValueError('contacts must be ContactSample values')
        if not math.isfinite(self.time) or self.time < 0:
            raise ValueError('time must be finite and nonnegative')


@dataclass(frozen=True)
class ContactEstimate:
    face: ContactFace
    measured_s: float
    gap: float  # positive is separation, m
    in_bounds: bool
    loaded: bool
    normal_force: float
    slip_speed: float | None
    point_error: float | None


@dataclass(frozen=True)
class SelectionContext:
    current: ContactTarget | None = None
    blocked: tuple[ContactTarget, ...] = ()
    trigger: str = 'initial'


@dataclass(frozen=True)
class SelectionResult:
    target: ContactTarget | None
    reason: str


@dataclass(frozen=True)
class PushResult:
    velocity_world: tuple[float, float]
    feasible: bool
    status: str
    solve_time: float = 0.
    iterations: int = 0
    objective: float | None = None
    constraint_violation: float | None = None

    def __post_init__(self):
        object.__setattr__(self, 'velocity_world', finite_tuple(self.velocity_world, 2, 'velocity'))


class Phase(str, Enum):
    SELECT = 'select'
    CONFIRM_CONTACT = 'confirm_contact'
    PUSH = 'push'
    GOAL_HOLD = 'goal_hold'
    REPOSITION_REQUIRED = 'reposition_required'
    RETRACT = 'retract'
    TRANSIT = 'transit'
    APPROACH = 'approach'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


@dataclass(frozen=True)
class RepositionRequest:
    target: ContactTarget
    previous_contact: ContactTarget | None
    trigger: str
    requested_at: float
    slider_pose_at_request: tuple[float, float, float]


@dataclass(frozen=True)
class RepositionPlan:
    """Step 4 output: path from actual center to precontact; approach is separate.

    length_m includes retreat+transit, and is Jtravel's initial convention.
    The future executor must revalidate the pose anchor and swept segments.
    """
    target: ContactTarget
    waypoints_world: tuple[tuple[float, float], ...]
    slider_pose_anchor: tuple[float, float, float]
    retreat_end_index: int = 0
    precontact_gap: float = 0.0
    clearance: float = 0.0

    def __post_init__(self):
        points = tuple(finite_tuple(p, 2, 'waypoint') for p in self.waypoints_world)
        if len(points) < 2:
            raise ValueError('a plan needs at least start and precontact waypoints')
        object.__setattr__(self, 'waypoints_world', points)
        object.__setattr__(self, 'slider_pose_anchor', finite_tuple(self.slider_pose_anchor, 3, 'pose'))
        if not isinstance(self.retreat_end_index, int) or not 0 <= self.retreat_end_index < len(points):
            raise ValueError('invalid retreat waypoint index')
        if not all(math.isfinite(v) and v >= 0 for v in (self.precontact_gap, self.clearance)):
            raise ValueError('plan distances must be finite and nonnegative')

    @property
    def length_m(self):
        return sum(math.dist(a, b) for a, b in zip(self.waypoints_world, self.waypoints_world[1:]))


@dataclass(frozen=True)
class MotionCost:
    total: float
    unit: str
    components: tuple[tuple[str, float], ...] = ()

    def __post_init__(self):
        if not math.isfinite(self.total) or not isinstance(self.unit, str) or not self.unit:
            raise ValueError('cost requires a finite total and explicit unit')
        components = tuple((str(k), float(v)) for k, v in self.components)
        if not all(math.isfinite(v) for _, v in components):
            raise ValueError('cost components must be finite')
        object.__setattr__(self, 'components', components)


@dataclass(frozen=True)
class PlanningResult:
    plan: RepositionPlan | None
    reason: str
    cost: MotionCost | None = None


class MotionStatus(str, Enum):
    RUNNING = 'running'
    READY = 'ready'  # hand back to supervisor; NOT a confirmed physical contact
    FAILED = 'failed'


@dataclass(frozen=True)
class MotionUpdate:
    velocity_world: tuple[float, float]
    phase: Phase
    status: MotionStatus
    reason: str = ''

    def __post_init__(self):
        object.__setattr__(self, 'velocity_world', finite_tuple(self.velocity_world, 2, 'velocity'))
        object.__setattr__(self, 'phase', Phase(self.phase))
        object.__setattr__(self, 'status', MotionStatus(self.status))
        if self.phase not in (Phase.RETRACT, Phase.TRANSIT, Phase.APPROACH, Phase.CONFIRM_CONTACT):
            raise ValueError('motion port may only own reposition phases')


@dataclass(frozen=True)
class TaskDecision:
    velocity_world: tuple[float, float]
    phase: Phase
    reason: str
    target: ContactTarget | None = None
    contact: ContactEstimate | None = None
    reposition_request: RepositionRequest | None = None
    push: PushResult | None = None

    @property
    def terminal(self):
        return self.phase in (Phase.SUCCEEDED, Phase.FAILED)


class ContactSelector(Protocol):
    def reset(self, seed: int = 0) -> None: ...
    def select(self, observation: WorldObservation, goal_pose: tuple,
               context: SelectionContext) -> SelectionResult: ...


class PushController(Protocol):
    def reset(self) -> None: ...
    def activate(self, target: ContactTarget) -> None: ...
    def compute(self, observation: WorldObservation, contact: ContactEstimate,
                goal_pose: tuple) -> PushResult: ...


class RepositionPlanner(Protocol):
    def plan(self, observation: WorldObservation, target: ContactTarget) -> PlanningResult: ...


class RepositionCostEvaluator(Protocol):
    def evaluate(self, plan: RepositionPlan, observation: WorldObservation) -> MotionCost: ...


class RepositionExecutor(Protocol):
    def start(self, observation: WorldObservation, request: RepositionRequest) -> None: ...
    def update(self, observation: WorldObservation) -> MotionUpdate: ...
    def cancel(self) -> None: ...
