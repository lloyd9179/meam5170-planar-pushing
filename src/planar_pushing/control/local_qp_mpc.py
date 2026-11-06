"""Transparent sparse multiple-shooting QP, one linearization/solve per call."""
from dataclasses import dataclass, field
import time
import numpy as np
from scipy import sparse
import osqp
from ..models.quasi_static import vector
from ..models.contact_geometry import ContactFace


def weight(value, n, name):
    a=np.asarray(value,dtype=float)
    if a.shape==(n,): a=np.diag(a)
    if a.shape!=(n,n) or not np.isfinite(a).all() or not np.allclose(a,a.T,atol=1e-12,rtol=0):
        raise ValueError(f'{name} must be a finite symmetric {n}x{n} matrix or diagonal')
    if np.linalg.eigvalsh(a).min() < -1e-12: raise ValueError(f'{name} must be PSD')
    return a


@dataclass(frozen=True)
class MPCConfig:
    horizon: int = 15
    dt: float = .02
    Q: tuple = (20.,20.,5.,0.)
    Qf: tuple = (80.,80.,20.,0.)
    R: tuple = (.1,.1)
    Rd: tuple = (.2,.2)
    velocity_lower: tuple = (0.,-.025)
    velocity_upper: tuple = (.025,.025)
    warm_start: bool = True
    regularization: float = 1e-9
    eps_abs: float = 1e-7
    eps_rel: float = 1e-7
    max_iter: int = 4000
    acceptance_tolerance: float = 5e-6

    def __post_init__(self):
        if not isinstance(self.horizon,int) or isinstance(self.horizon,bool) or self.horizon<1:
            raise ValueError('horizon must be a positive integer')
        for key in ('dt','eps_abs','eps_rel','acceptance_tolerance'):
            if not np.isfinite(getattr(self,key)) or getattr(self,key)<=0: raise ValueError(key)
        if not np.isfinite(self.regularization) or self.regularization<0: raise ValueError('regularization')
        if not isinstance(self.max_iter,int) or self.max_iter<1: raise ValueError('max_iter')
        for key,n in [('Q',4),('Qf',4),('R',2),('Rd',2)]:
            a=weight(getattr(self,key),n,key)
            object.__setattr__(self,key,tuple(tuple(row) for row in a))
        lo,hi=vector(self.velocity_lower,2,'velocity_lower'),vector(self.velocity_upper,2,'velocity_upper')
        if lo[0]<0 or np.any(lo>hi): raise ValueError('invalid velocity bounds; vn must be >=0')


@dataclass
class QPProblem:
    P: sparse.csc_matrix
    q: np.ndarray
    A: sparse.csc_matrix
    lower: np.ndarray
    upper: np.ndarray
    constant: float
    nominal_states: np.ndarray
    nominal_controls: np.ndarray
    goal: np.ndarray
    rows: dict

    def violation(self,z):
        value=self.A@z
        return float(max(0., np.max(self.lower-value),np.max(value-self.upper)))


@dataclass
class MPCResult:
    control: np.ndarray
    predicted_states: np.ndarray
    predicted_controls: np.ndarray
    objective: float | None
    solve_time: float
    status: str
    feasible: bool
    iterations: int = 0
    solver_time: float = 0.
    constraint_violation: float | None = None
    fallback_used: bool = False
    warm_started: bool = False


class LocalQPMPC:
    def __init__(self,model,config=None):
        if (model.state_dim,model.control_dim)!=(4,2): raise ValueError('expected a 4-state 2-input model')
        self.model=model;self.config=config or MPCConfig();self.reset()

    def reset(self):
        self._controls=None;self._face=None;self._previous_control=np.zeros(2)
        self.last_problem=None

    def rollout(self,x,controls,face):
        states=[np.asarray(x).copy()]
        for u in controls: states.append(self.model.discrete_dynamics(states[-1],u,self.config.dt,face))
        return np.array(states)

    def build_qp(self,current_state,goal_pose,contact_face,nominal_controls=None,previous_control=None):
        x0=vector(current_state,4,'current_state');goal=vector(goal_pose,3,'goal_pose')
        face=ContactFace(contact_face);c=self.config;H=c.horizon;n=6*H
        previous=np.zeros(2) if previous_control is None else vector(previous_control,2,'previous_control')
        controls=np.zeros((H,2)) if nominal_controls is None else np.asarray(nominal_controls,dtype=float).copy()
        if controls.shape!=(H,2) or not np.isfinite(controls).all(): raise ValueError('nominal controls shape')
        nominal=self.rollout(x0,controls,face)
        # One consistent lift for the entire short horizon; no wrapped derivatives.
        theta_goal=x0[2]+np.arctan2(np.sin(goal[2]-x0[2]),np.cos(goal[2]-x0[2]))
        target=np.array([goal[0],goal[1],theta_goal,x0[3]])
        Q,Qf,R,Rd=[np.array(getattr(c,key)) for key in ('Q','Qf','R','Rd')]
        q=np.zeros(n)
        xs=lambda k:slice(4*(k-1),4*k)  # k in 1..H
        us=lambda k:slice(4*H+2*k,4*H+2*(k+1))
        Wblocks=[Q]*(H-1)+[Qf]
        Pxx=2*sparse.block_diag(Wblocks,format='csc')
        difference=sparse.diags([np.ones(H),-np.ones(H-1)],[0,-1],shape=(H,H),format='csc')
        delta=sparse.kron(difference,sparse.eye(2),format='csc')
        Puu=2*(sparse.kron(sparse.eye(H),R,format='csc')
               +delta.T@sparse.kron(sparse.eye(H),Rd,format='csc')@delta)
        P=sparse.block_diag((Pxx,Puu),format='csc')+2*c.regularization*sparse.eye(n,format='csc')
        constant=float((x0-target)@Q@(x0-target)+previous@Rd@previous)
        for k,W in enumerate(Wblocks,1):
            q[xs(k)]=-2*W@target
            constant+=float(target@W@target)
        q[us(0)]=-2*Rd@previous
        rows={'dynamics':slice(0,4*H),'input':slice(4*H,6*H),
              'friction':slice(6*H,10*H),'contact_location':slice(10*H,11*H)}
        row_indices=[];column_indices=[];values=[]
        def block(row,col,value):
            rr,cc=np.nonzero(value)
            row_indices.extend((rr+row).tolist());column_indices.extend((cc+col).tolist())
            values.extend(np.asarray(value)[rr,cc].tolist())
        lower=np.full(11*H,-np.inf);upper=np.full(11*H,np.inf)
        for k in range(H):
            Ak,Bk=self.model.linearize(nominal[k],controls[k],c.dt,face)
            ck=self.model.discrete_dynamics(nominal[k],controls[k],c.dt,face)-Ak@nominal[k]-Bk@controls[k]
            eq=slice(4*k,4*(k+1));block(4*k,4*k,np.eye(4));block(4*k,4*H+2*k,-Bk)
            rhs=ck.copy()
            if k==0: rhs+=Ak@x0
            else: block(4*k,4*(k-1),-Ak)
            lower[eq]=rhs;upper[eq]=rhs
            box=slice(4*H+2*k,4*H+2*(k+1));block(4*H+2*k,4*H+2*k,np.eye(2))
            lower[box]=c.velocity_lower;upper[box]=c.velocity_upper
            cone=slice(6*H+4*k,6*H+4*(k+1))
            block(6*H+4*k,4*H+2*k,self.model.sticking_matrix(nominal[k],face));upper[cone]=0
            sr=10*H+k;block(sr,4*k+3,np.ones((1,1)))
            lower[sr],upper[sr]=self.model.contact_bounds(face)
        Aq=sparse.coo_matrix((values,(row_indices,column_indices)),shape=(11*H,n)).tocsc()
        assert P.shape==(n,n) and Aq.shape==(11*H,n) and q.shape==(n,)
        assert lower.shape==upper.shape==(11*H,)
        return QPProblem(P,q,Aq,lower,upper,constant,nominal,controls,target,rows)

    def _fallback(self,x,status,start,iterations=0,solver_time=0.,violation=None,warm=False):
        self._controls=None;self._previous_control=np.zeros(2)
        return MPCResult(np.zeros(2),np.tile(x,(self.config.horizon+1,1)),np.zeros((self.config.horizon,2)),
            None,time.perf_counter()-start,status,False,iterations,solver_time,violation,True,warm)

    def solve(self,current_state,goal_pose,contact_face):
        start=time.perf_counter();x=vector(current_state,4,'current_state');vector(goal_pose,3,'goal_pose')
        face=ContactFace(contact_face);c=self.config
        if face!=self._face: self.reset();self._face=face
        low,high=self.model.contact_bounds(face)
        if not low<=x[3]<=high: return self._fallback(x,'invalid_contact_location',start)
        warm=c.warm_start and self._controls is not None
        controls=np.vstack((self._controls[1:],self._controls[-1:])) if warm else np.zeros((c.horizon,2))
        qp=self.build_qp(x,goal_pose,face,controls,self._previous_control);self.last_problem=qp
        solver=osqp.OSQP()
        try:
            solver.setup(P=sparse.triu(qp.P,format='csc'),q=qp.q,A=qp.A,l=qp.lower,u=qp.upper,
                         verbose=False,eps_abs=c.eps_abs,eps_rel=c.eps_rel,max_iter=c.max_iter,
                         polishing=True,warm_starting=c.warm_start)
            if warm: solver.warm_start(x=np.r_[qp.nominal_states[1:].ravel(),controls.ravel()])
            raw=solver.solve(raise_error=False)
        except (osqp.OSQPException,ValueError,RuntimeError) as exc:
            return self._fallback(x,f'solver_exception:{type(exc).__name__}:{exc}',start,warm=warm)
        status=raw.info.status;iterations=raw.info.iter;solver_time=raw.info.solve_time
        valid=raw.x is not None and np.isfinite(raw.x).all()
        violation=qp.violation(raw.x) if valid else None
        if status!='solved' or not valid or violation>c.acceptance_tolerance:
            return self._fallback(x,status,start,iterations,solver_time,violation,warm)
        states=np.vstack((x,raw.x[:4*c.horizon].reshape(c.horizon,4)))
        optimal=raw.x[4*c.horizon:].reshape(c.horizon,2)
        # Check the first force cone at the measured state, independently of status.
        if not self.model.is_sticking_feasible(x,optimal[0],face,c.acceptance_tolerance):
            return self._fallback(x,'first_control_not_sticking',start,iterations,solver_time,violation,warm)
        self._controls=optimal.copy();self._previous_control=optimal[0].copy()
        objective=float(.5*raw.x@(qp.P@raw.x)+qp.q@raw.x+qp.constant)
        return MPCResult(optimal[0].copy(),states,optimal.copy(),objective,time.perf_counter()-start,
                         status,True,iterations,solver_time,violation,False,warm)
