# Step 2: local sparse QP-MPC design

## 1. Problem and scope

The controller tracks a pose with one fixed face, one material contact coordinate per prediction horizon and a sticking-mode model. It implements **one sequential linearization and one convex QP solve per control update**, not nonlinear black-box optimization. It never accesses MuJoCo or a viewer. High-level sampling, contact choice and repositioning are excluded.

State/input dimensions are nx=4, nu=2. Default horizon H=15 and dt=0.02 s give a 0.30 s prediction window. We adapt the plan's suggested 0.05 s interval to the already-tested Step 0 0.02 s interface. Step 1/2 servo demos explicitly set `EnvConfig.dt_control=mpc_config.dt`; physics substeps are 0.0002 s. If dt changes, integration verifies it is an integer multiple of the physics step. Horizon time and cost interpretation change with H/dt; they are tuning choices, not invariant guarantees.

## 2. Nominal trajectory, derivatives and affine term

A cold start uses ubar=0. A warm start shifts the previous optimal inputs by one step and repeats the last input. Roll out the nonlinear Euler model from the **latest measured x0** to obtain xbar. This keeps the nominal trajectory anchored to feedback rather than shifting stale state predictions.

\[
A_k=\frac{\partial f_d}{\partial x}(\bar x_k,\bar u_k),\quad
B_k=\frac{\partial f_d}{\partial u}(\bar x_k,\bar u_k),\quad
c_k=f_d(\bar x_k,\bar u_k)-A_k\bar x_k-B_k\bar u_k.
\]

The affine dynamics constraint is

\[
x_{k+1}-A_kx_k-B_ku_k=c_k.
\]

The k=0 right-hand side is c0+A0 x_current because x0 is known, not a decision variable. We retain c even when it happens to be zero; dropping it at a nonzero yaw/input nominal would give the wrong dynamics. Jacobians are the Step 1 central differences, not MuJoCo derivatives.

## 3. Angle branch and objective

Lift the goal angle once near the measured unwrapped current angle:

\[
\theta_g^{loc}=\theta_0+\operatorname{atan2}(\sin(\theta_g-\theta_0),\cos(\theta_g-\theta_0)).
\]

Use that same branch throughout the short horizon. The QP does not differentiate a wrap function. This local branch choice is appropriate for local tracking, not large multi-turn rotations; the exactly pi-ambiguous case has the usual branch choice.

With \(g=[p_{gx},p_{gy},\theta_g^{loc},s_0]^T\),

\[
J=\sum_{k=0}^{H-1}\left[(x_k-g)^TQ(x_k-g)+u_k^TRu_k
+(u_k-u_{k-1})^TR_d(u_k-u_{k-1})\right]
+(x_H-g)^TQ_f(x_H-g)+\epsilon\|z\|^2.
\]

u[-1] is the previous successfully issued local input; reset/failure sets it to zero. The s tracking weights default to zero. Q/Qf/R/Rd may be diagonal vectors or full symmetric PSD matrices. Config converts them into immutable matrix tuples and rejects invalid weights.

Defaults retain the supplied initial values: Q=diag(20,20,5,0), Qf=diag(80,80,20,0), R=diag(0.1,0.1), Rd=diag(0.2,0.2). Positions are in m and angle in rad; these weights implicitly compare those numerical units. No claim of physically dimensionless or optimal tuning is made. The sum is a discrete-time cost, not a time integral: weights are not silently multiplied by dt. Terminal weights are 4× stage weights. epsilon=1e-9 adds negligible explicit numerical regularization on all decision components, including unpenalized s. This term is included in objective tests and reported objective values.

## 4. Decision ordering and OSQP standard form

\[
z=[x_1^T,\ldots,x_H^T,u_0^T,\ldots,u_{H-1}^T]^T\in\mathbb R^{6H}.
\]

Zero-based slices: xk uses [4(k-1):4k] for k=1..H; uk uses [4H+2k:4H+2k+2] for k=0..H-1.

\[
\min_z\frac12 z^TPz+q^Tz\quad\text{s.t. }l\le A_{qp}z\le h.
\]

The complete symmetric P is retained for inspection, while only its upper triangle is sent to OSQP as specified by the [OSQP Python interface](https://osqp.org/docs/interfaces/python.html). Sparse matrices are CSC.

### Explicit objective assembly

- State blocks Pkk=2Q for k=1..H-1 and 2Qf at H; qk=-2Wk g.
- Let E be the lower-bidiagonal block difference operator: (Eu)0=u0, (Eu)k=uk-u[k-1]. Then Puu=2(I⊗R+Eᵀ(I⊗Rd)E).
- Thus adjacent input cross blocks are -2Rd. Interior input diagonals have both neighboring increment costs; the final diagonal has only its preceding increment cost.
- The first input linear term is -2Rd u[-1].
- Add 2epsilon I to P. This accounts for OSQP's factor 1/2.
- Track the omitted constant `(x0-g)'Q(x0-g) + u_prev'Rd*u_prev + sum(g'Wk*g)`. `MPCResult.objective` includes it, so it equals the direct tracking cost, not OSQP's constant-free objective.

P is PSD by construction from PSD sums; explicit regularization makes the default P positive definite. Unit tests compare the full direct cost against matrix cost, including constants, cross terms and the terminal stage. A centered H=1 integrator also has a hand-derived scalar optimal vn for an independent optimality check.

## 5. Linear constraints and row ordering

A_qp has 11H rows and 6H columns:

| Rows | Count | Form |
|---|---:|---|
| [0,4H) | 4H | affine dynamics equalities |
| [4H,6H) | 2H | input box |
| [6H,10H) | 4H | force cone and explicit vn>=0 |
| [10H,11H) | H | safe contact-location bounds |

Input limits default to 0<=vn<=0.025 m/s and -0.025<=vt<=0.025 m/s. The largest world speed is sqrt(2)*0.025≈0.03536 m/s, below the simulator's 0.08 m/s cap; rotations preserve speed. These are component limits, not an unadvertised Euclidean constraint.

Force constraints use the Step 1 matrix at nominal s:

\[
\begin{bmatrix}-n^TD\\(t-\mu n)^TD\\(-t-\mu n)^TD\\-1&0\end{bmatrix}u_k\le0.
\]

D=solve(AHAᵀ,F), so constraints are force-based, not |vt|<=mu vn. They are linear because face and nominal geometry are fixed during the solve. In this model sdot=0 exactly and the s dynamics row enforces s[k+1]=s[k]; since s0 is fixed, all predicted s equal s0, making the geometry freezing exact for that horizon (within finite-difference error). This would need reconsideration in a sliding model.

The s bounds are physical half-face length minus the 2 mm corner margin. x0 is validated before solving; only future s values require QP rows. There are no terminal equalities, terminal invariant sets, trust-region constraints, obstacle constraints or guarantees of recursive feasibility/stability. All present constraints are linear; no cone nonlinearities are hidden from OSQP.

### Sparse construction

State/input Hessians use sparse block-diagonal/Kronecker/difference matrices. Dynamics and constraint blocks are accumulated as COO nonzero triplets, then converted to CSC. This avoids repeated sparse slice mutation; it changes neither mathematical cost nor constraints. Dimension assertions check H-dependent shapes. `build_qp` returns P/q/A/l/h, nominal arrays, goal branch, objective constant and row slices for inspection or an alternate solver.

## 6. Warm-start and result acceptance

When enabled, the shifted previous input sequence supplies both the nominal nonlinear rollout and a primal OSQP warm-start vector (new rollout states plus shifted inputs). A fresh OSQP workspace is built each cycle because numerical linearization may change sparsity. We do not claim to reuse a factorization or dual solution. `warm_started` means a shifted primal guess was actually supplied. Disabling warm start starts the nominal from zeros; the previous executed input remains in the increment cost. A face change or explicit `mpc.reset()` clears prior-task state.

Default OSQP settings: eps_abs=eps_rel=1e-7, max_iter=4000, polishing=True. Only status **solved** is accepted; solved-inaccurate, infeasibility, iteration limit and exceptions produce explicit failure results. Additionally check finite decision entries, maximum absolute constraint violation <=5e-6 and the first force cone at the measured state. The residual is the maximum over raw rows in mixed units, not a dimensionless certificate; friction and input tests also check their own rows.

On failure, return zero control, fallback_used=True, feasible=False and an explicit status. No failed optimizer iterate reaches simulation. Returned fallback predictions are constant-state placeholders, not claimed feasible optimized trajectories; objective=None. An emergency zero may violate a user-configured positive vn lower bound; safety fallback is separate from successful optimization feasibility. The demo applies zero once and stops. Previous nominal controls are cleared to avoid propagating failed guesses.

## 7. Logs and execution sequence

`MPCResult` contains control, predicted_states (affine QP), predicted_controls, objective, solve_time, status, feasible, iterations, solver_time, constraint_violation, fallback_used and warm_started. `solve_time` measures the whole build+linearization+setup+solve path; solver_time is OSQP's internal solve time. The latter alone must not be presented as control-loop timing. Timing is observational and not a hard real-time guarantee.

Execution reads MuJoCo state, estimates s/unwrapped theta, solves the independent MPC, maps local input using R F u, executes one environment control interval, checks contact geometry and repeats. No solver code modifies simulator state. An optional GUI simply synchronizes after stepping.

The demo logs every control update, force-cone feasibility, status, both timings, objective, iterations, residual, physical slip flag and actual contact-point geometry. It stops on solver fallback, leaving the safe face interval, or a sustained geometric contact loss (three samples without load and gap>0.5 mm). Every unloaded endpoint sample is logged even if it is a transient soft-contact microgap; the persistence/gap test is an engineering distinction from macroscopic detachment, not a claim of continuous exact sticking. No recontact or contact-free repositioning is performed.

## 8. Tuning and replacement guide

| Parameter | Meaning | Default | Units | File |
|---|---|---|---|---|
| horizon | future control intervals | 15 | steps | control/local_qp_mpc.py: MPCConfig |
| dt | prediction and execution period | 0.02 | s | MPCConfig |
| Q/Qf | stage/terminal pose errors | above | cost per squared SI component | MPCConfig |
| R/Rd | input magnitude / increment | above | cost/(m/s)^2 | MPCConfig |
| velocity_lower/upper | component bounds | (0,-.025)/(.025,.025) | m/s | MPCConfig |
| warm_start | shifted primal and nominal guess | True | boolean | MPCConfig |
| regularization | epsilon ||z||² | 1e-9 | numerical cost | MPCConfig |
| eps_abs,eps_rel | OSQP tolerances | 1e-7,1e-7 | solver scaling | MPCConfig |
| acceptance_tolerance | independently checked row residual | 5e-6 | mixed constraint units | MPCConfig |
| max_iter | OSQP iteration cap | 4000 | iterations | MPCConfig |

Increase H to see farther ahead, with larger QPs and longer wall time. Changing dt changes Euler approximation, horizon time and command holding period. Increase Q/Qf to emphasize pose, R to discourage speed, Rd to smooth changes; none fixes a wrong contact mode. s is not a controllable tracking objective here. Inspect mode, finite-radius adapter, real pusher velocity, geometry, ell and timestep before changing weights when model-only succeeds but MuJoCo fails. Default weights were retained after fixing the measured-velocity compatibility issue, not adjusted to conceal it.

Another QP solver can consume `QPProblem`; replace setup/solve/status mapping in `LocalQPMPC.solve`, retaining the objective/residual tests. Another smooth predictor can preserve the current model contract. Contact-implicit MPC requires new contact force/gap/mode variables and complementarity treatment; replace the sticking cone/constant-s constraints and likely the optimization formulation rather than treating it as a solver switch.

## 9. Derived, approximated and heuristic

**Mathematically derived:** A, wrench mapping, elimination of positive kappa, G, force-cone inequalities, Euler-map derivatives, affine c, quadratic block factors/cross terms, sparse equality signs and decision ordering.

**Modeling approximations:** ellipsoidal friction surface, fixed pressure shape, quasi-static motion, fixed face, point contact, sticking and ds=0, Euler discretization and local linearization.

**Engineering choices:** RMS ell, cost weights, H/dt, component speeds, finite differences, regularization/tolerances, primal warm-start, velocity-servo hardware surrogate and gain, 0.2 ms servo physics step, slip/loss thresholds and demo goals. No global tracking/stability or exact real sticking guarantee is claimed.

The off-center goal is constructed once from a feasible model rollout, making the task a reasonable fixed-face test. The actual controller is free to choose inputs; no reference input sequence is replayed in closed loop. Local linearization with pushing-only constraints can stop at a small residual even for a reachable global pose. Model and MuJoCo outcomes, including the original failures, are reported separately.
