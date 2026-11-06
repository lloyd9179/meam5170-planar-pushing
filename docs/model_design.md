# Step 1: quasi-static model design

## 1. Scope and assumptions

This is a **single point contact, fixed face, sticking-mode**, velocity-controlled predictor for a symmetric rectangular slider. It is not contact-implicit dynamics. Ground pressure shape is fixed; inertia, impact, compliance, lift, roll/pitch, separation and sliding are outside the model. The pusher applies compressive contact forces only. MuJoCo remains the separate execution system; no MuJoCo derivatives or state writes are used by the model.

The formulas below implement the supplied design. The finite-radius adapter and an optional simulator velocity-servo mode are engineering adaptations; neither changes these equations.

## 2. Coordinates, states and contact faces

World axes are right-handed, +z up, yaw positive counterclockwise. Body axes coincide with the rectangle's full length L along x and width W along y. All dimensions are SI.

\[
x=[p_x,p_y,\theta,s]^T\in\mathbb R^4,\qquad
u=[v_n,v_t]^T\in\mathbb R^2.
\]

The input is the **body-frame contact-point material velocity**, resolved into inward normal n and tangent t. Define J as a +90° rotation; t=Jn, F=[n,t], so det(F)=+1. The discrete face is external to x and is not optimized.

| Face (`ContactFace`) | Face center b | Inward n | Tangent t | Physical s interval |
|---|---|---|---|---|
| LEFT | (-L/2,0) | (1,0) | (0,1) | (-W/2,W/2) |
| RIGHT | (L/2,0) | (-1,0) | (0,-1) | (-W/2,W/2) |
| TOP | (0,W/2) | (0,-1) | (1,0) | (-L/2,L/2) |
| BOTTOM | (0,-W/2) | (0,1) | (-1,0) | (-L/2,L/2) |

\[
r=b+s t,\quad R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix}.
\]

A positive s is along t, including the negative y direction on RIGHT. Corners are excluded because the single-face contact normal is ambiguous. MPC additionally stays 2 mm inside either edge. Model evaluation accepts the physical face interior, allowing numerical derivatives at the MPC safety bounds.

**Sticking means the material contact location is fixed: \(\dot s=0\).** A nonzero vt moves that material point with the body; it is not relative sliding along the body surface. Re-estimating s from MuJoCo at the next MPC update is measurement feedback, not a prediction that s can be commanded.

## 3. Contact Jacobian

For body twist \(v_b=[v_x,v_y,\omega]^T\), the material point velocity is translation plus the planar cross product:

\[
v_c=\begin{bmatrix}v_x-\omega r_y\\v_y+\omega r_x\end{bmatrix}
=A(r)v_b,\quad A=\begin{bmatrix}1&0&-r_y\\0&1&r_x\end{bmatrix}\in\mathbb R^{2\times3}.
\]

Power consistency \(f_c^Tv_c=w^Tv_b\) gives \(w=A^Tf_c\), with \(w=[f_x,f_y,\tau]^T\). Force is in N and torque in N m. Under sticking, \(v_c=Fu\).

## 4. Ellipsoidal limit surface and dynamics

Approximate the ground friction limit surface by

\[
f_x^2+f_y^2+\tau^2/\ell^2=F_{\max}^2,\qquad
H=\operatorname{diag}(1,1,1/\ell^2).
\]

Its gradient gives a twist direction \(v_b=\kappa H w\), with positive scale \(\kappa\) (units m/(N s)). Combining the above equations:

\[
K=AHA^T,\quad D=\operatorname{solve}(K,F),\quad
f_c=\frac{1}{\kappa}Du,\quad
v_b=\underbrace{HA^TD}_{G\in\mathbb R^{3\times2}}u.
\]

K is 2×2 dimensionless SPD because A has row rank two and H is positive definite for ell>0. D is 2×2; G has dimensionless first two rows and a 1/m third row. Hence G u returns m/s, m/s, rad/s. We use `np.linalg.solve`, never an explicit inverse. Each mobility evaluation rejects nonfinite results or condition number >1e10. Tests check SPD, condition numbers and A G=F on all faces. For default geometry, condition numbers are small (approximately 2–5 over the usable faces).

`D @ u` is a **scaled force**, not a force in Newtons without identifying kappa. We do not claim to predict absolute contact forces.

For u=(vn,0) at a symmetric face center, r is collinear with n, there is no moment about the center, and omega=0. For off-center contact, the moment arm is nonzero. In fact, for arbitrary body contact velocity v=Fu,

\[
\omega=\frac{r_x v_y-r_y v_x}{\ell^2+\|r\|^2}.
\]

On LEFT at s>0 and positive vn, omega is negative: a +x force at +y causes clockwise rotation. This explains both required qualitative experiments without prescribing slider motion.

World dynamics and the supplied Forward Euler model are

\[
f(x,u)=\begin{bmatrix}R(\theta)(Gu)_{1:2}\\(Gu)_3\\0\end{bmatrix},\qquad
f_d(x,u)=x+\Delta t f(x,u).
\]

Yaw stays continuous in the model. Only tracking-error reporting wraps an angle difference. No wrapping is introduced inside differentiation.

## 5. Force-based sticking feasibility

Physical Coulomb constraints are \(f_n=n^Tf_c\ge0\) and \(|f_t|=|t^Tf_c|\le\mu_p f_n\). With kappa>0 it cancels from the inequalities:

\[
C(s)u\le0,\quad C=\begin{bmatrix}
-n^TD\\(t-\mu_p n)^TD\\(-t-\mu_p n)^TD\\-1&0
\end{bmatrix}.
\]

The last row explicitly imposes vn>=0 as requested. We retain it even where it is redundant for a particular geometry. `|vt| <= mu*vn` is **not** used: mobility mixes tangential and normal force differently from velocity. For example, changing s changes D and thus the velocity-space image of the force cone.

Zero input lies at the cone apex and is feasible. These homogeneous inequalities do not guarantee a positive contact load or maintain a real contact after inertia/compliance causes separation. Sliding, contact breaking and recontact are not modes in this model.

## 6. Friction Parameters, Identifiability, and Model Mismatch Limitations

- **ell changes predicted motion** by changing the ratio between rotational and translational mobility. Its default \(\sqrt{(L^2+W^2)/12}=0.04163332\) m is a geometric RMS radius heuristic, not a calibrated exact pressure-distribution parameter.
- **mu_pusher changes force-cone feasibility**, not the unconstrained velocity map G. MuJoCo has independently configurable true contact friction.
- **mu_ground_nominal is metadata for the ideal model and a default for matched simulator comparisons.** Uniformly scaling ground friction with a fixed pressure shape rescales the required wrench/kappa, which cancels from the velocity map. We deliberately do not multiply model velocity by this coefficient.
- Ground friction changes can still affect MuJoCo inertia, breakaway, compliance and contact transients. Those effects are not identifiable from this velocity-only sticking model. Future robustness experiments must label this structural limitation; varying mu_ground alone is not a model of every friction mismatch effect.
- To represent pressure-shape uncertainty, vary ell or replace H; do not invent a ground-friction velocity gain. Absolute wrench prediction needs a force scale/limit surface normalization and kappa identification.

## 7. Numerical Jacobians

`linearize(x,u,dt,face)` returns 4×4 A and 4×2 B for **discrete** dynamics. Central differences use \(h_i=fd\_step\max(1,|x_i|)\) (likewise for inputs). The s perturbation is further limited to one quarter of the distance to the physical corner; the stencil never leaves the face. At a physical corner or an unresolvably small interior distance, evaluation raises a clear error rather than using a bogus derivative.

Face is fixed, yaw is unwrapped, and zero controls are supported. Directional-derivative tests use independent directions, a different difference scale, nonzero yaw near pi, all faces and both zero/nonzero inputs. The finite-difference Jacobian can later be replaced without changing MPC's model API.

## 8. Parameters and modification dependencies

| Parameter | Meaning | Default | Units | File |
|---|---|---|---|---|
| slider_length, slider_width | full rectangular dimensions | 0.12, 0.08 | m | models/quasi_static.py: ModelConfig |
| ell | torque/force length scale | RMS radius above | m | ModelConfig |
| mu_pusher | nominal force cone coefficient | 0.5 | dimensionless | ModelConfig |
| mu_ground_nominal | ground scale metadata / matched execution setup | 0.3 | dimensionless | ModelConfig |
| fd_step | central-difference relative step | 1e-6 | relative; SI perturbations | ModelConfig |
| corner_margin | MPC face safety inset | 0.002 | m | ModelConfig |
| dt | Euler propagation / MPC execution interval | 0.02 | s | control/local_qp_mpc.py: MPCConfig |
| pusher radius | simulator cylinder radius | 0.01 | m | env.py: EnvConfig |
| pusher_mode | optional measured-velocity execution | velocity_servo in Step 1/2; mocap remains Step 0 default | — | integration.py / env.py |
| pusher_servo_gain | XY velocity actuator gain | 500 in comparisons; EnvConfig default 1000 | N/(m/s) | integration.py / env.py |
| dt_physics | Step 1/2 servo substep | 0.0002 | s | integration.py |

Changing geometry affects contact Jacobians, face bounds, default ell (if not explicitly supplied) and simulator geometry. Changing ell affects every model rollout, Jacobian, force cone and MPC prediction. Changing dt requires matching the environment control interval and an integer number of physics substeps. Runtime modifications should use new immutable configs; reconstruct model/controller and reset warm starts.

## 9. Simulator adapter and finite cylinder

Ideal touching center geometry is

\[
c=p+R(r-\rho n).
\]

The implemented adapter sends \(\dot c\approx R F u\), the point-contact velocity approximation. The derivative of the exact fixed-s geometric center is instead

\[
\dot c_{geom}=RFu-\rho\omega R J n.
\]

Adding that second term for a **nonspinning cylinder** would introduce tangential material velocity mismatch. Thus exact fixed material s, no cylinder spin, and exact cylinder center geometry cannot all be enforced while the slider turns. We keep the point-velocity approximation explicit, estimate actual s from the center projection, and log contact-point error, s drift and material slip. No hidden position correction, slider teleportation or recontact controller is applied. A future rolling/spinning pusher model would require an expanded interface.

`model_state` uses measured slider pose and the pusher center projected into the selected body face. `contact_diagnostics` reads actual `data.contact.pos`, loaded normal forces, slider twist and physical pusher velocity. Slip flags are engineering diagnostics at |relative tangent velocity|>0.002 m/s, not a proof of exact sticking; subthreshold motion and vertical friction can remain. Contact-point error compares actual contact centroid to the intended face point at the **measured s**; accumulated drift from the initial s is recorded separately.

### Necessary Step 0 compatibility extension

The original mocap path reports zero pusher spatial velocity even while its position advances. This prevents the intended tangential relative velocity from entering friction. The [official MuJoCo discussion](https://mujoco.readthedocs.io/en/stable/modeling.html#mocap-bodies) describes this limitation. We reproduced it by querying `mj_objectVelocity` under command (0.025,0.025), which returned six zeros. Original mocap rotation failures are preserved in the report/results.

`EnvConfig(pusher_mode='velocity_servo')` adds two physical XY slide joints and velocity actuators. The slider is still a free body moved only by contact. The default mode and all original API signatures remain valid. Step 1/2 comparisons use a 0.1 kg pusher, gain 500 N/(m/s), implicit integration and 0.2 ms physics substeps. A 1 ms servo trial showed contact chatter; the smaller substep resolves this better. Saturated command reference is still `info['applied_velocity']`; actual servo velocity can differ under contact loads. This difference is part of model/execution mismatch.

## 10. Validation interpretation and replacing the model

`python -m planar_pushing.validate_model` runs centered and s=0.02 m LEFT pushes at 0.005/0.010/0.020 m/s, with 3 s duration. It establishes contact using a real gentle approach after settling. One-step prediction starts from each measured x. Multi-step open-loop prediction starts only once and **replays the identical world velocity references**, converted at the predicted angle; it is not repeatedly reset to the actual trajectory. Both error definitions use equivalent actuator commands. All samples, lost-load samples, s drift and slip flags remain in NPZ/CSV logs; separate per-run summaries list model and execution settings.

Results and the supplied approximations do not constitute calibration. See `step1_step2_report.md` for numerical errors, failures and timing. To replace this model, preserve `state_dim`, `control_dim`, `continuous_dynamics`, `discrete_dynamics`, `linearize`, `sticking_matrix`, `is_sticking_feasible` and `contact_bounds`, or update MPC's explicit constraint assembly. A nonlinear predictor with the same state/input/face contracts can reuse the sequential linearization framework. A sliding or contact-implicit predictor requires new state/mode variables and new feasibility constraints; it is not a drop-in change to H.

The replay uses the commanded/saturated center velocity reference, not a measured ideal velocity source. Servo load tracking error is therefore part of the reported prediction/execution discrepancy; see the report for the resulting centered open-loop offset.
