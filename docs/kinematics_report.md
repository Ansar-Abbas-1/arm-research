# Kinematics design report — 5-DOF arm (`arm_kinematics`, with `arm_trajectory` / `arm_camera` interfaces)

Date: 2026-10-07. Scope: how the kinematics is designed mathematically, derived from the code, `arm_interface/urdf/arm.urdf` and the existing docs; cross-checked numerically. Primary reference: **Siciliano, Sciavicco, Villani, Oriolo, *Robotics: Modelling, Planning and Control*, Springer 2009** [S]. Layout deviates from the default 5-section report template (topic-based, like the other docs in this folder): experimental setup is not applicable, results are the numerical checks in each section, limitations are collected in §9.

Conventions: URDF `rpy` = fixed-axis `Rz(yaw)·Ry(pitch)·Rx(roll)`; Jacobian in the **spatial (base-frame)** convention; angles in rad, lengths in m; `s(φ) = (sin φ, cos φ)`, `c(φ) = (cos φ, −sin φ)` are the images of a link's local `z` and `x` axis in the (x,z) plane after `Ry(φ)`.

Reference markers: `[S]` Siciliano 2009; other keys in §10. Siciliano section numbers marked † are from memory of the book's table of contents and were **not** verified (web search could not retrieve the contents); chapter and topic are reliable.

Tooling note: all "verified" numbers below were recomputed with an independent numpy implementation (`docs/fk_reference.py` plus throw-away scripts; not added to the repo).

---

## 1. Kinematic model

### 1.1 Chain from the URDF

`base_link → joint_1 → link_1 → joint_2 → link_2 → joint_3 → link_3 → joint_intermediate (fixed) → joint_4 → link_4 → joint_5 → link_5 → end_effector_joint (fixed) → end_effector`. The gripper (`joint_6`, prismatic, plus mirror) is a branch off `end_effector` and is invisible to every solver by construction (`ForwardKinematicsSolver` walks `parent_joint` from `tip_link` to `root_link`, `forward_kinematics.cpp`).

Each joint contributes (`[S]` §2.8, open chain; the repo uses URDF origin+axis instead of Denavit–Hartenberg [DH55])

```
T_i(q_i) = Trans(xyz_i) · Rot(rpy_i) · Rot(axis_i, q_i)         T(q) = Π_i T_i
```

Product-of-exponentials equivalent [LP17]: `T(q) = e^{[S₁]q₁} … e^{[S₅]q₅} M`, with screw axes `S_i = (ω_i ; −ω_i × p_i)` at `q = 0`. Both forms are the same map; the repo's form is the one URDF/TF use, so FK and `robot_state_publisher` agree by construction.

| joint | xyz (m) | rpy pitch (rad) | axis | limits (rad) |
|---|---|---|---|---|
| joint_1 | (0,0,0.03) | 0 | z | [−2.745, 2.736] |
| joint_2 | (0,0,0.065) | −1.57 | y | [0, 3.14] |
| joint_3 | (0,0,0.34) | 0 | y | [−2.22, 0] |
| joint_intermediate | (0.029,0,0) | 3.1412 | fixed | — |
| joint_4 | (0,0,0.376) | −1.8791 | y | [0, 4.505] |
| joint_5 | (0,0,0.336) | 1.81 | y | [−3.62, 0] |
| end_effector_joint | (0,0,0.105) | 0 | fixed | — |

(All roll/yaw of the origins are 0. All joint velocity limits are 2 rad/s.)

### 1.2 Structure: yaw followed by a planar 4R

Joint 1 is a `z` rotation applied first; joints 2–5 all rotate about `y` and every fixed rotation is a pure `Ry`. Rotations about the same axis commute and add, so the whole chain after `joint_1` lives in one vertical plane. With cumulative angles (angle of the local `z` axis from world `z` towards the plane's radial axis):

```
φ₂ = q₂ − 1.57
φ₃ = φ₂ + q₃
φᵢ = φ₃ + 3.1412                       (link_intermediate)
φ₄ = φᵢ − 1.8791 + q₄ = q₂+q₃+q₄ − 0.3079
φ₅ = φ₄ + 1.81 + q₅  = q₂+q₃+q₄+q₅ + 1.5021

p_planar(r,z) = (0, 0.095) + 0.34·s(φ₂) + 0.029·c(φ₃) + 0.376·s(φᵢ) + 0.336·s(φ₄) + 0.105·s(φ₅)
p = (r cos q₁, r sin q₁, z)             R = Rz(q₁) · Ry(φ₅)
```

`r` is a **signed** radius (the tip can be behind the `joint_1` axis, r < 0; the zero configuration has x = +0.0389 m but joint_5 at x = −0.066 m).

*Verified:* matches the URDF-composed FK to 1e-9 at `q = 0` (tip (0.0389, 0, 0.4515) m) and at `q = (0.3, 0.5, −0.2, 0.4, −0.3)` (tip (0.2839, 0.0878, 0.4510) m); the repo's own reference values are for `link_5` (the same numbers minus the 0.105 m fixed offset along link_5's `z`).

**Effective links.** The pair `(joint_intermediate, joint_4)` is an offset elbow: in link_3 axes the vector from joint 3 to joint 4 is `(0.029, −0.376)`-ish, i.e. one rigid link of length `ρ = 0.37713 m` at a constant angle `β = 3.0642 rad` (= π − 0.0774) to link_3's `z`: `0.029c(φ₃)+0.376s(φᵢ) = ρ·s(φ₃+β)`. So the planar chain is a 4R with link lengths `l₂ = 0.34`, `ρ = 0.37713`, `l₄ = 0.336`, `l_tool = 0.105`, shoulder at height 0.095 m. In DH terms this is `d₁ = 0.095`, `α₁ = ±π/2` (convention-dependent), `a₂..a₄ = 0.34, 0.37713, 0.336`, `a_tool = 0.105` plus constant `θ` offsets absorbing `−1.57, β, 1.81…`; the repo never builds a DH table, and the planar absolute-angle form above is exact, so it is used here instead.

**Why the tip is a fixed offset (M2 finding).** `end_effector` was originally behind a revolute `joint_6` whose axis coincided with its own translation, so orientation depended on an unrelated gripper angle. Making it fixed removes a spurious DOF; the tip pose is then `T(base→link_5)·Trans(0,0,0.105)` exactly (`forward_kinematics.md`).

**Joint zero/units.** Joint angles reach the model from encoder ticks as `q_i = sign_i·(ticks_i − bias_i)·2π/4096` (`arm_interface/src/robot_interface.cpp:419`, `config/ArmInterface.yaml`), i.e. a per-joint affine calibration (zero offset, direction, 1.53 mrad resolution).

### 1.3 Closed-form facts used later

* Maximum reach (any `q`, triangle inequality on the origin offsets): `0.03+0.065+0.34+0.029+0.376+0.336+0.105 = 1.281 m`; sampled maximum 1.189 m.
* `R(q) = Rz(q₁)Ry(φ₅)` has **roll ≡ 0** and **yaw = q₁ (or q₁+π if r<0) = atan2(p_y,p_x)** (+π). The reachable end-effector poses therefore form a **4-dimensional** submanifold of SE(3), `{(p, Rz(atan2(p_y,p_x)[+π])·Ry(φ))}`, parametrised by `(q₁, r, z, φ₅)`.
* Tool `y` axis `= R·ŷ = Rz(q₁)ŷ = â` (the common axis of joints 2–5) for every `q` (verified, difference exactly 0).

---

## 2. Differential kinematics

### 2.1 Geometric Jacobian

For revolute joint `i` with world axis `z_i`, world origin `p_i` (axis frame after the fixed origin, before the joint rotation) and tip `p_e` ([S] §3.1–3.2†; `jacobian.cpp`):

```
J_v,i = z_i × (p_e − p_i)        J_ω,i = z_i        J = [J_v ; J_ω] ∈ ℝ^{6×5}
```

Validated in `test_jacobian.cpp` against a central finite-difference Jacobian of FK (the angular block through `R(q+h e_i)R(q−h e_i)ᵀ ≈ Exp(2h z_i)`), agreement ~1e-5 (truncation error).

### 2.2 Structure of J for this arm (derivation)

Use the cylindrical basis `(ê_r, ê_θ, ẑ)` at the current `q₁`, `ê_θ = â`. Let `Δr_i = r_e − r_i`, `Δz_i = z_e − z_i`.

```
column 1 (joint_1): z₁ = ẑ,  p_e−p₁ = r ê_r + (z_e−0.03) ẑ  ⇒  J_v,1 = r ê_θ,   J_ω,1 = ẑ
columns 2..5:       z_i = â = ê_θ,  â × (Δr ê_r + Δz ẑ) = Δz ê_r − Δr ẑ  ⇒  J_v,i = (Δz_i, 0, −Δr_i),   J_ω,i = ê_θ
```

In components `[v_r, v_θ, v_z, ω_r, ω_θ, ω_z]`:

```
J = [   0     Δz₂   Δz₃   Δz₄   Δz₅ ]   v_r
    [   r      0     0     0     0  ]   v_θ
    [   0    −Δr₂  −Δr₃  −Δr₄  −Δr₅ ]   v_z
    [   0      0     0     0     0  ]   ω_r  (roll rate ≡ 0)
    [   0      1     1     1     1  ]   ω_θ  (pitch rate)
    [   1      0     0     0     0  ]   ω_z  (yaw rate)
```

Two structural linear constraints hold for every `q̇`: `ω_r = 0` and `v_θ = r·ω_z`. Hence

```
rank J ≤ 6 − 2 = 4 for every configuration, nullity ≥ 1.
```

*Verified numerically:* the 5th singular value of the 6×5 `J` is 1e-16…1e-19 at 5 random in-limit configurations and also at `q = 0` and two other poses (e.g. σ = [2.20, 1.02, 0.45, 0.22, 7e-17]). The null vector has zero `q₁` component and `Σ_{i=2..5} n_i = 0` (pitch-preserving planar self-motion; e.g. `n = (0, 0.632, −0.057, −0.752, 0.178)` at `q = (0.3,0.5,−0.2,0.4,−0.3)`).

**Task Jacobian (position + pitch).** `J_task = [J_v ; êₚᵀ J_ω]` with `êₚ` the tool `y` axis `= â`, so the pitch row is the **constant** `[0,1,1,1,1]` (verified). `J_task ∈ ℝ^{4×5}` has full row rank (4) except at the singularities below; `ker J_task = ker J` (1-D).

### 2.3 Singularities ([S] §3.3† arm singularities, applied)

`J_task` loses rank iff

1. `r = 0` (tip on the `joint_1` axis: the `v_θ` row `(r,0,0,0,0)` vanishes; `q₁` loses all leverage), or
2. the planar 3×4 Jacobian `(r, z, φ)` rank-drops. Its columns are `(Δz_i, −Δr_i, 1)ᵀ`, `i=2..5`; three of them are dependent iff the corresponding joint centres `(r_i,z_i)` are collinear (affine dependence of points in the plane). So: **all four planar joint centres on one line** (arm stretched or folded straight).

This corrects an earlier reading: at `q = 0` the joint centres are **not** collinear (joint centres (x,z): (0,0.095), (−0.34,0.095), (0.036,0.124), (−0.066,0.444)), and `σ_min(J_task) = 0.039` there — poorly conditioned (below the 0.05 damping threshold) but not singular. The "σ ≈ 1e-33" quoted in `arm_trajectory` Finding 1 is the structural zero singular value of the 6×5 `J` that exists at *every* `q` (§2.2), not evidence of a singularity; `q=0` is still a good point to avoid, because all four elbow joints sit on a joint limit simultaneously (`q₂=q₄=0` lower, `q₃=q₅=0` upper), so Newton steps are clamped.

*Statistics (20 000 uniform in-limit configurations):* `σ_min(J_task)` percentiles 1/10/50 % = 0.006/0.052/0.19; **9.6 %** of the joint box has `σ_min < 0.05`, where the variable damping of §4.3 is active.

### 2.4 Self-motion manifold and analytic inverse kinematics (derived here; not implemented in the repo)

Given a target `(x,y,z)` and tool pitch `Φ = φ₅`:

1. `q₁ = atan2(y,x)` with `r = +√(x²+y²)`, or `q₁ + π` with `r = −√(x²+y²)` (a second branch if inside the joint-1 range, which spans 5.48 of 2π rad).
2. Wrist point: `p₅ = (r,z) − l_tool·s(Φ)`.
3. **Free parameter** `φ₄ ∈ (−π, π]` (the angle of link_4): `p₄ = p₅ − l₄·s(φ₄)`.
4. Remaining two-link problem `d = p₄ − (0,0.095) = l₂ s(φ₂) + ρ s(φ₂+q₃+β)`; law of cosines ([S] §2.10†, planar two-link arm):

```
cos(q₃+β) = (|d|² − l₂² − ρ²) / (2 l₂ ρ)            q₃ = −β ± arccos(·)       (elbow branches)
φ₂ = arg(d_z + i d_x) − arg(l₂ + ρ e^{i(q₃+β)})      q₂ = φ₂ + 1.57
q₄ = φ₄ − φᵢ + 1.8791,   q₅ = Φ − φ₄ − 1.81           (angles taken mod 2π into the limits)
```

So the position+pitch IK solution set is a **1-parameter family (φ₄) × 2 elbow branches × 2 base branches**, intersected with the joint box. This is the exact meaning of "1-D null space" and of the "distinct basins" that make the numerical solvers need restarts (§4.5).

*Verified:* 2000 random configurations: the analytic solution recovers the original `q` in 2000/2000 cases (max error 2e-14 among recovered), and 300 × 5 random `φ₄` values give 0 pose mismatches (position and full rotation, tol 1e-9).

---

## 3. Task definition used by the solvers

* **Full pose (`InverseKinematicsSolver`)**: `e = [p_t − p ; rotvec(R_t Rᵀ)] ∈ ℝ⁶`, with `W = I` by default.
* **Position + pitch (`PositionPitchIkSolver`, default day-to-day path)**: `e_task = [e_pos ; ê_pitch · e_rot] ∈ ℝ⁴`, `ê_pitch = R_k ŷ` of the current iterate (re-evaluated every iteration), `J_task` as in §2.2.

**Orientation error.** `rotvec(R_t Rᵀ) = θu` (axis-angle of the relative rotation, base-frame), consistent with the spatial `J_ω`. [S] §3.7.3 (orientation error from the unit quaternion, `e_O = η ε_d − η_d ε − S(ε_d)ε`†) is `sin(θ/2)·u`-scaled; both have the same direction and the same first-order behaviour (≈ θu/2 vs θu), so only the effective gain differs. The rotation-vector is numerically safe at θ→0 (axis undefined but θu → 0).

**Corrected interpretation of "roll/yaw are irrelevant"** (`inverse_kinematics.md`, `position_pitch_ik.md`). The repo's docs say 6D IK has full column rank 5 with null space `{0}` away from singularities, and that dropping roll/yaw "opens" a null space. §2.2 shows rank 4 at all configurations: roll is identically zero and yaw is slaved to the position azimuth, so they are **dependent**, not free. Consequences:

* Position+pitch IK = full-pose IK *restricted to the reachable 4-D pose manifold*; the null space (1-D) exists for both.
* A generic 6-D target is off the manifold: its cost has a nonzero minimum (the "structurally unreachable" residual that `IkResult` reports) and `converged=false` with all restarts is the expected outcome; a target built from FK of a real `q` is on the manifold and converges.
* Solver behaviour is **unaffected** (damping handles the rank-deficient normal matrix); only the documented rationale changes. This also explains M3 Finding 2 (interpolating two reachable poses leaves the manifold) in one line: lerp+SLERP of two points of a 4-D submanifold of the 6-D SE(3) is generically off it.

---

## 4. Inverse kinematics

### 4.1 Continuous formulation: closed-loop IK and Lyapunov stability

With `e = x_d − x(q)`, constant `x_d`, `ė = −J_A q̇` (`J_A` = task Jacobian matching the error coordinates). Closed-loop inverse kinematics ([S] §3.7†, Jacobian (pseudo-)inverse algorithm): `q̇ = J^# K e`, `K = kI`, `k>0`. Take `V = ½ eᵀe`:

```
V̇ = eᵀ ė = − eᵀ J J^# K e
```

* Exact right pseudo-inverse `J^# = Jᵀ(JJᵀ)⁻¹` with `J` of full row rank (the 4×5 task): `J J^# = I₄`, so `V̇ = −k‖e‖² < 0`; `e` decays exponentially at rate `k` (global for the linearised error dynamics; the nonlinearity of `x(q)` makes the guarantee local and subject to joint limits).
* Damped `J^#_λ = Jᵀ(JJᵀ+λ²I)⁻¹` with `J = UΣVᵀ`: `J J^#_λ = U diag(σ_i²/(σ_i²+λ²)) Uᵀ`, so
  `V̇ = −k Σ_i σ_i²/(σ_i²+λ²) (u_iᵀ e)²  < 0` for every `σ_i>0` but with rate `k σ_i²/(σ_i²+λ²) → 0` as `σ_i → 0`: error components along near-singular directions decay slowly (the DLS tracking error), which is the price paid for bounded `q̇`.

### 4.2 Discrete step = Newton / Levenberg–Marquardt

Euler discretisation with `k Δt = 1` gives the Gauss–Newton step `Δq = J^# e`, which for the underdetermined 4×5 task is the **minimum-norm** Newton step. The damped step is the unique minimiser of

```
min_Δq  ‖J Δq − e‖²_W + λ² ‖Δq‖²    ⇒   (JᵀWJ + λ²I) Δq = JᵀW e   =   Jᵀ(JJᵀ+λ²W⁻¹)⁻¹ e  (W = I)
```

(Levenberg [Lev44] / Marquardt [Mar63] regularisation, equivalently a trust-region step; Nakamura–Hanafusa [NH86], Wampler [Wam86], Sugihara for weighted LM [Sug11]; Nocedal–Wright Ch. 10 [NW06]). The "push-through" identity `(JᵀJ+λ²I)⁻¹Jᵀ = Jᵀ(JJᵀ+λ²I)⁻¹` shows that the repo's two forms — left/normal equations for the 6×5 full-pose IK (`inverse_kinematics.cpp`) and right pseudo-inverse for the 4×5 task (`task_space_math.cpp`) — are **the same operator**; they differ in the task rows, not in the algebra. The old doc remark that the full-pose problem "needs the left form because it is overdetermined" is therefore a statement about the target, not a mathematical necessity.

### 4.3 Damping schedules

* **Adaptive LM (`InverseKinematicsSolver`)**: a step is accepted only if the weighted cost `eᵀWe` decreases; accepted ⇒ `λ ← 0.5λ`, rejected ⇒ `λ ← 3λ` (≤10 retries). This gives monotone cost decrease, hence convergence to a stationary point of `½eᵀWe` (local, not global) [NW06]. Clamp: `‖Δq‖_∞ ≤ 0.2 rad`, joint-limit clipping on `q`.
* **Variable (Chiaverini) damping (resolved rate, position+pitch IK)**: following the singular-value-dependent damping of [NH86], [CSE94], [Chi97],

```
λ²(σ_min) = 0                                  if σ_min ≥ ε
          = λ_max² (1 − (σ_min/ε)²)            otherwise
```

with `σ_min = √λ_min(J_task J_taskᵀ)`. Away from singularities this is the exact pseudo-inverse (no direction bias; the fixed-λ law scales direction `i` by `σ_i²/(σ_i²+λ²)` and so *rotates* a multi-component command, measured 2–4 % at λ = 0.05); near singularities it tapers continuously. **Heuristic flagged:** the code sets `λ_max = ε = 0.05` (one parameter, `damping`); the literature treats them as independent. `σ` has mixed units (m/rad in the position rows, rad/rad in the pitch row), so `ε = 0.05` is a scale choice, not a physical threshold. Given §2.3 it activates in ~10 % of the joint box.

### 4.4 Position + pitch solver details (`position_pitch_ik.cpp`)

Each iteration: `Δq = J_taskᵀ(J_taskJ_taskᵀ + λ²(σ_min)I)⁻¹ e_task`, clamped to `max_step = 0.2`. Control-flow additions that are not in the textbook scheme, each added to fix a measured failure (see `position_pitch_ik.md`): backtracking line search on `‖e_task‖²` when no posture task is active (stops wandering on unreachable interior samples); a final refinement of ≤6 Newton steps below 10 % tolerance; `converged` computed once from the true final `q`; restart selection with a tiered order *(collision-free ≻ within 10× tolerance ≻ lower residual / better alignment)*.

### 4.5 Why restarts are needed (consistent with §2.4)

The solution set of the 4-D task is a curve (φ₄) with 2 elbow × 2 base branches; local descent finds the branch nearest the seed. Restarts (`max_restarts = 20` in `inverse_kinematics.h` and `position_pitch_ik.h`; `inverse_kinematics.md` still says 10, stale) sample new basins uniformly within the joint limits. Measured (M2): fixed λ 40 %, adaptive λ alone 35 %, adaptive + restarts 100 % on 20 targets — adaptive damping improves how a basin is descended, restarts which basin.

### 4.6 Heuristic constants (not derived)

| constant | value | where |
|---|---|---|
| position / orientation tolerance | 1e-4 m / 1e-3 rad | both solvers |
| max iterations | 100 (full pose) / 200 (pos+pitch) | |
| `max_step` | 0.2 rad (∞-norm) | |
| λ₀, factors (full pose) | 0.05, ×0.5 / ×3 | standard LM defaults |
| `damping` = ε = λ_max (pos+pitch) | 0.05 | §4.3 |
| `max_inner_retries` | 10 / 5 | |
| drift cap multiplier | 10 × tolerance | |

---

## 5. Redundancy resolution

### 5.1 Resolved-rate with null-space projection

For velocity commands `ẋ = [v ; ω_pitch]`, `q̇ = J^#_λ ẋ + (I − J^#_λ J_task) q̇₀` (Whitney [Whi69]; gradient projection, Liégeois [Lie77]; [S] §3.4–3.5† redundancy analysis and inverse differential kinematics). With `J^#` exact, `J_task(I−J^#J_task) = 0`: the secondary motion `q̇₀` cannot disturb the task; with the damped inverse it does, by `O(λ²/σ²)` (the "leakage", measured negligible at λ=0.05 away from singularities). Output clamp to the velocity limits is a separate safety layer; the achieved/commanded ratio `‖J_task q̇_clamped‖/‖ẋ‖` is reported.

**Secondary objective** (link_4 vertical, `a = ẑ·(R₄ẑ)`):

```
R₄ = Rz(q₁)Ry(φ₄)  ⇒  a = cos φ₄,    ∂a/∂q_i = ẑ·(w_i × z_link) = −sin φ₄ ∂φ₄/∂q_i
w₁ = ẑ ⇒ ∂a/∂q₁ = 0,   w_{2..4} = ê_θ ⇒ ∂a/∂q_i = −sin φ₄ (i=2,3,4),   ∂a/∂q₅ = 0
```

(this is the closed form of the `q̇₀,i = k ẑ·(z_i × â)` of `resolved_rate.md`, whose triple-product argument shows the bias vanishes at `â = ẑ`). **Key structural fact:** `a` depends on joint space only through `S = q₂+q₃+q₄`; "link_4 vertical (up or down)" is the **hyperplane** `S ∈ {0.3079, 3.4495, 6.5911}` (within the limit range of S, [−2.22, 7.645]). On the 1-D self-motion curve this is an intersection of a curve with a hyperplane: generically isolated points that may not exist inside the joint box on the curve's branch, which is the mathematical reason some targets cannot be made vertical (the repo's "9 of 60 targets still more than 10° above the best found" and forward-mid ending at the `q₄=0` limit).

### 5.2 Why the original bias failed, and the posture optimiser

The old bias ascended the signed `a`; `a = −1` (vertical, pointing down) is a stationary point of `a` yet is the same physical goal (zero gravity moment on a vertical line), so `|a|` is the correct objective; with `|∇a| ∝ sin θ` a fixed-gain step converges at rate `∝ gain·sin²θ` (too slow for 200 iterations) — conclusions in `position_pitch_ik.md`, consistent with the formulas above.

**`OptimizePosture`** (after primary convergence) is projected-gradient ascent on `|a|` over the manifold `M = {q: f(q) = x_d}` with a retraction ([AMS08] for the concept):

```
g = sign(a)·∇a,   g_N = (I − V_r V_rᵀ) g    (projection on ker J_task; V_r = right singular vectors of non-zero σ)
step λ = min(max_step, (1−|a|)/|g_N|) along g_N/|g_N|,  then ≤8 damped Newton steps back to M
accept only if the primary task stays within tolerance and |a| strictly increases, else λ ← λ/2
```

*Rate (verification of the "tilt halves each iteration" claim):* near vertical `|a| = cos θ ≈ 1 − θ²/2`, `|g_N| = sin θ·|∇θ| ≈ cθ`, so `λ = (θ²/2)/(cθ) = θ/(2c)` and `Δθ = |∇θ| λ = θ/2`: this is Newton's method on a double root of `1 − |a|`, which converges linearly with ratio ½. Since `M` is 1-D, the algorithm is a 1-D line search along the curve; it cannot change branch (elbow flip) because that needs a joint-space path check.

*Independent bound (analytic scan, §2.4):* over 1000 uniform in-limit configurations, link_4 tilt `arccos|cos φ₄|` of the original configuration has mean 45.4°; the best tilt reachable **on the same q₁/r branch** (scanning φ₄ over 1441 values × 2 elbow branches within limits) has mean 15.3°, median 5.0°. The repo's own statistics (41.6° → 12.1° mean with 9.6° achievable) use a different target distribution, so the numbers are consistent in magnitude but not comparable one-to-one.

---

## 6. Workspace and collision

### 6.1 Reachable set

`workspace_sampler.cpp`: Monte Carlo draw of `q` uniformly in the box, FK, voxel counting. Extremal quantities converge (max reach 1.189 m ≤ 1.281 m bound; bbox X [−1.02,1.08], Y [−1.09,1.08], Z [−0.72,1.19] m); volume does not (0.18 m³ → 3.9 m³ depending on voxel size and samples), because the boundary shell is hit by few joint draws.

**Convergent alternative (derived here, not in the repo).** By §1.2 the reachable position set is a body of revolution about `z` of the planar set `W = {(r,z)}` of the 4R chain, swept over the `q₁` range (5.481 rad, i.e. 0.802 rad dead wedge) and over both signs of `r`. Pappus–Guldin: `V = ∫∫_W |r| · θ(r,z) dr dz`, with `θ` the swept angle (5.481 rad where only one sign of `r` is available, 2π where both are, since the two arcs `Q₁` and `Q₁+π` cover the circle). Sampling the **2-D** image (4 joints) with a 2-D occupancy grid is far better conditioned than a 3-D grid: 2×10⁶ / 8×10⁶ / 3.2×10⁷ samples at 1 cm cells give 4.79 / 4.86 / 4.89 m³; 0.5 cm cells at 3.2×10⁷ give 4.83 m³. Take **≈ 4.8 ± 0.1 m³** (position only, joint limits only, no self- or base-collision, occupancy-padded, so a mild overestimate) against the 7.05 m³ sphere of radius 1.189 m. Not verified against another method.

### 6.2 Base collision (`base_collision.cpp`)

Arm = capsules (radius 0.02) on the segments between consecutive joint origins and the tip; base = axis-aligned box `[−h,h]³`, deployed 0.4 m cube with the robot on its top face. Point–box distance is separable, `d²(x) = Σ_i max(0,|x_i|−h_i)²`; along `x(t) = q₀ + t u`, each axis term is `C¹` and convex, so `g(t) = d²(x(t))` is convex piecewise-quadratic with ≤6 interior breakpoints (where `x_i(t) = ±h_i`). Minimise each piece in closed form (vertex if inside, else end-points) and take the global minimum: exact, no iteration; collision iff `d²_min ≤ r²` (segment–AABB closest point, cf. [Eri05] Ch. 5). Skipping segment 0 is a provable optimisation: joints 1 and 2 lie on the `z` axis at `z = 0.03, 0.095 > 0` for every `q`, so segment 0 never enters the box `z ≤ 0`. Checked once per restart candidate, as the top tier of the selection order (§4.4).

---

## 7. Interface to trajectory generation (`arm_trajectory`)

* **Quintic time scaling** ([S] Ch. 4†, polynomial/point-to-point): `s(τ) = 10τ³−15τ⁴+6τ⁵`, `q(t) = q₀ + Δq·s(t/T)`. `s'(½) = 15/8` ⇒ `T_min,i = (15/8)|Δq_i|/v_max,i`, `T = max_i T_min,i` (synchronised arrival). Peak acceleration: `s'''(τ) = 60 − 360τ + 360τ² = 0 ⇒ τ = (1 ± 1/√3)/2`, `s''(0.2113) = 10/√3 = 5.7735`, so `|q̈|_max = 5.7735|Δq|/T²`; the acceleration-limited duration is `T ≥ √(5.7735|Δq|/a_max)` (verified numerically).
* **Cartesian paths**: position lerp + orientation SLERP [Sho85] with the same `s(τ)` ([S] §4.4†, operational-space trajectories), each sample IK-solved warm-started from the previous one (`max_restarts = 0` for interior samples to avoid basin jumps), duration rescaled until finite-differenced joint velocities respect the limits. Interior samples are best-effort because of §3 (generic path leaves the 4-D manifold; the position+pitch task is the part that *is* reachable).
* **Steady-state error loop** (`reach_correction.h`): Position Control Mode has no integral action, so loaded joints stop short. The node measures the FK error of the real joint states and re-aims at `x_target + c_k` with `c_{k+1} = c_k + g·e_k` (discrete integrator on the task-space error, capped at 6 cm; pitch compensation uses the same law about the tool `y` axis). Measured floor 4–7 mm ≈ arm repeatability under load.

---

## 8. Calibration chain and error budget

Four things stand between a joint reading and a pixel ray; only the last two are formal calibrations in the repo.

### 8.1 Joint mapping and URDF geometry (not calibrated)

* Joint zero, sign and tick scale per joint: `q = sign·(ticks − bias)·2π/4096`; `bias`/`sign`/limits are hand-set in `ArmInterface.yaml`; consistency with the URDF limits was audited (≤ 0.046 rad drift) and accepted. A zero-offset check script is proposed in `sim_to_real.md` but not built. Gripper: affine linear↔rotational placeholder, uncalibrated.
* **No kinematic calibration of the link parameters has been done** ([S] §2.9† kinematic calibration: identify geometric parameters by minimising the pose error `δx = J_θ δθ_param` over measured poses [RMR87], [HW96]); the URDF dimensions are nominal. First-order tip error: `δp = J_v δq` ≈ **1 mrad of joint error per metre of lever arm ≈ 1 mm**; one encoder tick (1.53 mrad) ≈ 1.5 mm/m. Two constants in the URDF are rounded where π-multiples are meant: pitch −1.57 vs −π/2 (0.80 mrad) and 3.1412 vs π (0.39 mrad), ≤ ~1 mm at 1.2 m reach, below the measured 4–7 mm repeatability but systematic, and trivially removable (flagged, not changed).
* The correction loop of §7 compensates this lumped error (sag, backlash, geometry, mapping) in task space, at the poses used; it is not a calibration.

### 8.2 Camera intrinsics (`arm_camera`, done on real images 2026-10-06)

Pinhole + Brown–Conrady radial/tangential ("plumb-bob") model [Bro71], planar target method of Zhang [Zha00]: `s [u v 1]ᵀ = K [R | t] [X Y 0 1]ᵀ` with `K = [[f_x,0,c_x],[0,f_y,c_y],[0,0,1]]` and distortion `(k₁,k₂,p₁,p₂)` (`k₃` fixed to 0), minimising reprojection error with `cv2.calibrateCamera`; the offline tool drops outlier views in rounds and reports per-parameter resampling spread. Real result: `f ≈ 1522 px`, `k₁ = −0.375`, `k₂ = 0.173`, 0.7 px RMS on the 53 sharp views (blurred views 2.2 px). Corner detection: classic chessboard detector first; the sector-based detector [DF18] labelled the board from opposite ends in half of the views, so a canonical orientation (origin cell dark) is enforced. Per-view pose: `solvePnP` with IPPE [CB14] followed by LM refinement (ChArUco: [GJ14] markers, plus interpolated corners). Principal-point ambiguity: GUI vs refit differ by 20 px (≈ 0.75°, ≈ 5 mm at 0.4 m); undecided by reprojection error, to be arbitrated by the hand-eye scatter. Texts: [HZ04] for the general model.

### 8.3 Hand–eye calibration, `X = T_ee,cam` (tooling built, real run deferred)

For samples `i` with `T_be,i` from FK/TF and `T_cB,i` (camera←board) from PnP, a fixed board gives `B = T_be,i X T_cB,i` for all `i`. Equating samples `i,j`:

```
(T_be,j⁻¹ T_be,i) X = X (T_cB,j T_cB,i⁻¹)       ⇔      A X = X B
R_A R_X = R_X R_B         (R_A − I) t_X = R_X t_B − t_A
```

The rotation equation implies `k_A = R_X k_B` (same angle, axes related by `R_X`). Identifiability ([TL89], [SA89]): **two motions with non-parallel rotation axes** determine `R_X` uniquely; the translation system `(R_{A_k} − I) t_X = ·` stacks with `ker(R_A − I) = span{k_A}`, so with two non-parallel axes the stacked matrix has trivial kernel (full rank 3). Methods available (`cv2.calibrateHandEye`): Tsai–Lenz [TL89], Park–Martin [PM94], Horaud–Dornaika [HD95], Andreff–Horaud–Espiau [AHE99], Daniilidis (dual quaternions) [Dan99]; optimal weighting in [SH06]. The repo solves with all five, reports the board-pose scatter and pixel RMS per method, refuses sets with < 20° of rotation or axis spread.

**Arm-specific excitation (derived):** the arm can rotate the camera only about the tool `y` axis (pitch, joints 2–5; constant `ŷ_ee`) and about base `z` (joint 1), which in the end-effector frame is `R_eeᵀẑ = Ry(φ₅)ᵀẑ = (−sin φ₅, 0, cos φ₅)`. Dot product with `ŷ`: exactly 0, so the two available axes are always **orthogonal** (the best case for conditioning) but only two exist: rotation about the tool/optical axis is never excited, which is why the procedure requires both large `q₁` swings and large pitch swings. The translation part inherits the arm's 4–7 mm error and sag.

### 8.4 ChArUco known-board mount

With board pose `T_bB` known: `X = T_be⁻¹ T_bB T_cB⁻¹`, one view suffices. A yaw error `δψ` of the entered board pose maps to a mount rotation error `δψ` plus translation error ≈ `d·δψ` (3° at 0.4 m ≈ 20 mm by this formula; the repo's synthetic test reports ~10 mm for a 3° case, so the formula is an upper-bound estimate); position error maps 1:1. Hand-eye on the same views gives an independent board-pose estimate (cross-check): in the synthetic test a 2° yaw error was recovered as 1.99°. Not yet run on the real arm.

### 8.5 Interim hand-measured mount (in use, 2026-10-07)

`X` built from measured lens offset and viewing directions: `R_bc = [x_img, y_img, z_opt]` with `x_img` orthogonalised against `z_opt` (Gram–Schmidt, skew reported/limited to 5°), `y = z × x`, `T_ee,cam = T_be⁻¹ · (R_bc, p_ee + offset)` (`static_mount.py`). Current values: `t = (−0.025, 0, −0.060) m`, yaw +90°, pitch = roll = 0 (camera upside down, looking along the tool axis). Expected error: a few mm and a few degrees; at 0.4 m, 3° ≈ 21 mm lateral at the target (`d·δθ`), which is above the arm's own accuracy — the first quantity to check when M7/M8 accuracy falls short (the calibration is deferred, not dropped).

---

## 9. Findings, limitations, recommendations

1. **Jacobian rank is 4 everywhere** (§2.2). Docs `inverse_kinematics.md` ("6>5 overdetermined, null space irrelevant"), `position_pitch_ik.md` ("full column rank 5, null space `{0}`") and `arm_trajectory` Finding 1 (σ≈1e-33 = singularity at `q=0`) should be rewritten: roll ≡ 0, yaw slaved to position, 1-D self-motion in both tasks, `q=0` is poorly conditioned (σ_min(J_task)=0.039) and at four joint limits rather than singular. No code change required. *Not edited here (docs were not part of the request).*
2. **Analytic IK exists** (§2.4): closed form per `(φ₄, elbow, base)` branch, verified. It could replace the stochastic restarts, enumerate branches (needed for the branch-switching item in the roadmap) and give an exact global optimum of the link_4 posture; not implemented.
3. **Heuristics to revisit:** `λ_max = ε` coupling and the mixed-unit σ (§4.3), tolerance/step constants (§4.6), the `10×` drift multiplier.
4. **URDF constants −1.57 / 3.1412** are rounded (≤ ~1 mm effect); worth a deliberate decision (keep as "as-built", or fix to exact values and re-run the FK reference tests, whose numbers depend on them).
5. **Workspace volume**: replace the 3-D voxel count by the planar Pappus estimate (§6.1), ≈ 4.8 m³; the 3-D figure in `workspace_identification.md` (0.18–0.26 m³) is an order-of-magnitude underestimate.
6. **Calibration status:** intrinsics real; hand-eye real run deferred; joint/URDF geometry never calibrated; end-to-end accuracy is limited first by the arm (4–7 mm) and by the interim mount (~20 mm at 0.4 m for a 3° error).
7. **Hardware-unverified** (per roadmap): posture optimiser and acceleration limit (simulation only), e-stop at full speed, the correction loop's pitch term on more poses.
8. The graphify output (`src/graphify-out`, dated 2026-10-05) was used only for orientation; every statement above was checked against source, URDF or numerics.

---

## 10. References

Primary:
* **[S]** B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Advanced Textbooks in Control and Signal Processing, Springer, 2009. Ch. 2 (rotations, homogeneous transforms, direct kinematics and DH, kinematic calibration, inverse kinematics of simple arms), Ch. 3 (geometric Jacobian, singularities, redundancy analysis, inverse differential kinematics with pseudo-inverse and damped least squares, IK algorithms incl. orientation error, manipulability), Ch. 4 (trajectory planning: polynomial and operational-space trajectories), Ch. 10 (visual servoing: pose estimation, camera and hand-eye calibration). Section numbers: see † note at the top.

Kinematics and redundancy:
* **[DH55]** J. Denavit, R. S. Hartenberg, "A kinematic notation for lower-pair mechanisms based on matrices," *J. Applied Mechanics* 22(2):215–221, 1955.
* **[LP17]** K. M. Lynch, F. C. Park, *Modern Robotics*, Cambridge University Press, 2017 (product of exponentials).
* **[Whi69]** D. E. Whitney, "Resolved motion rate control of manipulators and human prostheses," *IEEE Trans. Man-Machine Systems* 10(2):47–53, 1969.
* **[Lie77]** A. Liégeois, "Automatic supervisory control of the configuration and behavior of multibody mechanisms," *IEEE Trans. SMC* 7(12):868–871, 1977.
* **[NH86]** Y. Nakamura, H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control," *J. Dynamic Systems, Measurement, and Control* 108(3):163–171, 1986.
* **[Wam86]** C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods," *IEEE Trans. SMC* 16(1):93–101, 1986.
* **[CSE94]** S. Chiaverini, B. Siciliano, O. Egeland, "Review of the damped least-squares inverse kinematics with experiments on an industrial robot manipulator," *IEEE Trans. Control Systems Technology* 2(2):123–134, 1994. (verified)
* **[Chi97]** S. Chiaverini, "Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators," *IEEE Trans. Robotics and Automation* 13(3):398–410, 1997. (verified)
* **[Sug11]** T. Sugihara, "Solvability-unconcerned inverse kinematics by the Levenberg–Marquardt method," *IEEE Trans. Robotics* 27(5):984–991, 2011. (verified)
* **[Lev44]** K. Levenberg, *Quart. Appl. Math.* 2:164–168, 1944. **[Mar63]** D. W. Marquardt, *SIAM J. Appl. Math.* 11(2):431–441, 1963. **[NW06]** J. Nocedal, S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006 (Ch. 10).
* **[AMS08]** P.-A. Absil, R. Mahony, R. Sepulchre, *Optimization Algorithms on Matrix Manifolds*, Princeton Univ. Press, 2008 (retractions).
* Also relevant, not cited in text: Y. Nakamura, H. Hanafusa, T. Yoshikawa, "Task-priority based redundancy control of robot manipulators," *IJRR* 6(2):3–15, 1987; T. Yoshikawa, "Manipulability of robotic mechanisms," *IJRR* 4(2):3–9, 1985; A. Maciejewski, C. Klein, *J. Robotic Systems* 5(6):527–552, 1988; S. R. Buss, "Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods," 2004.

Geometry, trajectories:
* **[Sho85]** K. Shoemake, "Animating rotation with quaternion curves," *SIGGRAPH '85*, pp. 245–254. **[Eri05]** C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005.

Calibration and vision:
* **[RMR87]** Z. S. Roth, B. W. Mooring, B. Ravani, "An overview of robot calibration," *IEEE J. Robotics and Automation* 3(5):377–385, 1987. **[HW96]** J. M. Hollerbach, C. W. Wampler, "The calibration index and taxonomy for robot kinematic calibration methods," *IJRR* 15(6):573–591, 1996.
* **[Zha00]** Z. Zhang, "A flexible new technique for camera calibration," *IEEE TPAMI* 22(11):1330–1334, 2000. **[Bro71]** D. C. Brown, "Close-range camera calibration," *Photogrammetric Engineering* 37(8):855–866, 1971. **[HZ04]** R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge Univ. Press, 2004.
* **[CB14]** T. Collins, A. Bartoli, "Infinitesimal plane-based pose estimation," *IJCV* 109(3):252–286, 2014. (verified) **[GJ14]** S. Garrido-Jurado et al., "Automatic generation and detection of highly reliable fiducial markers under occlusion," *Pattern Recognition* 47(6):2280–2292, 2014. (verified) **[DF18]** A. Duda, U. Frese, "Accurate detection and localization of checkerboard corners for calibration," *BMVC* 2018. (verified)
* **[SA89]** Y. C. Shiu, S. Ahmad, "Calibration of wrist-mounted robotic sensors by solving homogeneous transform equations of the form AX=XB," *IEEE Trans. Robotics and Automation* 5(1):16–29, 1989. **[TL89]** R. Y. Tsai, R. K. Lenz, "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration," *IEEE Trans. Robotics and Automation* 5(3):345–358, 1989. (verified) **[PM94]** F. C. Park, B. J. Martin, "Robot sensor calibration: solving AX=XB on the Euclidean group," *IEEE TRA* 10(5):717–721, 1994. **[HD95]** R. Horaud, F. Dornaika, "Hand-eye calibration," *IJRR* 14(3):195–210, 1995. **[AHE99]** N. Andreff, R. Horaud, B. Espiau, "On-line hand-eye calibration," *3DIM*, 1999. **[Dan99]** K. Daniilidis, "Hand-eye calibration using dual quaternions," *IJRR* 18(3):286–298, 1999. **[SH06]** K. H. Strobl, G. Hirzinger, "Optimal hand-eye calibration," *IROS*, 2006.
* Conventions: ROS REP-103 / URDF specification (`rpy`, units); Eigen documentation (`AngleAxis`, `slerp`).

Verification status of the references: entries marked "(verified)" were confirmed by web search in this session (authors, venue, volume, pages); all others are given from memory and should be checked before being cited externally; the Siciliano section numbers (†) likewise.
