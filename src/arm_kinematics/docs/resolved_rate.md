# Resolved-rate velocity control — derivation

Damped-least-squares resolved-rate (velocity-level) control on top of `forward_kinematics.md`'s `ForwardKinematicsSolver` and `jacobian.h`'s `ComputeGeometricJacobian`. Originally developed inside `arm_teleop` for M4's joystick jogging (see that package's git history and `docs/teleop.md` for the live-testing findings that shaped the fixes below), relocated here since it's pure Jacobian/kinematics math with no ROS or teleop coupling — a general velocity-control capability, not something specific to joystick input.

## Task Jacobian

Commanded task velocity, 4 rows — X, Y, Z linear (root frame) + pitch (about the end-effector's **own current local Y axis**, not a root-frame axis, so "pitch" means the same physical wrist motion regardless of the arm's current orientation):
```
ẋ_cmd = [vx, vy, vz, ωpitch]ᵀ ∈ ℝ⁴
```
Reusing the existing geometric Jacobian `J ∈ ℝ^{6×5}` (`ComputeGeometricJacobian`, unchanged), project the angular rows onto the tool's current pitch axis (`ê_pitch = R_ee·[0,1,0]ᵀ`) instead of commanding root-frame ωx,ωy,ωz directly:
```
J_task = [ J_v ; êₚᵀ J_ω ] ∈ ℝ^{4×5}
```
Task rows (4) < joints (5) here — **opposite** of the full-pose IK case (`inverse_kinematics.md`: 6>5, left/normal-equations pseudo-inverse). This is the classic redundant-manipulator case, so it's the **right** damped pseudo-inverse (Whitney resolved-rate control):
```
q̇ = J_taskᵀ (J_task J_taskᵀ + λ²I)⁻¹ ẋ_cmd
```

## Variable damping

**`λ` is variable, not fixed** (Chiaverini-style damping) — a fixed `λ` was reported live (during M4) as the jog target systematically deviating from the commanded pose even in clearly feasible configurations, root-caused as follows. Write `J_task`'s SVD as `UΣVᵀ`; the fixed-`λ` law scales the component of `ẋ_cmd` along singular direction `i` by `σᵢ/(σᵢ²+λ²)` instead of the true pseudo-inverse's `1/σᵢ`. That ratio, `σᵢ²/(σᵢ²+λ²)`, differs *per singular direction* — near 1 for a large `σᵢ`, well below 1 for a small one — so a command with components along multiple directions doesn't just get uniformly shrunk, it gets **rotated** away from `ẋ_cmd`'s true minimum-norm image. At a realistic `λ=0.05`, `test_resolved_rate.cpp`'s own comment measured "~2-4% relative error" from exactly this even away from any real singularity. Fix: only damp once actually close to singular, using `σ_min` = the smallest singular value of `J_task` (equivalently `√(smallest eigenvalue of J_task J_taskᵀ)`, since `J_task J_taskᵀ = UΣ²Uᵀ`):
```
λ²(σ_min) = 0                              if σ_min ≥ λ_thresh
          = λ_thresh² · (1 − (σ_min/λ_thresh)²)   otherwise
```
`λ_thresh` reuses the caller's damping parameter directly (no separate threshold param). Well-conditioned ticks (`σ_min ≥ λ_thresh`, i.e. any pose that's actually feasible) now get the **exact** undamped pseudo-inverse — zero direction bias — while a tick genuinely approaching a singularity still tapers `q̇` up to the same `λ_thresh²` ceiling as before, continuous at `σ_min = λ_thresh`, so the safety property (never blowing up) is unchanged. Verified by `ResolvedRateTest.RealisticDampingDoesNotBiasDirectionAwayFromSingularity`: the same command that gave ~2-4% error under the old fixed `λ` now tracks to >99.9% at the realistic `λ_thresh=0.05`.

Roll and yaw are **not commanded at all** — not held fixed, just absent from `J_task`'s rows, so they drift to whatever the minimum-norm `q̇` implies. (Confirmed decision for the teleop use case, not a simplification: holding them fixed would add two more constrained rows fighting the primary command near a singularity, reintroducing the same class of conflict that broke an earlier per-tick position-IK jogging design — see `arm_teleop/docs/teleop.md`'s "History" section.)

`q̇` is then clamped componentwise to `velocity_limits` (rad/s, `GetActuatedJointVelocityLimits()`) — a defensive hardware-safety clamp independent of the damping above — and the tick's actually-achieved task velocity is reported against what was commanded:
```
achieved_to_commanded_ratio = ‖J_task · q̇_clamped‖ / ‖ẋ_cmd‖
```
so a caller can warn when a limit or near-singular direction meaningfully reduced the commanded motion, without ever treating that as a hard failure (unlike position-IK, there is no convergence check here to fail — a saturated/near-singular direction just contributes less achieved velocity, never freezes).

## Null-space secondary task: bias toward a vertical link

This chain is redundant for the 4D task above (5 joints, 4 task rows), so the primary-task solve above is a *minimum-norm* choice among infinitely many `q̇` that achieve the same task velocity — it expresses no preference at all among them. Optional secondary objective, projected into the task's null space (Liegeois gradient projection), biasing a chosen link's own local +Z axis toward vertical whenever there's spare capacity to do so, without ever changing the achieved task velocity:
```
q̇ = q̇_primary + (I − J_task⁺J_task)·q̇₀
```
`J_task⁺ = J_taskᵀ(J_task J_taskᵀ+λ²I)⁻¹` is the same damped pseudo-inverse already computed for the primary solve (reused, not recomputed), so `N = I − J_task⁺J_task` is one extra matrix product. With `â = R_link(q)·[0,0,1]ᵀ` (the chosen link's own local Z axis in root frame) and cost `H(q) = −ẑ_world·â` (minimizing H maximizes vertical alignment):
```
∂H/∂qᵢ = −ẑ_world·(zᵢ × â),   i upstream of the chosen link       q̇₀,ᵢ = k·ẑ_world·(zᵢ × â)
```
`zᵢ` (joint i's own world-frame rotation axis) is read directly off columns of the *same* 6×5 Jacobian `J` already computed for the primary task — it depends only on the shared root→joint_i sub-chain, not on which tip a solver was built for, so no second Jacobian evaluation is needed. Only joints upstream of the chosen link affect its orientation (`q̇₀` is zero for the rest) — for `arm_teleop`'s default choice of `link_4`, that's joints 1–4; joint 5 is downstream of it and has zero effect, confirmed from the URDF (`joint_5`'s origin in `link_4`'s own frame is `xyz="0 0 0.336"`, i.e. `link_4`'s *local Z* is its own long axis — that's the axis this feature aligns to world Z, not an arbitrary choice).

`ComputeResolvedRateVelocity` takes this as two extra, defaulted parameters (`secondary_link_solver`, `nullspace_gain`) so every existing call site and test is unaffected unless it opts in. A caller constructs a second `ForwardKinematicsSolver` for whichever link it wants aligned, purely to get `â` via `ComputePose` on the upstream joint sub-vector.

**Disabling it**: `nullspace_gain = 0.0` (or a null `secondary_link_solver`) turns the secondary task off entirely, reducing to the primary-task-only law above.

**Gated on an actual commanded task velocity** (`‖ẋ_cmd‖ > 0`), not merely on some higher-level "active" state — reported and confirmed as a real bug in `arm_teleop`'s live hardware testing: with the bias active whenever a deadman-style enable was held, holding it with a centered stick made the arm visibly drift/crawl with nothing commanding it, since the bias follows a gradient with no target pose to settle into and so never converges to exactly zero on its own. A caller's own deadzone already makes `ẋ_cmd` exactly zero whenever there's no real input, so this check is exact, not a heuristic. `ResolvedRateNullspaceTest.NoBiasWithZeroPrimaryCommand` covers this directly.

**A caveat inherited from damping**: `N` is built from the *damped* pseudo-inverse, so `J_task·(N·q̇₀)` is only *approximately* zero in general (exactly zero only in the undamped limit) — the same kind of small, acknowledged bias as the primary task's own damped-tracking error (see `test_resolved_rate.cpp`'s `kNegligibleDamping` isolation). At `λ=0.05` this leakage into the primary task is negligible in practice (confirmed: `DoesNotDisturbThePrimaryTask` gtest), but it's worth knowing it's not a mathematical guarantee at a realistic damping magnitude.

## Testing

`test_resolved_rate.cpp`: zero command → zero velocity; a small linear/pitch command's *integrated, re-computed-via-FK* motion matches the commanded velocity (independent cross-check against `ComputePose`, not a self-consistency check against the same Jacobian call used to derive the velocity); the realistic-damping direction-bias regression above; velocity limits are respected and measurably reduce the achieved ratio. Plus (`ResolvedRateNullspaceTest`): `nullspace_gain=0` matches no secondary solver at all, byte-for-byte; a *zero* primary command produces *zero* velocity even with the bias enabled (the gating fix); a *nonzero* primary command's alignment improves specifically because of the bias, isolated by comparing with-bias vs. without-bias for the same primary command rather than a raw before/after (a raw comparison conflates the bias's own contribution with the primary motion's own incidental effect on alignment, which can go either way); the secondary task doesn't measurably disturb an active primary command.
