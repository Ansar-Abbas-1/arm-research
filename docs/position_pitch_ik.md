# Position + pitch IK — derivation

Builds on [`inverse_kinematics.md`](inverse_kinematics.md) (why full 6D pose IK is overdetermined on this chain) and [`resolved_rate.md`](resolved_rate.md) (the damped right-pseudo-inverse + null-space math this solver reuses).

## Why this exists

`InverseKinematicsSolver` targets a full 6D pose with 5 joints: 6 task rows over 5 columns, **overdetermined**. Away from singularities the Jacobian has full column rank (5), and a full-column-rank matrix is injective — `J·Δq = 0 ⟹ Δq = 0`. The null space is `{0}`. There is no spare joint freedom for a secondary objective (e.g. "keep `link_4` vertical") to live in when the primary task is a full 6D pose: every joint is already spoken for.

This arm's roll and yaw are mechanically irrelevant to the task at hand (confirmed: M2 found it "can't reach a general 3D orientation, only position + one in-plane angle" — that one angle is pitch, about the end-effector's own local Y axis, the same convention `ComputeResolvedRateVelocity` already uses for velocity control). Dropping roll/yaw from the IK target — the same reduction resolved-rate already made — turns the task 4D over 5 joints: **redundant**, with a real (generically 1-dimensional) null space a secondary task can use, exactly as in resolved-rate. `PositionPitchIkSolver` applies that reduction to the *iterative position solve*, not just velocity control, so a caller configured with a secondary posture gets it applied on every `Solve()`, automatically — not something that has to be separately streamed or gated behind a live control loop.

## Task and iteration

At each Newton iteration, evaluated at the current iterate `q_k`:
```
pose_k = FK(q_k)
e_pos = p_target − p_k                                    ∈ ℝ³
ê_pitch = R_k · [0,1,0]ᵀ                                   (current iterate's own local Y axis)
e_rot = RotationVector(R_target · R_kᵀ)                    ∈ ℝ³  (full angle-axis pose error)
e_pitch = ê_pitch · e_rot                                  ∈ ℝ   (component of the rotation error along pitch)
e_task = [e_pos ; e_pitch] ∈ ℝ⁴
J_task = [ J_v ; ê_pitchᵀ J_ω ] ∈ ℝ^{4×5}                  (identical construction to resolved_rate.md)
```
`e_pitch`/`J_task` use the **current** iterate's own local Y axis at every step, not a fixed reference — the same body-frame convention `ComputeResolvedRateVelocity` uses, kept consistent between the velocity-level and position-level solvers rather than inventing a second definition of "pitch."

Each step is a single damped right-pseudo-inverse solve (`task_space_math.h`, shared with `ComputeResolvedRateVelocity`) — Chiaverini variable damping, so a well-conditioned iterate takes the exact undamped Gauss-Newton step, tapering only near an actual singularity:
```
Δq = J_taskᵀ (J_task J_taskᵀ + λ²(σ_min) I)⁻¹ e_task
```
clamped to `max_step` (inf-norm) as a safety margin, same convention as `InverseKinematicsSolver`. No accept/reject Levenberg-Marquardt logic (unlike `InverseKinematicsSolver`): Chiaverini damping already keeps the step well-behaved near a singularity, so there's nothing an LM-style cost-based rejection would add for this redundant, well-posed 4D task.

**Convergence** is position + pitch only, reported the same `[position(3); rotation(3)]` residual layout as `InverseKinematicsSolver::IkResult` — but only the position and pitch-axis-projected component of that 3-vector rotation residual are actually driven toward zero; the remaining two rotation components are reported for information only (whatever roll/yaw the run happened to land on).

**Restarts** on non-convergence use the same policy as `InverseKinematicsSolver::Solve` (random reseed within joint limits, keep the best/lowest-residual result) — this chain's distinct solution basins are a property of the chain, not of which task dimension is being solved for.

## Null-space secondary task, applied at every step

If `nullspace_secondary_link_solver`/`nullspace_gain` are configured (construction-time, not per-call — see the header for why), the same `qdot0` bias `ComputeResolvedRateVelocity` computes (Liegeois gradient toward vertical, see `resolved_rate.md`) is recomputed fresh each iteration (the secondary link's pose changes as `q` does) and projected into `J_task`'s null space, added to the primary Newton step. This has a genuine fixed point at exact vertical alignment: writing the bias as `q̇₀,ᵢ = k · ẑ_world · (zᵢ × â)`, at `â = ẑ_world` the scalar triple product `ẑ_world · (zᵢ × ẑ_world) = zᵢ · (ẑ_world × ẑ_world) = 0` for every `i` (cyclic identity on the scalar triple product) — so the bias vanishes exactly there, not just approximately. Driving it iteratively to convergence (rather than applying it for one control tick) is therefore well-posed: the solver doesn't just nudge toward vertical, it actually settles there, subject to whatever the position+pitch constraint still allows.

`nullspace_gain` here plays the role of a per-iteration **step** gain (radians added directly, every Newton step), not a rad/s **rate** the way it does in `ComputeResolvedRateVelocity` (where the caller multiplies by `dt`, ~0.02s at 50Hz, before it becomes an actual position change) — the two contexts need separately-tuned values, confirmed the hard way: `nullspace_gain=0.4` (resolved-rate's own default, the natural value to reach for) made this solver fail to converge at all on an otherwise-easy target, since applied without any `dt` scaling it's a ~50x larger per-step contribution than the same number means in the velocity context. `0.05` converges cleanly on the same target. `PositionPitchIkOptions::nullspace_gain` still defaults to `0.0` (off), same as `ComputeResolvedRateVelocity`'s default — this is a caution for whoever turns it on, not a change to the default policy.

## Testing

`test_position_pitch_ik.cpp`: a target's position+pitch, reconstructed via FK from a known random reachable configuration, is matched by the converged solution (independent check — the *joint values* need not match the seed configuration, since roll/yaw and the redundant DOF mean multiple configurations can share the same position+pitch); a small random-target convergence benchmark (mirroring `inverse_kinematics.md`'s own, at a smaller N to keep the suite fast); the null-space bias measurably improves the converged configuration's vertical alignment relative to the same target solved without it (with-vs-without isolation, same methodology as `ResolvedRateNullspaceTest`); `nullspace_gain=0` matches no secondary solver at all; and a target whose orientation is deliberately *not* on this arm's reachable 5D submanifold (an arbitrary roll/yaw baked into the full pose handed to `InverseKinematicsSolver`, which fails to converge on it) still converges here, since only its position+pitch component — which *is* independently reachable — is ever asked for.
