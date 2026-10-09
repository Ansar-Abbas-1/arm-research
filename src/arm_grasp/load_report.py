"""Analysis of a gripper recording (numpy, no ROS): what the motor load really does, so the flagged `close/*` values of
config/grasp.yaml can be set from data instead of guesses.

Inputs are three time series, each (t [s], values) with t increasing:
  * measured joint_6          (/joint_states_in)
  * commanded joint_6         (/joint_states_out, the stream of gripper commands)
  * |motor load| of joint_6 in % of max torque (/arm_interface/motor_load, sign follows the motor, so use |load|)

`analyse` returns a dict of facts, each one something a parameter depends on:
  load_rate_hz, load_max_gap_s        -> close/load_max_age (must exceed the largest gap)
  static_load_median/std, motion_load -> close/load_band, confirm_samples (noise), whether motion alone loads the motor
  tracking_error, load_slope          -> how the load grows with the command error (blocked servo), close/trim_speed
  time_to_target_s                    -> delay between crossing the target and the load showing it (with close_speed)
Nothing here assumes a gripper width or an object size.
"""
import numpy as np


def _interp(t, series):
    ts, vs = series
    return np.interp(t, ts, vs)


def rate_and_gap(t):
    """(median rate [Hz], largest gap [s]) of a timestamp series."""
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return 0.0, float("inf")
    dt = np.diff(t)
    return float(1.0 / np.median(dt)), float(dt.max())


def segments(mask, t, min_len=0.0):
    """Index ranges (i0, i1) of consecutive True in `mask` that last at least min_len seconds."""
    out, start = [], None
    for i, m in enumerate(list(mask) + [False]):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if t[i - 1] - t[start] >= min_len:
                out.append((start, i - 1))
            start = None
    return out


def analyse(measured, command, load, target=37.5, speed_eps=2e-4, static_min_s=1.0):
    """Facts from one recording. `measured`, `command`, `load` are (t, values); times in one clock."""
    tl, ll = np.asarray(load[0], float), np.abs(np.asarray(load[1], float))
    rate, gap = rate_and_gap(tl)
    out = {"load_rate_hz": rate, "load_max_gap_s": gap, "load_samples": int(len(tl))}
    if len(tl) < 3:
        return out
    pos = _interp(tl, measured)
    cmd = _interp(tl, command)
    vel_pos = np.gradient(pos, tl)
    vel_cmd = np.gradient(cmd, tl)
    moving = (np.abs(vel_cmd) > speed_eps) | (np.abs(vel_pos) > speed_eps)
    # static: neither the command nor the jaws move and nothing is pushed (no command error), for at least static_min_s
    still = ~moving & (np.abs(cmd - pos) < 5e-4)
    static_idx = np.concatenate([np.arange(i0, i1 + 1) for i0, i1 in segments(still, tl, static_min_s)] or [np.array([], int)])
    if len(static_idx):
        out["static_load_median"] = float(np.median(ll[static_idx]))
        out["static_load_std"] = float(np.std(ll[static_idx]))
        out["static_load_max"] = float(ll[static_idx].max())
    free = moving & (np.abs(cmd - pos) < 5e-4)           # moving without pushing against anything
    if free.sum() >= 3:
        out["motion_load_median"] = float(np.median(ll[free]))
        out["motion_load_max"] = float(ll[free].max())
    # tracking error while closing (the command is ahead of the jaws): measured - command > 0 for a closing gripper
    err = pos - cmd
    push = err > 5e-4
    if push.sum() >= 5:
        e = err[push]
        L = ll[push]
        A = np.vstack([e * 1000.0, np.ones_like(e)]).T                  # load [%] = slope [%/mm] * error [mm] + offset
        slope, offset = np.linalg.lstsq(A, L, rcond=None)[0]
        r = float(np.corrcoef(e, L)[0, 1]) if np.std(e) > 0 and np.std(L) > 0 else 0.0
        out.update(tracking_error_max_mm=float(1000.0 * e.max()), load_slope_pct_per_mm=float(slope), load_offset_pct=float(offset),
                   load_error_correlation=r, push_samples=int(push.sum()))
    # held load: the command is ahead of the jaws and nothing moves, for at least 1 s (the steady state of a grasp)
    held = np.concatenate([np.arange(i0, i1 + 1) for i0, i1 in segments(push & ~moving, tl, 1.0)] or [np.array([], int)])
    if len(held):
        out["held_load_median"] = float(np.median(ll[held]))
        out["held_load_max"] = float(ll[held].max())
    # time from the command being ahead of the jaws by 0.5 mm until the load reaches the target (the measurement delay seen from the load)
    hit = np.where(ll >= target)[0]
    if len(hit):
        out["first_time_at_target_s"] = float(tl[hit[0]] - tl[0])
        out["max_load"] = float(ll.max())
    else:
        out["max_load"] = float(ll.max())
    return out


def format_report(facts, target=37.5, limit=45.0, steady_max=40.0):
    """Human-readable lines for the facts, with the parameter each one informs."""
    f = facts
    lines = ["load samples: %d at %.1f Hz, largest gap %.3f s   (close/load_max_age must be larger than the gap)" % (f["load_samples"], f["load_rate_hz"], f["load_max_gap_s"])]
    if "static_load_median" in f:
        lines.append("static load: median %.1f %%, std %.2f %%, max %.1f %%   (noise: close/load_band, confirm_samples)" % (f["static_load_median"], f["static_load_std"], f["static_load_max"]))
    if "motion_load_median" in f:
        lines.append("load while moving freely: median %.1f %%, max %.1f %%   (motion alone must stay well under %.0f %%)" % (f["motion_load_median"], f["motion_load_max"], target))
    if "load_slope_pct_per_mm" in f:
        lines.append("load vs command error while pushing: %.2f %%/mm (offset %.1f %%), correlation %.2f, up to %.1f mm, %d samples   (close/trim_speed, how far the command may lead)" % (
            f["load_slope_pct_per_mm"], f["load_offset_pct"], f["load_error_correlation"], f["tracking_error_max_mm"], f["push_samples"]))
    if f.get("max_load", 0.0) >= limit:
        lines.append("!!! MAX LOAD %.1f %% REACHED THE PEAK LIMIT OF %.0f %%" % (f["max_load"], limit))
    if f.get("held_load_median") is not None and f["held_load_median"] >= steady_max:
        lines.append("!!! HELD LOAD %.1f %% IS AT OR ABOVE %.0f %%: the motor tripped its overload protection holding 43 to 48 %%" % (f["held_load_median"], steady_max))
    if "held_load_median" in f:
        lines.append("held load (steady state, command ahead of the still jaws): median %.1f %%, max %.1f %%   (must stay under %.0f %%)" % (f["held_load_median"], f["held_load_max"], steady_max))
    lines.append("max load %.1f %%%s" % (f.get("max_load", 0.0), (", reached %.0f %% at %.2f s" % (target, f["first_time_at_target_s"])) if "first_time_at_target_s" in f else ", never reached %.0f %%" % target))
    return lines
