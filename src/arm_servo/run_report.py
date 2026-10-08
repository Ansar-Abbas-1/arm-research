"""Summary and plots of one recorded visual servo run (no ROS; matplotlib only for plot_run)."""
import numpy as np


def _first_time_below(t, e, threshold):
    idx = np.where(e < threshold)[0]
    return float(t[idx[0]] - t[0]) if len(idx) else None


def summarize(records):
    """Dict of the run's numbers: outcome, duration, error milestones, speeds, distances, vision age."""
    t = np.array([r["t"] for r in records])
    states = [r["state"] for r in records]
    servo = [i for i, s in enumerate(states) if s == "SERVOING"]
    last = records[-1]
    err = np.array([r["error_norm"] for r in records])
    # error_norm is only meaningful while servoing
    e_servo = err[servo] if servo else np.array([])
    t_servo = t[servo] if servo else np.array([])
    v = np.array([np.linalg.norm(r["v_cmd"]) for r in records])
    age = np.array([r["measurement_age"] for r in records if r["measurement_age"] >= 0])
    q = [r["q"] for r in records if r.get("q") is not None]
    max_joint_speed = None
    if len(q) > 5:
        tq = np.array([r["t"] for r in records if r.get("q") is not None])
        qq = np.array(q)
        # speed from a 0.3 s baseline (the status stream is 10 Hz and the encoders have 1-tick steps)
        step = max(1, int(round(0.3 / max(1e-6, np.median(np.diff(tq))))))
        if len(tq) > step:
            dq = (qq[step:] - qq[:-step]) / (tq[step:] - tq[:-step])[:, None]
            max_joint_speed = float(np.abs(dq).max())
    final = [r for r in records if r["state"] in ("DONE", "ABORTED")]
    out = {
        "outcome": last["state"], "reason": last.get("reason", ""), "left_view": bool(last.get("left_view", False)),
        "target_id": last.get("target_id", -1),
        "duration_s": float(t[-1] - t[0]),
        "servoing_s": float(t_servo[-1] - t_servo[0]) if len(t_servo) > 1 else 0.0,
        "initial_error_mm": float(1000 * e_servo[0]) if len(e_servo) else None,
        "final_error_mm": float(1000 * e_servo[-1]) if len(e_servo) else None,
        "time_to_20mm_s": _first_time_below(t_servo, e_servo, 0.020) if len(e_servo) else None,
        "time_to_10mm_s": _first_time_below(t_servo, e_servo, 0.010) if len(e_servo) else None,
        "time_to_5mm_s": _first_time_below(t_servo, e_servo, 0.005) if len(e_servo) else None,
        "max_commanded_speed_mm_s": float(1000 * v.max()),
        "max_joint_speed_rad_s": max_joint_speed,
        "min_ee_distance_m": float(min(r["ee_distance"] for r in records if r["ee_distance"] < 100)) if any(r["ee_distance"] < 100 for r in records) else None,
        "min_camera_distance_m": float(min(r["camera_distance"] for r in records if r["camera_distance"] < 100)) if any(r["camera_distance"] < 100 for r in records) else None,
        "max_measurement_age_s": float(age.max()) if len(age) else None,
        "mean_measurement_age_s": float(age.mean()) if len(age) else None,
        "final_pitch_error_deg": float(np.degrees(abs(last.get("pitch_error_rad", 0.0)))),
        "ground_known": bool(last.get("ground_known", False)),
        "records": len(records),
    }
    if final:
        out["ended_after_s"] = float(final[0]["t"] - t[0])
    return out


def format_summary(s):
    def f(x, fmt):
        return "n/a" if x is None else fmt % x
    lines = ["outcome: %s%s%s" % (s["outcome"], " (%s)" % s["reason"] if s["reason"] else "",
                                  " (the marker left the view at the stand-off)" if s["left_view"] else ""),
             "target marker %d; run %.1f s, servoing %.1f s" % (s["target_id"], s["duration_s"], s["servoing_s"]),
             "error: start %s mm, final %s mm; below 20 / 10 / 5 mm after %s / %s / %s s" % (
                 f(s["initial_error_mm"], "%.1f"), f(s["final_error_mm"], "%.1f"), f(s["time_to_20mm_s"], "%.1f"),
                 f(s["time_to_10mm_s"], "%.1f"), f(s["time_to_5mm_s"], "%.1f")),
             "final pitch error %.1f deg; max commanded speed %.1f mm/s; max joint speed %s rad/s" % (
                 s["final_pitch_error_deg"], s["max_commanded_speed_mm_s"], f(s["max_joint_speed_rad_s"], "%.3f")),
             "closest approach: end effector %s m, camera %s m; measurement age mean %s s, max %s s" % (
                 f(s["min_ee_distance_m"], "%.3f"), f(s["min_camera_distance_m"], "%.3f"),
                 f(s["mean_measurement_age_s"], "%.2f"), f(s["max_measurement_age_s"], "%.2f"))]
    return "\n".join(lines)


def plot_run(records, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r["t"] for r in records]) - records[0]["t"]
    err = np.array([r["error_ee"] for r in records])
    en = np.array([r["error_norm"] for r in records])
    v = np.array([r["v_cmd"] for r in records])
    servo = np.array([r["state"] == "SERVOING" for r in records])
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    ax[0].plot(t[servo], 1000 * err[servo]); ax[0].legend(["x", "y", "z"], ncol=3); ax[0].set_ylabel("error in E [mm]"); ax[0].grid(alpha=.3)
    ax[1].semilogy(t[servo], 1000 * np.maximum(en[servo], 1e-3)); ax[1].axhline(5, ls="--", c="g"); ax[1].set_ylabel("|error| [mm] (5 mm line)"); ax[1].grid(alpha=.3, which="both")
    ax[2].plot(t, 1000 * v); ax[2].set_ylabel("commanded v in E [mm/s]"); ax[2].legend(["x", "y", "z"], ncol=3); ax[2].grid(alpha=.3)
    ee = np.array([min(r["ee_distance"], 1.0) for r in records]); cam = np.array([min(r["camera_distance"], 1.0) for r in records])
    ax[3].plot(t, ee, label="end effector to marker"); ax[3].plot(t, cam, label="camera to marker"); ax[3].axhline(0.08, ls="--", c="r", label="8 cm minimum")
    ax[3].set_ylabel("distance [m]"); ax[3].set_xlabel("time [s]"); ax[3].legend(); ax[3].grid(alpha=.3)
    fig.suptitle("visual servo run: %s" % records[-1]["state"])
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)
