import numpy as np


def basic_step_analysis(samples: list[dict]) -> dict:
    if len(samples) < 20:
        return {"status": "not_enough_data"}
    t = np.array([x["timestamp_ms"] for x in samples], dtype=float) / 1000.0
    target = np.array([x["target"] for x in samples], dtype=float)
    feedback = np.array([x["feedback"] for x in samples], dtype=float)
    error = target - feedback
    step = np.max(np.abs(np.diff(target))) if len(target) > 1 else 0
    overshoot = 0.0
    if target[-1] != 0:
        overshoot = max(0.0, (feedback.max() - target[-1]) / abs(target[-1]) * 100)
    steady_state_error = float(abs(error[-1]))
    zc = int(np.sum(np.diff(np.sign(error)) != 0))
    abs_error = np.abs(error)
    if hasattr(np, "trapezoid"):
        iae = float(np.trapezoid(abs_error, t))
    else:
        iae = float(sum((abs_error[i] + abs_error[i - 1]) * (t[i] - t[i - 1]) * 0.5 for i in range(1, len(t))))
    return {
        "status": "ok",
        "step_size": float(step),
        "overshoot_pct": float(overshoot),
        "steady_state_error": steady_state_error,
        "oscillation_count": zc,
        "iae": iae,
    }
