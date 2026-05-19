import numpy as np


def basic_step_analysis(samples: list[dict]) -> dict:
    if len(samples) < 20:
        return {"status": "not_enough_data"}
    t = np.array([x["timestamp_ms"] for x in samples], dtype=float) / 1000.0
    target = np.array([x["target"] for x in samples], dtype=float)
    feedback = np.array([x["feedback"] for x in samples], dtype=float)
    error = target - feedback
    step = float(np.max(np.abs(np.diff(target)))) if len(target) > 1 else 0.0
    steady_state_error = float(abs(error[-1]))
    abs_error = np.abs(error)
    if hasattr(np, "trapezoid"):
        iae = float(np.trapezoid(abs_error, t))
    else:
        iae = float(sum((abs_error[i] + abs_error[i - 1]) * (t[i] - t[i - 1]) * 0.5 for i in range(1, len(t))))
    if step < 1e-6:
        return {
            "status": "no_step",
            "step_size": step,
            "overshoot_pct": 0.0,
            "steady_state_error": steady_state_error,
            "oscillation_count": 0,
            "iae": iae,
        }
    overshoot = 0.0
    if target[-1] != 0:
        overshoot = max(0.0, (feedback.max() - target[-1]) / abs(target[-1]) * 100)
    zc = int(np.sum(np.diff(np.sign(error)) != 0))
    return {
        "status": "ok",
        "step_size": step,
        "overshoot_pct": float(overshoot),
        "steady_state_error": steady_state_error,
        "oscillation_count": zc,
        "iae": iae,
    }


def identify_motor_step_model(samples: list[dict]) -> dict:
    if len(samples) < 20:
        return {"status": "not_enough_data"}

    t = np.array([x["timestamp_ms"] for x in samples], dtype=float) / 1000.0
    target = np.array([x.get("target", 0.0) for x in samples], dtype=float)
    feedback = np.array([x.get("feedback", 0.0) for x in samples], dtype=float)
    output = np.array([x.get("output", 0.0) for x in samples], dtype=float)
    p_term = np.array([x.get("p_term", 0.0) for x in samples], dtype=float)
    i_term = np.array([x.get("i_term", 0.0) for x in samples], dtype=float)
    d_term = np.array([x.get("d_term", 0.0) for x in samples], dtype=float)

    step_index = int(np.argmax(np.abs(np.diff(target)))) + 1 if len(target) > 1 else 0
    target_step = float(target[step_index] - target[max(0, step_index - 1)])
    if abs(target_step) < 1e-6:
        return {"status": "no_step"}
    if abs(target[-1]) < 1e-6:
        return {"status": "zero_target"}

    pre_start = max(0, step_index - 8)
    pre_end = max(pre_start + 1, step_index)
    post_start = min(len(samples) - 1, step_index + max(5, len(samples) // 2))
    tail_count = max(5, len(samples) // 5)
    v0 = float(np.mean(feedback[pre_start:pre_end]))
    vss = float(np.mean(feedback[-tail_count:]))
    out0 = float(np.mean(output[pre_start:pre_end]))
    out_ss = float(np.mean(output[-tail_count:]))
    final_target = float(target[-1])
    peak = float(np.max(feedback[step_index:]))
    steady_error = float(final_target - vss)
    delta_v = vss - v0

    if abs(delta_v) < max(0.005, abs(final_target) * 0.03):
        return {
            "status": "weak_response",
            "target": final_target,
            "steady_feedback": vss,
            "steady_error": steady_error,
        }

    pid_correction_tail = float(np.mean((p_term + i_term + d_term)[-tail_count:]))
    feedforward_est = out_ss - pid_correction_tail
    if feedforward_est <= 1.0:
        feedforward_est = out_ss

    # Estimate dead time and first-order time constant from 10% and 63.2% crossings.
    y10 = v0 + 0.10 * delta_v
    y632 = v0 + 0.632 * delta_v
    post_t = t[step_index:]
    post_y = feedback[step_index:]
    if delta_v > 0.0:
        idx10 = np.where(post_y >= y10)[0]
        idx632 = np.where(post_y >= y632)[0]
    else:
        idx10 = np.where(post_y <= y10)[0]
        idx632 = np.where(post_y <= y632)[0]
    delay_s = float(post_t[idx10[0]] - t[step_index]) if len(idx10) else 0.0
    tau_s = float(post_t[idx632[0]] - t[step_index] - delay_s) if len(idx632) else float(t[-1] - t[step_index]) / 2.0
    delay_s = max(0.0, delay_s)
    tau_s = max(0.05, tau_s)

    # Use the estimated feedforward PWM as the step amplitude because the loop is feedforward + correction.
    # The pre-step buffer can contain old commands; do not subtract it for feedforward gain unless it is clearly at rest.
    pre_target = float(np.mean(target[pre_start:pre_end]))
    pre_output_dirty = abs(pre_target) > 1e-3 or out0 > 50.0
    effective_pwm_step = feedforward_est if pre_output_dirty else max(abs(feedforward_est - out0), 1.0)
    pwm_gain = abs(delta_v) / max(abs(effective_pwm_step), 1.0)
    if pwm_gain <= 1e-6:
        return {"status": "bad_gain"}

    # Conservative IMC/Lambda PI for first-order-plus-dead-time plant.
    # Use a large lambda because the car has drivetrain deadband, encoder filtering, and shared serial telemetry delay.
    lambda_s = max(8.0 * tau_s, 1.5)
    raw_kp = tau_s / (pwm_gain * (lambda_s + delay_s))
    ti = max(tau_s + 0.5 * delay_s, 0.05)
    raw_ki = raw_kp / ti

    current_kp = float(samples[-1].get("kp", 0.0))
    current_ki = float(samples[-1].get("ki", 0.0))
    if current_kp > 0.0:
        kp = min(raw_kp, current_kp * 1.8)
    else:
        kp = min(raw_kp, 500.0)
    if current_ki > 0.0:
        ki = min(raw_ki, current_ki * 2.0)
    else:
        ki = min(raw_ki, max(kp * 0.25, 80.0))

    if abs(vss) > 0.005:
        recommended_ff_pwm = feedforward_est * abs(final_target) / abs(vss)
    else:
        recommended_ff_pwm = final_target / pwm_gain
    recommended_ff_gain = recommended_ff_pwm / 7100.0
    overshoot_pct = 0.0
    if abs(final_target) > 1e-6:
        overshoot_pct = max(0.0, (peak - final_target) / abs(final_target) * 100.0)

    quality: list[str] = []
    if post_start >= len(samples) - 1:
        quality.append("采样窗口偏短，稳态估计可能不准")
    if abs(steady_error) > max(0.015, abs(final_target) * 0.08):
        quality.append("尾段仍有明显稳态误差，模型只作粗估")
    if overshoot_pct > 20.0:
        quality.append("超调较大，一阶模型拟合会偏保守")
    if feedforward_est < 1000.0:
        quality.append("估算前馈低于常见起转PWM，可能混入零速样本")
    if pre_output_dirty:
        quality.append("阶跃前窗口含非零目标/输出，已改用尾段比例估算前馈")

    return {
        "status": "ok",
        "target": final_target,
        "initial_feedback": v0,
        "steady_feedback": vss,
        "steady_error": steady_error,
        "peak_feedback": peak,
        "overshoot_pct": overshoot_pct,
        "steady_output": out_ss,
        "feedforward_pwm": feedforward_est,
        "recommended_ff_pwm": recommended_ff_pwm,
        "recommended_ff_gain": recommended_ff_gain,
        "plant_gain_mps_per_pwm": pwm_gain,
        "tau_s": tau_s,
        "delay_s": delay_s,
        "lambda_s": lambda_s,
        "raw_imc_kp": float(raw_kp),
        "raw_imc_ki": float(raw_ki),
        "recommended_kp": float(kp),
        "recommended_ki": float(ki),
        "recommended_kd": 0.0,
        "quality": quality,
    }
