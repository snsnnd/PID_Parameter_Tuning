#include "pid_debug.h"

/*
 * 一体化示例：直接使用 pid_controller_update + telemetry 上报
 *  - device_id = 7
 *  - channel_id = 0: 外环位置/角度环
 *  - channel_id = 1: 内环角速度环
 */

typedef struct {
    pid_controller_t pos_loop;
    pid_controller_t rate_loop;
} cascaded_ctrl_t;

static void fill_tel(pid_debug_telemetry_t *t,
                     uint32_t now_ms,
                     float target,
                     float feedback,
                     float output,
                     const pid_debug_param_t *p,
                     float p_term,
                     float i_term,
                     float d_term,
                     float extra1,
                     float extra2) {
    t->timestamp_ms = now_ms;
    t->target = target;
    t->feedback = feedback;
    t->error = target - feedback;
    t->output = output;
    t->kp = p->kp;
    t->ki = p->ki;
    t->kd = p->kd;
    t->p_term = p_term;
    t->i_term = i_term;
    t->d_term = d_term;
    t->extra1 = extra1;
    t->extra2 = extra2;
}

void cascaded_ctrl_init(cascaded_ctrl_t *ctrl,
                        const pid_debug_param_t *pos_param,
                        const pid_debug_param_t *rate_param) {
    if (ctrl == 0) return;
    pid_controller_init(&ctrl->pos_loop, pos_param);
    pid_controller_init(&ctrl->rate_loop, rate_param);
}

float cascaded_ctrl_step(cascaded_ctrl_t *ctrl,
                         uint32_t now_ms,
                         float target_pos_deg,
                         float measured_pos_deg,
                         float measured_rate_dps) {
    if (ctrl == 0) return 0.0f;

    /* 外环：角度/位置 -> 目标角速度 */
    float target_rate_dps = pid_controller_update(&ctrl->pos_loop, target_pos_deg, measured_pos_deg, now_ms);
    float pos_error = target_pos_deg - measured_pos_deg;
    float pos_p = ctrl->pos_loop.param.kp * pos_error;
    float pos_i = ctrl->pos_loop.param.ki * ctrl->pos_loop.integral;
    float pos_d = ctrl->pos_loop.prev_d_term;

    pid_debug_telemetry_t tel_pos = {0};
    fill_tel(&tel_pos, now_ms, target_pos_deg, measured_pos_deg, target_rate_dps,
             &ctrl->pos_loop.param, pos_p, pos_i, pos_d,
             measured_rate_dps, 0.0f);
    pid_debug_send_telemetry(7, 0, &tel_pos);

    /* 内环：角速度 -> 执行器输出(电流/扭矩/PWM) */
    float actuator_cmd = pid_controller_update(&ctrl->rate_loop, target_rate_dps, measured_rate_dps, now_ms);
    float rate_error = target_rate_dps - measured_rate_dps;
    float rate_p = ctrl->rate_loop.param.kp * rate_error;
    float rate_i = ctrl->rate_loop.param.ki * ctrl->rate_loop.integral;
    float rate_d = ctrl->rate_loop.prev_d_term;

    pid_debug_telemetry_t tel_rate = {0};
    fill_tel(&tel_rate, now_ms, target_rate_dps, measured_rate_dps, actuator_cmd,
             &ctrl->rate_loop.param, rate_p, rate_i, rate_d,
             measured_pos_deg, target_pos_deg);
    pid_debug_send_telemetry(7, 1, &tel_rate);

    return actuator_cmd;
}
