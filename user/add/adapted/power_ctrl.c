#include "power_ctrl.h"
#include <math.h>

/* ---- 常量, 与原工程 PowerController.hpp/cpp 一致 ---- */
#define REFEREE_FULL_BUFF_SET   60.0f   /* 裁判缓冲能量上目标 J */
#define REFEREE_BASE_BUFF_SET   50.0f   /* 裁判缓冲能量下目标 J */
#define CAP_FULL_BUFF_SET       230.0f  /* 电容能量上目标 (0~255), 约 90% */
#define CAP_BASE_BUFF_SET       30.0f   /* 电容能量下目标 */
#define ERR_DIST_SET            20.0f   /* Σerror 高于此值完全按误差分配 */
#define PROP_DIST_SET           15.0f   /* Σerror 低于此值完全按功率比例分配 */
#define MAX_CAP_POWER_OUT       300.0f  /* 电容最大放电功率 */
#define CAP_OFFLINE_RUNOUT_THRESHOLD 43.0f
#define MAX_REFEREE_BUFF        60.0f
#define CAP_REFEREE_BOTH_GG_COE 0.85f
#define POWER_PD_KP             50.0f
#define POWER_PD_KD             0.2f
#define POWER_PD_D_ALPHA        0.2f
#define RLS_DEAD_ZONE_W         5.0f
#define MOTOR_DISC_HOLD_CYCLES  1000    /* 电机断连后按最后状态继续计功率的周期数 */

static const uint8_t HERO_LIMIT_HP_FIRST[10]     = {55, 60, 65, 70, 75, 80, 85, 90, 100, 120};
static const uint8_t INFANTRY_LIMIT_HP_FIRST[10] = {45, 50, 55, 60, 65, 70, 75, 80, 90, 100};
#define SENTRY_LIMIT 100.0f

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float fmaxf_(float a, float b) { return a > b ? a : b; }
static inline int   feq0(float a) { return fabsf(a) < 1e-5f; }

/* ---- 简易 PD, 对应原工程 Core::Control::PID 在能量环的用法 ---- */
static void pd_init(pc_pd_t *p, float kp, float kd, float out_max, float d_alpha)
{
    p->kp = kp; p->kd = kd; p->out_max = out_max; p->d_alpha = d_alpha;
    p->last_err = 0.0f; p->d_filt = 0.0f; p->first = 1;
}
static void pd_reset(pc_pd_t *p) { p->last_err = 0.0f; p->d_filt = 0.0f; p->first = 1; }
static float pd_run(pc_pd_t *p, float set, float fb, float dt)
{
    float err = set - fb;
    float d = 0.0f;
    if (!p->first && dt > 0.0f) d = (err - p->last_err) / dt;
    p->first = 0;
    p->last_err = err;
    p->d_filt = p->d_alpha * d + (1.0f - p->d_alpha) * p->d_filt;
    return clampf(p->kp * err + p->kd * p->d_filt, -p->out_max, p->out_max);
}

/* ============================================================ */
void pc_init(pc_t *pc, const pc_config_t *cfg)
{
    int i;
    pc->cfg = *cfg;
    pc->k1 = cfg->k1; pc->k2 = cfg->k2; pc->k3 = cfg->k3;
    rls2_init(&pc->rls, cfg->rls_delta, cfg->rls_lambda, cfg->k1, cfg->k2);
    pd_init(&pc->pd_base, POWER_PD_KP, POWER_PD_KD, MAX_CAP_POWER_OUT, POWER_PD_D_ALPHA);
    pd_init(&pc->pd_full, POWER_PD_KP, POWER_PD_KD, MAX_CAP_POWER_OUT, POWER_PD_D_ALPHA);

    pc->latest_level = (cfg->division == PC_SENTRY) ? 10 : 1;
    pc->referee_max_power = 45.0f;
    pc->min_max_power = 30.0f;
    pc->power_upper_limit = CAP_OFFLINE_RUNOUT_THRESHOLD + MAX_CAP_POWER_OUT;
    pc->base_max_power = pc->full_max_power = pc->user_max_power = 45.0f;
    pc->power_buff = 0.0f;
    pc->measured_power = pc->estimated_power = pc->effective_power = 0.0f;
    pc->cap_energy_out = 0;
    for (i = 0; i < PC_WHEEL_NUM; i++) pc->disc_cnt[i] = 0;
}

void pc_set_user_max_power(pc_t *pc, float power_w)
{
    pc->user_max_power = clampf(power_w, pc->min_max_power, pc->power_upper_limit);
}

/* ============================================================
 * 后台 1 kHz: 对应原工程 powerDaemon()
 * ============================================================ */
void pc_daemon_update(pc_t *pc, const pc_motor_fb_t fb[PC_WHEEL_NUM], const pc_ext_fb_t *ext, float dt)
{
    int i;
    uint8_t cap_ok = ext->cap_ok;
    uint8_t ref_ok = ext->referee_ok;

    /* 1. 裁判上限 + 用户可配置上限 (断连时按等级查表, 保守处理) */
    if (ref_ok) {
        pc->referee_max_power = fmaxf_(ext->referee_power_limit_w, CAP_OFFLINE_RUNOUT_THRESHOLD);
        pc->latest_level = (ext->robot_level > 10 || ext->robot_level < 1) ? 1 : ext->robot_level;
        if (cap_ok)
            pc->power_upper_limit = pc->referee_max_power + MAX_CAP_POWER_OUT;
        else
            pc->power_upper_limit = pc->referee_max_power +
                                    POWER_PD_KP * (sqrtf(REFEREE_FULL_BUFF_SET) - sqrtf(REFEREE_BASE_BUFF_SET));
    } else {
        switch (pc->cfg.division) {
        case PC_HERO:     pc->referee_max_power = HERO_LIMIT_HP_FIRST[pc->latest_level - 1]; break;
        case PC_INFANTRY: pc->referee_max_power = INFANTRY_LIMIT_HP_FIRST[pc->latest_level - 1]; break;
        default:          pc->referee_max_power = SENTRY_LIMIT; break;
        }
        pc->power_upper_limit = cap_ok ? pc->referee_max_power + MAX_CAP_POWER_OUT
                                       : pc->referee_max_power * CAP_REFEREE_BOTH_GG_COE;
    }
    pc->min_max_power = pc->referee_max_power * 0.8f;

    /* 2. 能量反馈源: 电容优先, 其次裁判缓冲能量 */
    float full_set, base_set;
    if (cap_ok) {
        pc->power_buff = ext->cap_energy_255;
        full_set = CAP_FULL_BUFF_SET; base_set = CAP_BASE_BUFF_SET;
    } else {
        if (ref_ok) pc->power_buff = ext->referee_buffer_j;
        full_set = REFEREE_FULL_BUFF_SET; base_set = REFEREE_BASE_BUFF_SET;
    }

    /* 电容断连时, 用裁判数据判断电容是否已耗尽 (耗尽后裁判功率才可信) */
    pc->cap_energy_out = (!cap_ok && ref_ok &&
                          ext->referee_buffer_j < MAX_REFEREE_BUFF &&
                          ext->referee_chassis_power_w > CAP_OFFLINE_RUNOUT_THRESHOLD);

    /* 3. 能量环: 在 sqrt(E) 域做 PD, 得到功率上限的两个边界 */
    if (!cap_ok && !ref_ok) {
        pc->base_max_power = pc->full_max_power = pc->referee_max_power * CAP_REFEREE_BOTH_GG_COE;
        pd_reset(&pc->pd_base);
        pd_reset(&pc->pd_full);
    } else {
        pc->base_max_power = fmaxf_(pc->referee_max_power - pd_run(&pc->pd_base, sqrtf(base_set), sqrtf(pc->power_buff), dt),
                                    pc->min_max_power);
        pc->full_max_power = fmaxf_(pc->referee_max_power - pd_run(&pc->pd_full, sqrtf(full_set), sqrtf(pc->power_buff), dt),
                                    pc->min_max_power);
    }
    pc->user_max_power = clampf(pc->user_max_power, pc->min_max_power, pc->power_upper_limit);

    /* 4. 用反馈力矩/转速估计当前底盘功率 (电机刚断连 1 s 内按最后状态计) */
    float eff = 0.0f, phi[2] = {0.0f, 0.0f};
    for (i = 0; i < PC_WHEEL_NUM; i++) {
        if (fb[i].connected) pc->disc_cnt[i] = 0;
        else if (pc->disc_cnt[i] < MOTOR_DISC_HOLD_CYCLES) pc->disc_cnt[i]++;
        if (pc->disc_cnt[i] < MOTOR_DISC_HOLD_CYCLES) {
            eff    += fb[i].torque_nm * fb[i].av_rad_s;
            phi[0] += fabsf(fb[i].av_rad_s);
            phi[1] += fb[i].torque_nm * fb[i].torque_nm;
        }
    }
    pc->effective_power = eff;
    pc->estimated_power = pc->k1 * phi[0] + pc->k2 * phi[1] + eff + pc->k3;

    /* 5. 实测功率源 */
    uint8_t meas_valid = 0;
    if (cap_ok) { pc->measured_power = ext->cap_chassis_power_w; meas_valid = 1; }
    else if (ref_ok && pc->cap_energy_out) { pc->measured_power = ext->referee_chassis_power_w; meas_valid = 1; }
    else pc->measured_power = pc->estimated_power;

    /* 6. RLS 在线辨识 k1,k2 (只在有真实功率反馈 + 死区外 + 非负功时) */
    if (pc->cfg.rls_enabled && meas_valid && fabsf(pc->measured_power) > RLS_DEAD_ZONE_W &&
        !(!cap_ok && pc->estimated_power < 0.0f)) {
        rls2_update(&pc->rls, phi, pc->measured_power - eff - pc->k3);
        pc->k1 = fmaxf_(pc->rls.theta[0], 1e-5f);
        pc->k2 = fmaxf_(pc->rls.theta[1], 1e-5f);
    }
}

/* ============================================================
 * 功率环: 对应原工程 getControlledOutput()
 * ============================================================ */
void pc_limit(pc_t *pc, const pc_wheel_in_t in[PC_WHEEL_NUM], const uint8_t connected[PC_WHEEL_NUM], float out[PC_WHEEL_NUM])
{
    int i;
    const float k0 = pc->cfg.k0;
    const float k3n = pc->k3 / (float)PC_WHEEL_NUM;

    float max_power = clampf(pc->user_max_power, pc->full_max_power, pc->base_max_power);
    float cmd_power[PC_WHEEL_NUM], err[PC_WHEEL_NUM];
    float sum_cmd = 0.0f, sum_err = 0.0f, sum_required = 0.0f, allocatable = max_power;

    /* 1. 把 PID 输出代入模型, 预测每个轮子将消耗的功率 */
    for (i = 0; i < PC_WHEEL_NUM; i++) {
        if (!connected[i]) { cmd_power[i] = 0.0f; err[i] = 0.0f; continue; }
        float tau = in[i].pid_output * k0;
        float w   = in[i].cur_av;
        cmd_power[i] = tau * w + fabsf(w) * pc->k1 + tau * tau * pc->k2 + k3n;
        sum_cmd += cmd_power[i];
        err[i] = fabsf(in[i].set_av - in[i].cur_av);
        if (cmd_power[i] <= 0.0f || feq0(cmd_power[i]))
            allocatable -= cmd_power[i];     /* 负功(回收)的轮子把功率还给池子 */
        else {
            sum_err += err[i];
            sum_required += cmd_power[i];
        }
    }
    pc->sum_cmd_power = sum_cmd;
    pc->max_power_limited = max_power;

    /* 2. 不超功率: 直接沿用 PID 输出 */
    if (sum_cmd <= max_power) {
        for (i = 0; i < PC_WHEEL_NUM; i++) out[i] = connected[i] ? in[i].pid_output : 0.0f;
        return;
    }

    /* 3. 超功率: 误差分配 与 等比分配 的置信度混合 */
    float conf;
    if (sum_err > ERR_DIST_SET) conf = 1.0f;
    else if (sum_err > PROP_DIST_SET) conf = (sum_err - PROP_DIST_SET) / (ERR_DIST_SET - PROP_DIST_SET);
    else conf = 0.0f;

    for (i = 0; i < PC_WHEEL_NUM; i++) {
        if (!connected[i]) { out[i] = 0.0f; continue; }
        if (cmd_power[i] <= 0.0f || feq0(cmd_power[i])) { out[i] = in[i].pid_output; continue; }

        float w_err  = (sum_err > 0.0f) ? err[i] / sum_err : 0.0f;
        float w_prop = (sum_required > 0.0f) ? cmd_power[i] / sum_required : 0.0f;
        float weight = conf * w_err + (1.0f - conf) * w_prop;
        float p_i    = weight * allocatable;

        /* 4. 解 k2 τ² + ω τ + (k1|ω| + k3/4 - P_i) = 0, 取与 PID 输出同号的根 */
        float w = in[i].cur_av;
        float delta = w * w - 4.0f * pc->k2 * (pc->k1 * fabsf(w) + k3n - p_i);
        float tau;
        if (delta > 0.0f && !feq0(delta))
            tau = (in[i].pid_output > 0.0f) ? (-w + sqrtf(delta)) / (2.0f * pc->k2)
                                            : (-w - sqrtf(delta)) / (2.0f * pc->k2);
        else
            tau = -w / (2.0f * pc->k2);      /* 重根或无实根: 取顶点近似解 */

        out[i] = clampf(tau / k0, -in[i].pid_max_out, in[i].pid_max_out);
    }
}
