/* power_ctrl.h - RM2024-PowerModule 麦轮/全向轮功率控制的纯 C 移植
 *
 * 两个调用点:
 *   pc_daemon_update()  1 kHz 后台任务: 能量环 + 功率估计 + RLS 辨识 + 故障处理
 *   pc_limit()          底盘控制周期内调用: 功率环, 把 4 个轮 PID 输出衰减到功率上限内
 *
 * 电机功率模型:  P_i = τ_i ω_i + k1|ω_i| + k2 τ_i² + k3/4
 */
#ifndef POWER_CTRL_H
#define POWER_CTRL_H

#include <stdint.h>
#include "rls2.h"

#define PC_WHEEL_NUM 4

typedef enum { PC_INFANTRY = 0, PC_HERO, PC_SENTRY } pc_division_t;

/* ---- 电机底层反馈 (由你的电机驱动填充) ---- */
typedef struct {
    float   torque_nm;   /* 反馈力矩 Nm = 电流反馈(A) * KA(Nm/A) * 减速比 */
    float   av_rad_s;    /* 输出轴角速度 rad/s = rpm * PI / 30 */
    uint8_t connected;   /* 1: 在线 */
} pc_motor_fb_t;

/* ---- 外部反馈 (裁判系统 + 超级电容, 不可用的填 0 并把 *_ok 置 0) ---- */
typedef struct {
    uint8_t cap_ok;                 /* 电容通讯正常且 DCDC 使能且无错误码 */
    float   cap_energy_255;         /* 电容剩余能量 0~255 */
    float   cap_chassis_power_w;    /* 电容测得的底盘实时功率 W */

    uint8_t referee_ok;             /* 裁判系统在线 */
    float   referee_power_limit_w;  /* chassis_power_limit */
    float   referee_buffer_j;       /* buffer_energy, 满 60 J */
    float   referee_chassis_power_w;/* chassis_power */
    uint8_t robot_level;            /* 1~10 */
} pc_ext_fb_t;

/* ---- 功率环每个轮子的输入 (底盘任务每周期填) ---- */
typedef struct {
    float pid_output;    /* 轮速 PID 输出, 单位 = 电机指令单位 (如 3508 的 -16384~16384) */
    float cur_av;        /* 当前角速度 rad/s */
    float set_av;        /* 目标角速度 rad/s */
    float pid_max_out;   /* PID 输出限幅 */
} pc_wheel_in_t;

typedef struct {
    pc_division_t division;
    float k0;            /* Nm / 指令单位 = KA*减速比*电流限幅(A)/指令限幅 */
    float k1, k2, k3;    /* 模型初值, 原工程 0.22 / 1.2 / 2.78 */
    float rls_lambda;    /* 0.99999 */
    float rls_delta;     /* 1e-5 */
    uint8_t rls_enabled;
} pc_config_t;

typedef struct {
    float kp, kd, out_max, d_alpha;
    float last_err, d_filt;
    uint8_t first;
} pc_pd_t;

typedef struct {
    pc_config_t cfg;
    float k1, k2, k3;

    /* 能量环状态 */
    float referee_max_power;    /* 裁判上限 (断连时查表) */
    float power_upper_limit;    /* 用户可配置的最高上限 */
    float min_max_power;        /* 用户可配置的最低上限 = 0.8 * 裁判上限 */
    float base_max_power;       /* 能量环下限边界 (防掉血) */
    float full_max_power;       /* 能量环上限边界 (防电容过满) */
    float user_max_power;       /* 用户配置的功率上限 */
    float power_buff;           /* 当前能量反馈 */
    pc_pd_t pd_base, pd_full;

    /* 功率估计 */
    float measured_power;
    float estimated_power;
    float effective_power;
    uint8_t cap_energy_out;

    uint8_t latest_level;
    uint16_t disc_cnt[PC_WHEEL_NUM];
    rls2_t rls;

    /* 调试用 */
    float sum_cmd_power;
    float max_power_limited;
} pc_t;

void  pc_init(pc_t *pc, const pc_config_t *cfg);
void  pc_set_user_max_power(pc_t *pc, float power_w);
/* dt: 距上次调用的秒数 (1 kHz 时为 0.001f) */
void  pc_daemon_update(pc_t *pc, const pc_motor_fb_t fb[PC_WHEEL_NUM], const pc_ext_fb_t *ext, float dt);
/* out[i] 为衰减后的电机指令, 未超功率时等于 pid_output */
void  pc_limit(pc_t *pc, const pc_wheel_in_t in[PC_WHEEL_NUM], const uint8_t connected[PC_WHEEL_NUM], float out[PC_WHEEL_NUM]);

#endif
