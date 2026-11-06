/* chassis_example.c - 演示底盘任务与功率模块的接线方式, 对应原工程 ChassisController.cpp
 * 把带 TODO 的函数换成你工程里的电机/PID/裁判/电容接口即可。
 */
#include <math.h>
#include "power_ctrl.h"

#define USE_MECANUM 1        /* 0 = 全向轮 */
#define WHEEL_DIAMETER 0.153f
#define FB_LENGTH 0.40f      /* 麦轮: 前后轮距 */
#define RL_LENGTH 0.40f      /* 麦轮: 左右轮距 */
#define WHEEL_TO_CENTRE 0.28f/* 全向: 轮到中心距离 */

/* M3508 + C620 示例参数 */
#define M3508_KA            0.3f     /* Nm/A, 含减速箱后的力矩常数 (0.0156*19.2) */
#define M3508_CURRENT_LIMIT 20.0f    /* A */
#define M3508_OUTPUT_LIMIT  16384.0f /* 指令单位 */

static pc_t g_pc;

/* ---- TODO: 换成你的底层接口 ---- */
extern float  motor_get_rpm(int i);
extern float  motor_get_current_a(int i);
extern uint8_t motor_is_connected(int i);
extern void   motor_set_output(int i, float cmd);
extern float  pid_run(int i, float set, float fb);
extern float  pid_out_max(int i);
extern void   fill_ext_feedback(pc_ext_fb_t *ext);  /* 裁判系统 + 电容 */

void chassis_power_init(void)
{
    pc_config_t cfg = {
        .division    = PC_INFANTRY,
        .k0          = M3508_KA * M3508_CURRENT_LIMIT / M3508_OUTPUT_LIMIT,
        .k1 = 0.22f, .k2 = 1.2f, .k3 = 2.78f,
        .rls_lambda  = 0.99999f,
        .rls_delta   = 1e-5f,
        .rls_enabled = 1,
    };
    pc_init(&g_pc, &cfg);
    pc_set_user_max_power(&g_pc, 60.0f);
}

/* 1 kHz 后台任务 (独立于底盘控制任务) */
void power_daemon_1khz(void)
{
    pc_motor_fb_t fb[4];
    pc_ext_fb_t ext;
    for (int i = 0; i < 4; i++) {
        fb[i].av_rad_s  = motor_get_rpm(i) * (float)M_PI / 30.0f;
        fb[i].torque_nm = motor_get_current_a(i) * M3508_KA;
        fb[i].connected = motor_is_connected(i);
    }
    fill_ext_feedback(&ext);
    pc_daemon_update(&g_pc, fb, &ext, 0.001f);
}

/* 底盘控制周期 (1 kHz): 运动学 -> PID -> 功率环 -> 输出
 * 轮序: 0=右前 1=左前 2=左后 3=右后;  X 向右, Y 向前, w 逆时针为正 */
void chassis_control_step(float vx, float vy, float vw)
{
    float target_av[4], cur_av[4];
    pc_wheel_in_t in[4];
    uint8_t connected[4];
    float out[4];

#if USE_MECANUM
    float half = (FB_LENGTH + RL_LENGTH) / 2.0f;
    target_av[0] = (-vy - vx - vw * half) * 2.0f / WHEEL_DIAMETER;
    target_av[1] = ( vy - vx - vw * half) * 2.0f / WHEEL_DIAMETER;
    target_av[2] = ( vy + vx - vw * half) * 2.0f / WHEEL_DIAMETER;
    target_av[3] = (-vy + vx - vw * half) * 2.0f / WHEEL_DIAMETER;
#else
    const float s = 0.70710678f;
    target_av[0] = (-s * vx - s * vy - vw * WHEEL_TO_CENTRE) * 2.0f / WHEEL_DIAMETER;
    target_av[1] = (-s * vx + s * vy - vw * WHEEL_TO_CENTRE) * 2.0f / WHEEL_DIAMETER;
    target_av[2] = ( s * vx + s * vy - vw * WHEEL_TO_CENTRE) * 2.0f / WHEEL_DIAMETER;
    target_av[3] = ( s * vx - s * vy - vw * WHEEL_TO_CENTRE) * 2.0f / WHEEL_DIAMETER;
#endif

    for (int i = 0; i < 4; i++) {
        cur_av[i]       = motor_get_rpm(i) * (float)M_PI / 30.0f;
        connected[i]    = motor_is_connected(i);
        in[i].cur_av    = cur_av[i];
        in[i].set_av    = target_av[i];
        in[i].pid_output  = pid_run(i, target_av[i], cur_av[i]);
        in[i].pid_max_out = pid_out_max(i);
    }

    pc_limit(&g_pc, in, connected, out);

    for (int i = 0; i < 4; i++) motor_set_output(i, out[i]);
}
