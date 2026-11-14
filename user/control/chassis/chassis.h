#ifndef SENTRY_CHASSIS_CHASSIS_H
#define SENTRY_CHASSIS_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 底盘模式：由遥控器拨杆唯一决定，决定控制权、底盘行为和功率约束。
 * 遥控器模式只取遥控器输入并各固定一种行为；全自动只取上位机指令，两种来源不叠加。
 * s2=3 的 s1=1、s1=3 功能暂定，当前同为 CHASSIS_FREE。 */
typedef enum
{
    CHASSIS_STOP = 0,              /* s2=2 或遥控器链路异常：安全态，清零环路与输出 */
    CHASSIS_FREE,                  /* s2=1、s1=1 遥控自由：ch3=x、ch2=y、ch1=w 直接控制底盘，均在底盘坐标系；受功率上限约束 */
    CHASSIS_GIMBAL_FOLLOW_POWERED, /* s2=1、s1=2 遥控跟随云台：ch3=x、ch2=y 在云台坐标系，正前方对齐云台；不受功率上限约束 */
    CHASSIS_SPIN_FAST,             /* s2=1、s1=3 遥控小陀螺：ch3=x、ch2=y 在云台坐标系，快速小陀螺；受功率上限约束 */
    CHASSIS_AUTO,                  /* s2=3、s1=2 全自动：隔离遥控器，只采用上位机指令，行为由上位机在 chassis_behavior_t 中指定；受功率上限约束 */
} chassis_mode_t;

/* 底盘行为：平移坐标系与自转方式的组合，本身不决定输入来源和功率约束。
 * 遥控器模式各固定一种（FREE 为直控，GIMBAL_FOLLOW_POWERED 为跟随云台，SPIN_FAST 为快速小陀螺）；
 * 全自动下作为子行为由上位机指定，全部受功率上限约束。 */
typedef enum
{
    CHASSIS_BEHAVIOR_DIRECT = 0,    /* 直控：x、y、w 均在底盘坐标系，直接作为速度目标；全自动下上位机链路离线时为该行为且速度为零 */
    CHASSIS_BEHAVIOR_SPIN_SLOW,     /* 慢速小陀螺：x、y 在云台坐标系，自转为角速度上限的 CHASSIS_SPIN_SLOW_RATIO */
    CHASSIS_BEHAVIOR_SPIN_FAST,     /* 快速小陀螺：x、y 在云台坐标系，自转为角速度上限的 CHASSIS_SPIN_FAST_RATIO */
    CHASSIS_BEHAVIOR_GIMBAL_FOLLOW, /* 跟随云台：x、y 在云台坐标系，自转使底盘正前方对齐云台 */
    CHASSIS_BEHAVIOR_CLIMB,         /* 爬坡：沿底盘正向按爬坡速度平移并跟随爬坡朝向；仅全自动下爬坡输入有效时进入 */
} chassis_behavior_t;

/* 唯一写入者为遥控器任务；初始化保持自由模式，停止态只能经遥控器复位流程退出。 */
extern volatile chassis_mode_t actChassis;

/* 任务每周期提供的控制输入。无来源的字段按安全的默认值填写，不伪造数据。
 * 遥控器字段只在遥控器模式采用，上位机和爬坡字段只在 CHASSIS_AUTO 采用。 */
typedef struct
{
    float joystick_x;                       /* 摇杆 ch3 换算量，x 向，向前为正，范围 -1～1；FREE 在底盘坐标系，其余遥控器模式在云台坐标系 */
    float joystick_y;                       /* 摇杆 ch2 换算量，y 向，向左为正，范围 -1～1；坐标系同 joystick_x */
    float joystick_w;                       /* 摇杆 ch1 换算量，自转方向与 w 一致，范围 -1～1；仅 FREE 使用 */
    float keyboard_x;                       /* 键盘 W-S 平移量，与 joystick_x 叠加，范围 -1～1 */
    float keyboard_y;                       /* 键盘 A-D 平移量，与 joystick_y 叠加，范围 -1～1 */
    bool boost_request;                     /* 遥控器键盘 shift 给出的 boost 请求 */
    bool navigation_enabled;                /* 上位机控制链路在线；链路超时后由任务清零 */
    float navigation_vx;                    /* 上位机 x 向速度，向前为正，单位 m/s；直控时在底盘坐标系，其余子行为在云台坐标系 */
    float navigation_vy;                    /* 上位机 y 向速度，向左为正，单位 m/s；坐标系同 navigation_vx */
    float navigation_wz;                    /* 上位机自转角速度，单位 rad/s，逆时针为正；仅直控使用 */
    chassis_behavior_t navigation_behavior; /* 上位机指定的全自动子行为，navigation_enabled 为真时生效 */
    bool navigation_supercap;               /* 上位机 boost 请求；协议暂无该字段，当前保持为假 */
    float climb_direction;                  /* 爬坡朝向，单位 rad；爬坡控制器未接入时填 NAN 表示未激活 */
    float climb_speed;                      /* 爬坡平移速度，单位 m/s，底盘正向为正；未接入时填 NAN */
    float climb_measure_yaw;                /* 底盘实测朝向，单位 rad；未接入时填 NAN */
} chassis_command_t;

/* 给任务提供的底盘接口，控制链路移植自 RMCS-main 全向轮底盘：
 * 行为模式与速度目标 → 功率预算 → 平移/自转/轮速环 → 功率与摩擦约束下的力矩优化 → 电机指令。
 * sample_period_s 是任务实际控制周期，同时用于按周期累加的 PID 和虚拟缓冲能量积分。
 * 初始化或下发失败返回 false，由任务延时后重新初始化。 */
bool chassis_init(float sample_period_s);

/* 每个控制周期调用一次。平移输入的坐标系见 chassis_command_t，云台和底盘坐标系均为 x 向前、y 向左，单位 m/s；
 * w 逆时针为正，单位 rad/s。
 * 模式取遥控器任务写入的 actChassis，行为、输入来源和功率约束按 chassis_mode_t 的定义确定。
 * STOP、速度目标无效或任一轮电机离线时四轮零输出并复位控制器，条件恢复后自动继续。 */
bool chassis_update(const chassis_command_t *command, uint32_t now_tick);

#endif
