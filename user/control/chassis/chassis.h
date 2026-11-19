#ifndef SENTRY_CHASSIS_CHASSIS_H
#define SENTRY_CHASSIS_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 底盘模式：由遥控器拨杆唯一决定。s2 决定功率约束，s1 决定子模式：
 * s2=2 停止；s2=3 受功率上限约束；s2=1 不受功率上限约束，只保留电机、电调、电源和电流的硬件保护。
 * 遥控器子模式只取摇杆并各固定一种行为；全自动只取上位机指令，两种来源不叠加。 */
typedef enum
{
    CHASSIS_STOP = 0,         /* s2=2 或遥控器链路异常：安全态，清零环路与输出 */
    CHASSIS_FREE,             /* s2=3、s1=1 自由控制：ch3、ch2、ch1 控制 x、y 平移和自转，均在底盘坐标系；受功率约束 */
    CHASSIS_AUTO,             /* s2=3、s1=2 全自动：隔离遥控器，只采用上位机指令及其指定的 chassis_behavior_t；受功率约束 */
    CHASSIS_SPIN,             /* s2=3、s1=3 小陀螺：ch3、ch2 控制上电世界系下的 x、y 平移，快速小陀螺；受功率约束 */
    CHASSIS_FREE_UNLIMITED,   /* s2=1、s1=1 自由控制，输入同 CHASSIS_FREE；不受功率约束 */
    CHASSIS_FOLLOW_UNLIMITED, /* s2=1、s1=2 跟随云台：ch3、ch2 控制云台坐标系下的 x、y 平移，正前方对齐云台；不受功率约束 */
    CHASSIS_SPIN_UNLIMITED,   /* s2=1、s1=3 小陀螺，输入同 CHASSIS_SPIN；不受功率约束 */
} chassis_mode_t;

/* 底盘行为：平移坐标系与自转方式的组合，本身不决定输入来源和功率约束。
 * 遥控器子模式各固定一种（自由控制为直控，跟随云台为跟随，小陀螺为快速小陀螺）；
 * 全自动下作为子行为由上位机指定。
 * 上电世界系：姿态解算启动时刻的底盘坐标系，此后固定不动；底盘在其中的朝向即底盘 IMU 的实时 yaw
 * （chassis_feedback_t 的 chassis_yaw）。平移在上电世界系的行为每周期按该 yaw 转到底盘坐标系，
 * 目标朝向与该 yaw 同一基准；IMU 数据无效时这些行为按停止处理。
 * 枚举值即上位机控制帧 chassis_behavior 字段的行为编号：已分配的值不得改动，
 * 新增行为接在末尾并同步 usb_commun.h 的 USB_CHASSIS_BEHAVIOR_MAX。 */
typedef enum
{
    CHASSIS_BEHAVIOR_STOP = 0,          /* 停止：平移与自转目标均为零；全自动下链路离线时使用 */
    CHASSIS_BEHAVIOR_DIRECT = 1,        /* 直控：x、y 平移和自转均在底盘坐标系，直接作为速度目标 */
    CHASSIS_BEHAVIOR_TRANSLATE = 2,     /* 只平移：x、y 平移在上电世界系，自转目标为零；对应 RMCS AUTO */
    CHASSIS_BEHAVIOR_SPIN_SLOW = 3,     /* 慢速小陀螺：x、y 平移在上电世界系，自转为角速度上限的 CHASSIS_SPIN_SLOW_RATIO */
    CHASSIS_BEHAVIOR_SPIN_FAST = 4,     /* 快速小陀螺：x、y 平移在上电世界系，自转为角速度上限的 CHASSIS_SPIN_FAST_RATIO */
    CHASSIS_BEHAVIOR_ALIGNMENT = 5,     /* 四面对齐：x、y 平移在上电世界系，自转使底盘 x、y 轴中最近的一条对齐平移方向；对应 RMCS ALIGNMENT、ALIGNMENT_POWERED */
    CHASSIS_BEHAVIOR_STEP_DOWN = 6,     /* 两面对齐：x、y 平移在上电世界系，自转使底盘正前方或正后方中最近的一面对齐目标朝向；对应 RMCS STEP_DOWN */
    CHASSIS_BEHAVIOR_LAUNCH_RAMP = 7,   /* 单面对齐：x、y 平移在上电世界系，自转使底盘正前方对齐目标朝向；对应 RMCS LAUNCH_RAMP */
    CHASSIS_BEHAVIOR_GIMBAL_FOLLOW = 8, /* 跟随云台：x、y 平移在云台坐标系，自转使底盘正前方对齐云台；唯一使用云台坐标系的行为 */
    CHASSIS_BEHAVIOR_CLIMB = 9,         /* 爬坡：沿底盘正向按爬坡速度平移，自转使底盘正前方对齐爬坡朝向；仅全自动下爬坡输入有效时进入 */
} chassis_behavior_t;

/* 唯一写入者为遥控器任务；初始为受功率约束的自由控制，停止态只能经遥控器复位流程退出。 */
extern volatile chassis_mode_t actChassis;

/* 任务每周期提供的控制输入。无来源的字段按安全的默认值填写，不伪造数据。
 * stick_* 只在遥控器子模式采用，nav_*、boost 和 climb_* 只在 CHASSIS_AUTO 采用。 */
typedef struct
{
    float stick_x;                   /* 摇杆 ch3 换算量，x 向，向前为正，范围 -1～1；自由控制在底盘坐标系，小陀螺在上电世界系，跟随云台在云台坐标系 */
    float stick_y;                   /* 摇杆 ch2 换算量，y 向，向左为正，范围 -1～1；坐标系同 stick_x */
    float stick_wz;                  /* 摇杆 ch1 换算量，自转，逆时针为正，范围 -1～1；仅自由控制使用 */
    bool nav_online;                 /* 上位机控制链路在线；链路超时后由任务清零 */
    float nav_vx;                    /* 上位机 x 向速度，向前为正，单位 m/s；直控时在底盘坐标系，跟随云台时在云台坐标系，其余子行为在上电世界系 */
    float nav_vy;                    /* 上位机 y 向速度，向左为正，单位 m/s；坐标系同 nav_vx */
    float nav_wz;                    /* 上位机自转角速度，逆时针为正，单位 rad/s；仅直控使用 */
    float nav_target_yaw;            /* 上位机目标朝向，上电世界系，与底盘 IMU 的 yaw 同一基准，单位 rad；仅两面对齐、单面对齐使用 */
    chassis_behavior_t nav_behavior; /* 上位机指定的全自动子行为，nav_online 为真时生效 */
    bool boost;                      /* 全自动下的 boost 请求，仅上位机链路在线时生效；协议暂无该字段，当前保持为假 */
    float climb_direction;           /* 爬坡朝向，上电世界系，与底盘 IMU 的 yaw 同一基准，单位 rad；爬坡控制器未接入时填 NAN 表示未激活 */
    float climb_speed;               /* 爬坡平移速度，单位 m/s，底盘正向为正；未接入时填 NAN */
} chassis_command_t;

/* 给任务提供的底盘接口，控制链路移植自 RMCS-main 全向轮底盘：
 * 模式与速度目标 → 功率预算 → 平移/自转/轮速环 → 功率与摩擦约束下的力矩优化 → 硬件保护 → 电机指令。
 * period_s 是任务实际控制周期，单位 s，同时用于按周期累加的 PID 和虚拟缓冲能量积分。
 * 初始化或下发失败返回 false，由任务延时后重新初始化。 */
bool chassis_init(float period_s);

/* 每个控制周期调用一次。平移输入的坐标系见 chassis_command_t，底盘坐标系、云台坐标系和上电世界系均为 x 向前、y 向左，单位 m/s；
 * 自转角速度逆时针为正，单位 rad/s。底盘实测朝向不由 command 提供，统一取 port 层反馈的底盘 IMU yaw。
 * 模式取遥控器任务写入的 actChassis，行为、输入来源和功率约束按 chassis_mode_t 的定义确定。
 * command 仅在调用期间只读借用，不保存指针。STOP、速度目标无效或任一轮电机离线时四轮零输出并复位控制器，条件恢复后自动继续。 */
bool chassis_update(const chassis_command_t *command, uint32_t tick);

#endif
