/* CubeMX 创建说明：由用户创建常驻线程，不在 freertos.c 用户区手写创建代码。
 * 名称 chassis_task，入口 Chassis_Task，优先级 osPriorityAboveNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；入口应绑定本文件实现，避免重复强定义。
 * 无独立信号量；本任务内初始化底盘并完成功率预算，不再需要 power_task 线程。
 */
#include "chassis_task.h"
#include "cmsis_os2.h"

#include <math.h>

#include "chassis.h"
#include "remote.h"
#include "uart_commun.h"

#define CHASSIS_TASK_PERIOD_MS     2U    /* 底盘控制周期，单位 ms；PID 按该周期累加，修改后须重新换算 ki、kd，并同时用于功率预算积分。 */
#define CHASSIS_START_RETRY_MS     100U  /* 总线初始化或发送失败后的重试间隔，单位 ms。 */
#define CHASSIS_COMMAND_TIMEOUT_MS 200U  /* 上位机控制帧链路超时，单位 ms；超时后导航速度、目标朝向和行为请求复位，全自动档位下速度目标归零。 */

_Static_assert(UART_CHASSIS_BEHAVIOR_MAX == CHASSIS_BEHAVIOR_CLIMB, "UART 控制帧行为编号上限须等于 chassis_behavior_t 的最后一项");

void Chassis_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_freq = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_freq * CHASSIS_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t retry_ticks = (tick_freq * CHASSIS_START_RETRY_MS + 999U) / 1000U;
    const uint32_t command_timeout_ticks = (tick_freq * CHASSIS_COMMAND_TIMEOUT_MS + 999U) / 1000U;
    const float period_s = (float)period_ticks / (float)tick_freq;
    /* 爬坡控制器未接入，两个爬坡输入保持 NAN 表示未激活。 */
    static chassis_command_t command = {
        .nav_behavior = CHASSIS_BEHAVIOR_DIRECT,
        .climb_direction = NAN,
        .climb_speed = NAN,
    };
    static uart_receive_cmd_t host_command;
    uint32_t last_command_tick = 0U;
    bool chassis_ready = chassis_init(period_s);
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!chassis_ready)
        {
            osDelay(retry_ticks);
            chassis_ready = chassis_init(period_s);
            next_tick = osKernelGetTickCount();
            continue;
        }

        const uint32_t tick = osKernelGetTickCount();
        /* 上位机数据为外部输入，非有限值的帧不采用，也不刷新链路时间。 */
        if (uart_read_command(&host_command) && isfinite(host_command.data.x) && isfinite(host_command.data.y)
            && isfinite(host_command.data.w) && isfinite(host_command.data.target_yaw))
        {
            command.nav_vx = host_command.data.x;
            command.nav_vy = host_command.data.y;
            command.nav_wz = host_command.data.w;
            command.nav_target_yaw = host_command.data.target_yaw;
            /* 行为编号已由 UART 接收校验，取值与 chassis_behavior_t 一一对应；仅遥控器选中全自动时由控制层采用。 */
            command.nav_behavior = (chassis_behavior_t)host_command.data.chassis_behavior;
            command.nav_online = true;
            last_command_tick = tick;
        }
        if (command.nav_online && tick - last_command_tick >= command_timeout_ticks)
        {
            /* 链路超时：导航速度和目标朝向清零，行为请求回到直控，全自动档位下速度目标归零。 */
            command.nav_vx = 0.0f;
            command.nav_vy = 0.0f;
            command.nav_wz = 0.0f;
            command.nav_target_yaw = 0.0f;
            command.nav_behavior = CHASSIS_BEHAVIOR_DIRECT;
            command.nav_online = false;
        }

        /* 遥控器任务写入摇杆数据，这里只在调度锁内复制三个通道；未获得保护时按摇杆回中处理。 */
        int16_t stick_x = 0;
        int16_t stick_y = 0;
        int16_t stick_wz = 0;
        const int32_t lock_state = osKernelLock();
        if (lock_state >= 0)
        {
            stick_x = (int16_t)(rc_data.rc.ch3 - RC_STICK_OFFSET);
            stick_y = (int16_t)(rc_data.rc.ch2 - RC_STICK_OFFSET);
            stick_wz = (int16_t)(rc_data.rc.ch1 - RC_STICK_OFFSET);
        }
        if (lock_state == 0)
        {
            (void)osKernelRestoreLock(lock_state);
        }
        command.stick_x = (float)stick_x * CHASSIS_STICK_X_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        command.stick_y = (float)stick_y * CHASSIS_STICK_Y_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        command.stick_wz = (float)stick_wz / CHASSIS_STICK_FULL_SCALE;

        chassis_ready = chassis_update(&command, tick);

        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
