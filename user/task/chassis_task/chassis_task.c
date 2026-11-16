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
#include "usb_commun.h"

#define CHASSIS_TASK_PERIOD_MS     2U    /* 底盘控制周期，单位 ms；PID 按该周期累加，修改后须重新换算 ki、kd，并同时用于功率预算积分。 */
#define CHASSIS_START_RETRY_MS     100U  /* 总线初始化或发送失败后的重试间隔，单位 ms。 */
#define CHASSIS_COMMAND_TIMEOUT_MS 200U  /* 上位机控制帧链路超时，单位 ms；超时后导航速度和小陀螺请求清零，全自动档位下速度目标归零。 */

void Chassis_Task(void *argument)
{
    (void)argument;

    const uint32_t kernel_tick_frequency = osKernelGetTickFreq();
    const uint32_t control_period_ticks = (kernel_tick_frequency * CHASSIS_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t init_retry_ticks = (kernel_tick_frequency * CHASSIS_START_RETRY_MS + 999U) / 1000U;
    const uint32_t navigation_timeout_ticks =
        (kernel_tick_frequency * CHASSIS_COMMAND_TIMEOUT_MS + 999U) / 1000U;
    const float control_period_seconds = (float)control_period_ticks / (float)kernel_tick_frequency;
    /* 爬坡控制器未接入，三个爬坡输入保持 NAN 表示未激活；上位机协议暂无 boost 字段，boost_request 保持为假。 */
    static chassis_command_t chassis_command = {
        .navigation_behavior = CHASSIS_BEHAVIOR_DIRECT,
        .climb_direction = NAN,
        .climb_speed = NAN,
        .climb_measure_yaw = NAN,
    };
    static receive_cmd_t received_command;
    uint32_t last_navigation_command_tick = 0U;
    bool chassis_ready = chassis_init(control_period_seconds);
    uint32_t next_control_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!chassis_ready)
        {
            osDelay(init_retry_ticks);
            chassis_ready = chassis_init(control_period_seconds);
            next_control_tick = osKernelGetTickCount();
            continue;
        }

        const uint32_t current_tick = osKernelGetTickCount();
        /* 上位机数据为外部输入，非有限值的帧不采用，也不刷新链路时间。 */
        if (usb_read_command(&received_command) && isfinite(received_command.data.x)
            && isfinite(received_command.data.y) && isfinite(received_command.data.w))
        {
            chassis_command.navigation_velocity_x = received_command.data.x;
            chassis_command.navigation_velocity_y = received_command.data.y;
            chassis_command.navigation_angular_velocity = received_command.data.w;
            /* 当前协议只有小陀螺开关：0 为直控，1 为快速小陀螺；仅遥控器选中全自动时由控制层采用。 */
            chassis_command.navigation_behavior = received_command.data.chassis_spin
                ? CHASSIS_BEHAVIOR_SPIN_FAST : CHASSIS_BEHAVIOR_DIRECT;
            chassis_command.navigation_enabled = true;
            last_navigation_command_tick = current_tick;
        }
        if (chassis_command.navigation_enabled
            && current_tick - last_navigation_command_tick >= navigation_timeout_ticks)
        {
            /* 链路超时：导航速度和小陀螺请求清零，全自动档位下速度目标归零。 */
            chassis_command.navigation_velocity_x = 0.0f;
            chassis_command.navigation_velocity_y = 0.0f;
            chassis_command.navigation_angular_velocity = 0.0f;
            chassis_command.navigation_behavior = CHASSIS_BEHAVIOR_DIRECT;
            chassis_command.boost_request = false;
            chassis_command.navigation_enabled = false;
        }

        /* 遥控器任务写入摇杆数据，这里只在调度锁内复制三个通道。 */
        const int32_t previous_kernel_lock = osKernelLock();
        const int16_t joystick_x_raw = previous_kernel_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch3 - RC_STICK_OFFSET);
        const int16_t joystick_y_raw = previous_kernel_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch2 - RC_STICK_OFFSET);
        const int16_t joystick_angular_raw = previous_kernel_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch1 - RC_STICK_OFFSET);
        if (previous_kernel_lock == 0)
        {
            (void)osKernelRestoreLock(previous_kernel_lock);
        }

        chassis_command.joystick_x = (float)joystick_x_raw * CHASSIS_STICK_X_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        chassis_command.joystick_y = (float)joystick_y_raw * CHASSIS_STICK_Y_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        chassis_command.joystick_angular = (float)joystick_angular_raw / CHASSIS_STICK_FULL_SCALE;

        chassis_ready = chassis_update(&chassis_command, current_tick);

        next_control_tick += control_period_ticks;
        osDelayUntil(next_control_tick);
    }
}
