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

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_frequency * CHASSIS_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t retry_ticks = (tick_frequency * CHASSIS_START_RETRY_MS + 999U) / 1000U;
    const uint32_t command_timeout_ticks = (tick_frequency * CHASSIS_COMMAND_TIMEOUT_MS + 999U) / 1000U;
    const float sample_period_s = (float)period_ticks / (float)tick_frequency;
    /* 爬坡控制器未接入，三个爬坡输入保持 NAN 表示未激活；上位机协议暂无 boost 字段，navigation_supercap 保持为假。 */
    chassis_command_t command = {
        .navigation_behavior = CHASSIS_BEHAVIOR_DIRECT,
        .climb_direction = NAN,
        .climb_speed = NAN,
        .climb_measure_yaw = NAN,
    };
    uint32_t last_command_tick = 0U;
    bool ready = chassis_init(sample_period_s);
#if STACK_TEST_ENABLED
    StackTest_Record(STACK_TEST_CHASSIS);
#endif
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!ready)
        {
            osDelay(retry_ticks);
            ready = chassis_init(sample_period_s);
#if STACK_TEST_ENABLED
            StackTest_Record(STACK_TEST_CHASSIS);
#endif
            next_tick = osKernelGetTickCount();
            continue;
        }

        const uint32_t now_tick = osKernelGetTickCount();
        receive_cmd_t received;
        /* 上位机数据为外部输入，非有限值的帧不采用，也不刷新链路时间。 */
        if (usb_read_command(&received) && isfinite(received.data.x) && isfinite(received.data.y) && isfinite(received.data.w))
        {
            command.navigation_vx = received.data.x;
            command.navigation_vy = received.data.y;
            command.navigation_wz = received.data.w;
            /* 当前协议只有小陀螺开关：0 为直控，1 为快速小陀螺；仅遥控器选中全自动时由控制层采用。 */
            command.navigation_behavior = received.data.chassis_spin ? CHASSIS_BEHAVIOR_SPIN_FAST : CHASSIS_BEHAVIOR_DIRECT;
            command.navigation_enabled = true;
            last_command_tick = now_tick;
        }
        if (command.navigation_enabled && now_tick - last_command_tick >= command_timeout_ticks)
        {
            /* 链路超时：导航速度和小陀螺请求清零，全自动档位下速度目标归零。 */
            command.navigation_vx = 0.0f;
            command.navigation_vy = 0.0f;
            command.navigation_wz = 0.0f;
            command.navigation_behavior = CHASSIS_BEHAVIOR_DIRECT;
            command.navigation_enabled = false;
        }

        /* 遥控器任务写入摇杆和键盘数据，这里只在调度锁内复制三个通道和按键。 */
        const int32_t previous_lock = osKernelLock();
        const int16_t stick_x = previous_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch3 - RC_STICK_OFFSET);
        const int16_t stick_y = previous_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch2 - RC_STICK_OFFSET);
        const int16_t stick_w = previous_lock < 0 ? 0 : (int16_t)(rc_data.rc.ch1 - RC_STICK_OFFSET);
        const Key_t key = previous_lock < 0 ? (Key_t){.value = 0U} : rc_data.key;
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }

        command.joystick_x = (float)stick_x * CHASSIS_STICK_X_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        command.joystick_y = (float)stick_y * CHASSIS_STICK_Y_DIRECTION / CHASSIS_STICK_FULL_SCALE;
        command.joystick_w = (float)stick_w / CHASSIS_STICK_FULL_SCALE;
        command.keyboard_x = (float)key.bits.W - (float)key.bits.S;
        command.keyboard_y = (float)key.bits.A - (float)key.bits.D;
        /* RMCS 的 boost 还可由旋钮通道触发，本工程遥控器模块未解析该通道。 */
        command.boost_request = key.bits.Shift != 0U;

        ready = chassis_update(&command, now_tick);
#if STACK_TEST_ENABLED
        StackTest_Record(STACK_TEST_CHASSIS);
#endif

        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
