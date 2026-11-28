/* CubeMX 创建说明：保留或由用户创建遥控器常驻线程及二值信号量。
 * 线程名称 remote_task，入口 Remote_Task，优先级 osPriorityLow，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128。
 * 二值信号量名称 RemoteSem，最大值 1、初值 0，句柄为 RemoteSemHandle。
 * USART3 按 DBUS 配置：100000 bps、Word Length 9 Bits（含校验位，即 8 位数据）、Parity Even、1 停止位。
 * USART3 RX DMA 使用 Circular，USART/DMA 中断优先级为 5；收到有效输入前保持安全默认。
 */
#include "remote_task.h"
#include "cmsis_os2.h"

#include "chassis.h"
#include "cancom_task.h"
#include "remote.h"
#include "user_uart.h"
#include "usart.h"
#include "watchdog_control.h"

#include <string.h>

#include "main.h"

#define REMOTE_SEMAPHORE_WAIT_MS 10U
/* DBUS 一帧 18 字节约 2 ms、帧周期 14 ms，残帧超时取在帧内最长间隔与帧间空闲之间；未上板标定。 */
#define REMOTE_FRAME_TIMEOUT_MS 6U

#define REMOTE_DMA_BUFFER_SIZE 128U

static uint8_t dma_buffer[REMOTE_DMA_BUFFER_SIZE];
uart_rx_t remote_uart_rx = {
    .handle = &huart3,
    .buffer = dma_buffer,
    .size = REMOTE_DMA_BUFFER_SIZE,
};

static uint8_t frame[RC_FRAME_LENGTH];
static uint8_t frame_length;
static uint8_t received_data[REMOTE_DMA_BUFFER_SIZE];
static uint32_t last_byte_tick;
static bool has_valid_frame;
static bool stop_latched;  /* 仅 s2=2 急停置位；链路超时进入的停止态不锁存，恢复后按拨杆直接回到对应模式。 */

static bool process_received_data(const uint8_t *data, uint16_t data_length);
static void RemoteCallback(const Remote *rc);
static void MouseKeyCallback(const Mouse *mouse, Key_t key, Key_t lastkey);
static void StopCallback(void);

void Remote_Task(void *argument)
{
    (void)argument;

    remote_clear();
    Remote_RegisterCallbacks(RemoteCallback, MouseKeyCallback, StopCallback);
    bool uart_started = uart_rx_start(&remote_uart_rx);
    frame_length = 0U;
    has_valid_frame = false;

    for (;;)
    {
        (void)osSemaphoreAcquire(RemoteSemHandle, REMOTE_SEMAPHORE_WAIT_MS);
        const uint32_t now_tick = osKernelGetTickCount();

        if (!uart_started || remote_uart_rx.error_pending)
        {
            /* 线路错误或启动失败只丢弃残帧并重启接收，遥控数据和模式保持；持续收不到有效帧时由下方链路超时进入停止态。 */
            frame_length = 0U;
            uart_started = uart_rx_start(&remote_uart_rx);
        }
        else
        {
            bool dma_overrun;
            const uint16_t received_length = uart_rx_read(&remote_uart_rx, received_data, &dma_overrun);
            /* 残帧只复位拼帧：DBUS 无同步字，靠帧间空闲对齐，避免旧残帧与新帧拼出通过范围校验的错位帧。 */
            if (dma_overrun || (frame_length != 0U && now_tick - last_byte_tick >= REMOTE_FRAME_TIMEOUT_MS))
            {
                frame_length = 0U;
            }
            if (received_length != 0U)
            {
                last_byte_tick = now_tick;
            }
            if (process_received_data(received_data, received_length))
            {
                watchdog_control_feed(WATCH_REMOTE);
            }
        }

        /* 遥控器关机、断线或持续线路错误：回到零输出停止态但不锁存，收到有效帧后按拨杆恢复，不复位系统。 */
        if (has_valid_frame && watchdog_control_is_expired(WATCH_REMOTE))
        {
            remote_clear();
            actChassis = CHASSIS_STOP;
            frame_length = 0U;
            has_valid_frame = false;
        }
    }
}

static bool process_received_data(const uint8_t *data, uint16_t data_length)
{
    bool valid_frame_received = false;
    for (uint16_t index = 0U; index < data_length; index++)
    {
        frame[frame_length++] = data[index];

        if (frame_length < RC_FRAME_LENGTH){continue;}

        if (remote_frame_is_valid(frame))
        {
            has_valid_frame = true;
            valid_frame_received = true;
            remote_rx(frame);
            CanCom_TaskNotifyRemoteFrame();
            frame_length = 0U;
        }
        else
        {
            memmove(frame, &frame[1], RC_FRAME_LENGTH - 1U);
            frame_length = RC_FRAME_LENGTH - 1U;
        }
    }
    return valid_frame_received;
}


static void RemoteCallback(const Remote *rc)
{
    /* 只有 s2=2 急停锁存后拨回遥控输入才复位；链路超时后的重连不走复位流程。 */
    if (stop_latched){

        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
    }

    if (rc->s2 != 1) {return;}

    /* s2=1 的子模式均不受功率约束：s1=1 自由控制，s1=2 跟随云台，s1=3 小陀螺。 */
    switch (rc->s1)
    {
    case 1:
        actChassis = CHASSIS_FREE_UNLIMITED;
        break;
    case 2:
        actChassis = CHASSIS_FOLLOW_UNLIMITED;
        break;
    case 3:
        actChassis = CHASSIS_SPIN_UNLIMITED;
        break;
    default:
        break;
    }
}


static void MouseKeyCallback(const Mouse *mouse, Key_t key, Key_t lastkey)
{
    (void)mouse;
    (void)key;
    (void)lastkey;

    /* 急停锁存只能由遥控输入模式的复位流程退出。 */
    if (stop_latched){return;}

    /* 本回调只在 s2=3 时调用，子模式均受功率约束：s1=1 自由控制，s1=2 全自动（只取上位机指令），s1=3 小陀螺。 */
    switch (rc_data.rc.s1)
    {
    case 1:
        actChassis = CHASSIS_FREE;
        break;
    case 2:
        actChassis = CHASSIS_AUTO;
        break;
    case 3:
        actChassis = CHASSIS_SPIN;
        break;
    default:
        break;
    }
}


static void StopCallback(void)
{
    stop_latched = true;
    actChassis = CHASSIS_STOP;
}
