/* CubeMX 创建说明：保留或由用户创建遥控器常驻线程及二值信号量。
 * 线程名称 remote_task，入口 Remote_Task，优先级 osPriorityLow，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128。
 * 二值信号量名称 RemoteSem，最大值 1、初值 0，句柄为 RemoteSemHandle。
 * USART3 RX DMA 使用 Circular，USART/DMA 中断优先级为 5；收到有效输入前保持安全默认。
 */
#include "remote_task.h"
#include "cmsis_os2.h"

#include "chassis.h"
#include "remote.h"
#include "user_uart.h"
#include "usart.h"
#include "watch_task.h"

#include <string.h>

#include "main.h"

#define REMOTE_SEMAPHORE_WAIT_MS 10U
#define REMOTE_LINK_TIMEOUT_MS 100U

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
static uint32_t last_valid_frame_tick;
static bool has_valid_frame;

static void process_received_data(const uint8_t *data, uint16_t data_length, bool dma_overrun);
static void RemoteCallback(Remote *rc);
static void MouseKeyCallback(Mouse *mouse, Key_t key, Key_t lastkey);
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

        if (!uart_started || remote_uart_rx.error_pending)
        {
            remote_clear();
            actChassis = CHASSIS_STOP;
            frame_length = 0U;
            has_valid_frame = false;
            uart_started = uart_rx_start(&remote_uart_rx);
            continue;
        }

        bool dma_overrun;
        const uint16_t received_length = uart_rx_read(&remote_uart_rx, received_data, &dma_overrun);
        process_received_data(received_data, received_length, dma_overrun);

        if (has_valid_frame && (osKernelGetTickCount() - last_valid_frame_tick >= REMOTE_LINK_TIMEOUT_MS))
        {
            /* 链路超时：清零输入并切换底盘到安全行为 */
            remote_clear();
            actChassis = CHASSIS_STOP;
            frame_length = 0U;
            has_valid_frame = false;
        }
    }
}

static void process_received_data(const uint8_t *data, uint16_t data_length, bool dma_overrun)
{
    if (dma_overrun){frame_length = 0U;}

    for (uint16_t index = 0U; index < data_length; index++)
    {
        frame[frame_length++] = data[index];

        if (frame_length < RC_FRAME_LENGTH){continue;}

        if (remote_frame_is_valid(frame))
        {
            last_valid_frame_tick = osKernelGetTickCount();
            has_valid_frame = true;
            swdg_feed_remote();
            remote_rx(frame);
            frame_length = 0U;
        }
        else
        {
            memmove(frame, &frame[1], RC_FRAME_LENGTH - 1U);
            frame_length = RC_FRAME_LENGTH - 1U;
        }
    }
}


static void RemoteCallback(Remote *rc)
{
    if (actChassis == CHASSIS_STOP){

        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
    }

    if (rc->s1 == 1 && actChassis != CHASSIS_NORMAL) {
        actChassis = CHASSIS_NORMAL;
    }  //底盘不跟随云台
    else if (rc->s1 == 3 && actChassis != CHASSIS_FOLLOW_GIMBAL) {
        actChassis = CHASSIS_FOLLOW_GIMBAL;
    }   //底盘跟随云台
    else if (rc->s1 == 2 && actChassis != CHASSIS_GYROSCOPE) {
        actChassis = CHASSIS_GYROSCOPE;
    } //小陀螺模式
}


static void MouseKeyCallback(Mouse *mouse, Key_t key, Key_t lastkey)
{
    (void)mouse;
    (void)key;
    (void)lastkey;
}


static void StopCallback(void)
{
    actChassis = CHASSIS_STOP;
}
