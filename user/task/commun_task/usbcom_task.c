/* CubeMX 创建说明：由用户创建 USB 收发常驻线程，不在 freertos.c 中手写创建代码。
 * 名称 usbcom_Task，入口 UsbCom_Task，生成句柄 usbsend_taskHandle，优先级 osPriorityNormal。
 * 栈 1024 字节；若界面以 32 位字填写，填 256；不需要信号量，内核时钟为 1000 Hz。
 * 启用 USB_OTG_FS Device_Only 与 USB_DEVICE CDC_FS，保留 OTG_FS 中断。
 * USB 在本任务入口初始化；当前生成的 INS 弱入口被用户实现覆盖，不会执行其中的初始化。
 */
#include "usbcom_task.h"

#include "cmsis_os2.h"
#include "usb_commun.h"
#include "usb_device.h"

#define USB_SEND_PERIOD_MS       USB_SEND_MIN_INTERVAL_MS  /* 发送调度检查周期，单位 ms；USB 忙时保留待发数据。 */
#define USB_RX_FRAME_TIMEOUT_MS  50U    /* 半帧等待上限，单位 ms，超时清除拼接状态。 */

void UsbCom_Task(void *argument)
{
    (void)argument;

    MX_USB_DEVICE_Init();
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();
        usb_receive_process(now_tick, USB_RX_FRAME_TIMEOUT_MS);
        usb_send_update(now_tick);
        next_tick += USB_SEND_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
