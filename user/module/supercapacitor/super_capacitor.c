#include "super_capacitor.h"

#include <math.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* 电容协议帧率未知，超时暂与遥控链路一致；上报周期由实车确认后核对。 */
#define SUPER_CAPACITOR_LINK_TIMEOUT_MS 100U

static super_capacitor_data_t capacitor_data;
static uint32_t last_rx_tick;
static bool has_rx;

void super_capacitor_update_status(const uint8_t *data)
{
    float chassis_power_w;
    memcpy(&chassis_power_w, &data[1], sizeof(chassis_power_w));
    if (!isfinite(chassis_power_w))
    {
        /* 非有限值的帧丢弃，不刷新在线时间。 */
        return;
    }

    const uint32_t tick = osKernelGetTickCount();
    const UBaseType_t saved_mask = taskENTER_CRITICAL_FROM_ISR();
    capacitor_data.chassis_power_w = chassis_power_w;
    capacitor_data.cap_energy = data[7];
    capacitor_data.error = data[0] != 0U;
    last_rx_tick = tick;
    has_rx = true;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
}

void super_capacitor_get_data(super_capacitor_data_t *data)
{
    taskENTER_CRITICAL();
    const uint32_t now_tick = osKernelGetTickCount();
    *data = capacitor_data;
    data->online = has_rx && now_tick - last_rx_tick <= SUPER_CAPACITOR_LINK_TIMEOUT_MS;
    taskEXIT_CRITICAL();
}

void super_capacitor_build_command(uint8_t *data, uint16_t power_limit_w, uint16_t buffer_energy_j, bool enable_dcdc)
{
    /* 字节 0：按裁判 chassis 输出状态设置 enableDCDC、systemRestart 清零；字节 1～4 以小端 uint16 转发裁判上限和缓冲能量。 */
    data[0] = enable_dcdc ? 0x01U : 0U;
    data[1] = (uint8_t)(power_limit_w & 0xFFU);
    data[2] = (uint8_t)(power_limit_w >> 8U);
    data[3] = (uint8_t)(buffer_energy_j & 0xFFU);
    data[4] = (uint8_t)(buffer_energy_j >> 8U);
    data[5] = 0U;
    data[6] = 0U;
    data[7] = 0U;
}

void super_capacitor_build_enable_command(uint8_t *data)
{
    /* 上电使能帧：enableDCDC 与 systemRestart 均置位，其余字段为零。 */
    data[0] = 0x03U;
    data[1] = 0U;
    data[2] = 0U;
    data[3] = 0U;
    data[4] = 0U;
    data[5] = 0U;
    data[6] = 0U;
    data[7] = 0U;
}
