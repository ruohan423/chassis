#include "super_capacitor.h"

#include "cmsis_os2.h"

/* 电容协议帧率未知，超时暂与遥控链路一致 */
#define SUPER_CAPACITOR_LINK_TIMEOUT_MS 100U

static super_capacitor_data_t capacitor_data;
static uint32_t last_rx_tick;
static bool has_rx;

void super_capacitor_update(uint8_t dcdc_enabled, uint8_t error_code, uint8_t energy, float chassis_power)
{
    capacitor_data.dcdc_enabled = dcdc_enabled;
    capacitor_data.error_code = error_code;
    capacitor_data.energy = energy;
    capacitor_data.chassis_power = chassis_power;
    last_rx_tick = osKernelGetTickCount();
    has_rx = true;
}

const super_capacitor_data_t *super_capacitor_get_data(void)
{
    capacitor_data.online = has_rx && (osKernelGetTickCount() - last_rx_tick) <= SUPER_CAPACITOR_LINK_TIMEOUT_MS;
    return &capacitor_data;
}
