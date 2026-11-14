#include "super_capacitor.h"

#include <math.h>

#include "cmsis_os2.h"

/* 电容协议帧率未知，超时暂与遥控链路一致 */
#define SUPER_CAPACITOR_LINK_TIMEOUT_MS 100U

/* 状态帧 16 位原始量到物理量的线性映射范围，取自 RMCS Supercap。 */
#define SUPER_CAPACITOR_POWER_MIN_W   (-100.0f)
#define SUPER_CAPACITOR_POWER_MAX_W   400.0f
#define SUPER_CAPACITOR_VOLTAGE_MAX_V 50.0f

static super_capacitor_data_t capacitor_data;
static float command_power_limit_w;
static bool command_enabled;
static uint32_t last_rx_tick;
static bool has_rx;

static float decode_uint16(const uint8_t *data, float min_value, float max_value);

void super_capacitor_update_status(const uint8_t data[SUPER_CAPACITOR_STATUS_LENGTH])
{
    capacitor_data.chassis_power_w = decode_uint16(&data[0], SUPER_CAPACITOR_POWER_MIN_W, SUPER_CAPACITOR_POWER_MAX_W);
    capacitor_data.supercap_voltage_v = decode_uint16(&data[2], 0.0f, SUPER_CAPACITOR_VOLTAGE_MAX_V);
    capacitor_data.chassis_voltage_v = decode_uint16(&data[4], 0.0f, SUPER_CAPACITOR_VOLTAGE_MAX_V);
    capacitor_data.enabled = data[6] != 0U;
    last_rx_tick = osKernelGetTickCount();
    has_rx = true;
}

const super_capacitor_data_t *super_capacitor_get_data(void)
{
    capacitor_data.online = has_rx && (osKernelGetTickCount() - last_rx_tick) <= SUPER_CAPACITOR_LINK_TIMEOUT_MS;
    return &capacitor_data;
}

void super_capacitor_set_command(float charge_power_limit_w, bool enabled)
{
    command_power_limit_w = charge_power_limit_w;
    command_enabled = enabled;
}

void super_capacitor_build_command(uint8_t data[SUPER_CAPACITOR_COMMAND_LENGTH])
{
    /* 协议以 1 W 为单位的 8 位无符号数表示上限。 */
    data[0] = (uint8_t)fminf(fmaxf(command_power_limit_w, 0.0f), 255.0f);
    data[1] = command_enabled ? 1U : 0U;
}

/* 小端 16 位原始量按满量程线性映射到 [min_value, max_value]。 */
static float decode_uint16(const uint8_t *data, float min_value, float max_value)
{
    return (float)(uint16_t)(data[0] | (data[1] << 8)) / 65535.0f * (max_value - min_value) + min_value;
}
