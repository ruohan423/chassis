#ifndef SENTRY_CHASSIS_UART_COMMUN_H
#define SENTRY_CHASSIS_UART_COMMUN_H

#include <stdbool.h>
#include <stdint.h>

#include "judge.h"
#include "user_uart.h"

#define UART_SEND_MIN_INTERVAL_MS 2U
#define UART_STATE_SEND_INTERVAL_MS 10U
#define UART_CHASSIS_BEHAVIOR_MAX 9U

/* UART1 上位机控制帧，44 字节，小端；字段单位与底盘控制接口一致。 */
typedef struct
{
    uint32_t frame_header;
    uint32_t frame_timestamp;
    struct
    {
        float x;
        float y;
        float w;
        float target_yaw;
        uint8_t chassis_behavior;
        bool pitch_cmd;
        uint8_t gimabal_model;
        float gimabal_tar_angle;
        float gimabal_pat_angle;
        bool activate_energy;
    } data;
    uint8_t eof;
} uart_receive_cmd_t;

/* UART1 哨兵指令帧，16 字节，载荷遵循裁判协议 0x0120。 */
typedef struct
{
    uint32_t frame_header;
    uint32_t frame_timestamp;
    sentry_cmd_t data;
    uint8_t eof;
} uart_receive_sentry_t;

/* UART1 底盘状态帧，24 字节；yaw 单位 rad，wz 单位 rad/s。 */
typedef struct
{
    uint32_t frame_header;
    uint32_t frame_timestamp;
    struct
    {
        float chassis_yaw;
        float chassis_wz;
        bool pitch_status;
        bool energy_mechanis_done;
    } data;
    uint8_t eof;
} uart_send_cmd_t;

extern uart_rx_t host_uart_rx;

void uart_commun_update_imu(float yaw_rad, float yaw_rate_rad_s);
void uart_commun_reset(void);
void uart_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks);
/* command 为调用方输出对象；仅在调用期间写入，不会保存该指针。 */
bool uart_read_command(uart_receive_cmd_t *command);
void uart_send_update(uint32_t now_tick);

#endif
