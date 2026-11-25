/* CubeMX 创建说明：仅上板联调上位机串口协议时由用户创建，不用于常规运行。
 * 名称 uart_test_task，入口 UartTest_Task，优先级 osPriorityLow，栈 512 字节；CMSIS-RTOS2 stack_size 单位为字节，
 * 界面以 32 位字填写时填 128。无独立信号量；当前周期要求内核时钟为 1000 Hz。
 * 本任务独占 USART1 TX（Normal DMA），按 uart_commun.c 的发送帧格式直接发出模拟数据，不经过 uart_send_update()；
 * 启用时必须停用 uartcom_task（或把该线程入口改为 UartTest_Task），两者不得同时发送。
 * 本任务不启动 USART1 接收，测试期间上位机下行帧不被处理。
 */
#include "uart_test_task.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"
#include "judge.h"
#include "uart_commun.h"
#include "usart.h"

#define UART_TEST_PERIOD_MS        UART_SEND_MIN_INTERVAL_MS      /* 任务周期，同时是相邻两帧的最小起发间隔，单位 ms。 */
#define UART_TEST_STATE_PERIOD_MS  UART_STATE_SEND_INTERVAL_MS    /* 底盘状态帧发送周期，单位 ms，与正式发送一致为 100 Hz。 */
#define UART_TEST_TX_TIMEOUT_MS    50U                            /* 单帧 DMA 发送超时，单位 ms；超时后中止本帧，与 uart_commun.c 一致。 */
#define UART_TEST_FRAME_COUNT      3U                             /* 每种数据包循环发送的模拟帧数。 */
#define UART_TEST_JUDGE_COUNT      16U                            /* 转发的裁判数据包种类数，与 uart_commun.c 的转发表一致。 */
#define UART_TEST_STATE_INDEX      UART_TEST_JUDGE_COUNT          /* 状态帧在发送时刻表和帧序号表中的下标。 */
#define UART_TEST_TX_BUFFER_SIZE   (8U + sizeof(ground_robot_position_t) + 1U)  /* 最长帧：4 字节帧头、4 字节时间戳、40 字节载荷和 1 字节帧尾。 */

typedef struct
{
    uint16_t command_id;
    uint16_t period_ms;
    uint8_t length;
    const uint8_t *frames;      /* UART_TEST_FRAME_COUNT 帧载荷连续存放，每帧 length 字节。 */
} test_packet_t;

_Static_assert(UART_TEST_TX_BUFFER_SIZE >= sizeof(uart_send_cmd_t), "发送缓冲区必须容纳状态帧");

/* 0x1001 底盘状态帧；frame_timestamp 在发送时填入当前 tick。 */
static const uart_send_cmd_t state_frames[UART_TEST_FRAME_COUNT] = {
    {.frame_header = 0x1001U, .data = {.chassis_yaw = 0.0f, .chassis_wz = 0.0f, .pitch_status = false, .energy_mechanis_done = false}, .eof = 0xA5U},
    {.frame_header = 0x1001U, .data = {.chassis_yaw = 1.5708f, .chassis_wz = 3.1416f, .pitch_status = true, .energy_mechanis_done = false}, .eof = 0xA5U},
    {.frame_header = 0x1001U, .data = {.chassis_yaw = -1.5708f, .chassis_wz = -3.1416f, .pitch_status = false, .energy_mechanis_done = true}, .eof = 0xA5U},
};

static const game_status_t game_status_frames[UART_TEST_FRAME_COUNT] = {
    {.game_type = 1U, .game_progress = 1U, .stage_remain_time = 180U, .sync_timestamp = 1795000000ULL},
    {.game_type = 1U, .game_progress = 4U, .stage_remain_time = 420U, .sync_timestamp = 1795000001ULL},
    {.game_type = 4U, .game_progress = 5U, .stage_remain_time = 0U, .sync_timestamp = 1795000002ULL},
};

static const game_result_t game_result_frames[UART_TEST_FRAME_COUNT] = {{0U}, {1U}, {2U}};

static const game_robot_hp_t robot_hp_frames[UART_TEST_FRAME_COUNT] = {
    {.ally_1_robot_hp = 500U, .ally_3_robot_hp = 400U, .ally_4_robot_hp = 400U, .damage_difference = 0, .ally_7_robot_hp = 400U,
     .outpost_r_hp = 1500U, .ally_base_hp = 5000U, .outpost_b_hp = 1500U, .enemy_base_hp = 5000U, .ally_control_degree = 0U, .enemy_control_degree = 0U},
    {.ally_1_robot_hp = 350U, .ally_3_robot_hp = 200U, .ally_4_robot_hp = 0U, .damage_difference = 300, .ally_7_robot_hp = 380U,
     .outpost_r_hp = 1000U, .ally_base_hp = 5000U, .outpost_b_hp = 600U, .enemy_base_hp = 4500U, .ally_control_degree = 120U, .enemy_control_degree = 80U},
    {.ally_1_robot_hp = 0U, .ally_3_robot_hp = 150U, .ally_4_robot_hp = 300U, .damage_difference = -450, .ally_7_robot_hp = 120U,
     .outpost_r_hp = 0U, .ally_base_hp = 3200U, .outpost_b_hp = 1500U, .enemy_base_hp = 5000U, .ally_control_degree = 200U, .enemy_control_degree = 650U},
};

static const event_data_t event_frames[UART_TEST_FRAME_COUNT] = {
    {0x00000000UL, 0x00000000UL},
    {0x12345678UL, 0x000000A5UL},
    {0x89ABCDEFUL, 0x5A5A5A5AUL},
};

static const referee_warning_t warning_frames[UART_TEST_FRAME_COUNT] = {
    {.level = 1U, .offending_robot_id = 0U, .count = 1U},
    {.level = 2U, .offending_robot_id = 7U, .count = 2U},
    {.level = 3U, .offending_robot_id = 3U, .count = 1U},
};

static const dart_info_t dart_info_frames[UART_TEST_FRAME_COUNT] = {
    {.dart_remaining_time = 0U, .dart_info = 0x0000U},
    {.dart_remaining_time = 15U, .dart_info = 0x0125U},
    {.dart_remaining_time = 3U, .dart_info = 0x03AAU},
};

static const robot_status_t robot_status_frames[UART_TEST_FRAME_COUNT] = {
    {.robot_id = 7U, .robot_level = 1U, .current_hp = 400U, .maximum_hp = 400U, .shooter_barrel_cooling_value = 80U, .shooter_barrel_heat_limit = 260U,
     .chassis_power_limit = 100U, .bullet_speed_limit = 25.0f, .power_management_gimbal_output = 1U, .power_management_chassis_output = 1U, .power_management_shooter_output = 1U},
    {.robot_id = 7U, .robot_level = 1U, .current_hp = 250U, .maximum_hp = 400U, .shooter_barrel_cooling_value = 80U, .shooter_barrel_heat_limit = 260U,
     .chassis_power_limit = 100U, .bullet_speed_limit = 25.0f, .power_management_gimbal_output = 1U, .power_management_chassis_output = 1U, .power_management_shooter_output = 0U},
    {.robot_id = 107U, .robot_level = 1U, .current_hp = 0U, .maximum_hp = 400U, .shooter_barrel_cooling_value = 40U, .shooter_barrel_heat_limit = 130U,
     .chassis_power_limit = 60U, .bullet_speed_limit = 22.5f, .power_management_gimbal_output = 0U, .power_management_chassis_output = 0U, .power_management_shooter_output = 0U},
};

static const power_heat_data_t power_heat_frames[UART_TEST_FRAME_COUNT] = {
    {.buffer_energy = 60U, .shooter_17mm_barrel_heat = 0U, .shooter_42mm_barrel_heat = 0U},
    {.buffer_energy = 35U, .shooter_17mm_barrel_heat = 120U, .shooter_42mm_barrel_heat = 0U},
    {.buffer_energy = 5U, .shooter_17mm_barrel_heat = 255U, .shooter_42mm_barrel_heat = 100U},
};

static const robot_pos_t robot_pos_frames[UART_TEST_FRAME_COUNT] = {
    {.x = 1.5f, .y = 2.5f, .angle = 0.0f},
    {.x = 6.25f, .y = 7.75f, .angle = 90.0f},
    {.x = 13.5f, .y = 4.125f, .angle = 270.5f},
};

static const buff_t buff_frames[UART_TEST_FRAME_COUNT] = {
    {.recovery_buff = 0U, .cooling_buff = 0U, .defence_buff = 0U, .vulnerability_buff = 0U, .attack_buff = 0U},
    {.recovery_buff = 5U, .cooling_buff = 200U, .defence_buff = 50U, .vulnerability_buff = 0U, .attack_buff = 100U},
    {.recovery_buff = 1U, .cooling_buff = 500U, .defence_buff = 0U, .vulnerability_buff = 25U, .attack_buff = 300U},
};

static const hurt_data_t hurt_frames[UART_TEST_FRAME_COUNT] = {
    {.armor_id = 0U, .hp_deduction_reason = 0U, .hit_sequence = 1U, .hit_position_x = 100, .hit_position_y = -50},
    {.armor_id = 2U, .hp_deduction_reason = 0U, .hit_sequence = 2U, .hit_position_x = -120, .hit_position_y = 60},
    {.armor_id = 0U, .hp_deduction_reason = 5U, .hit_sequence = 3U, .hit_position_x = 0, .hit_position_y = 0},
};

static const shoot_data_t shoot_frames[UART_TEST_FRAME_COUNT] = {
    {.bullet_type = 1U, .shooter_number = 1U, .launching_frequency = 10U, .initial_speed = 23.5f},
    {.bullet_type = 1U, .shooter_number = 1U, .launching_frequency = 20U, .initial_speed = 24.75f},
    {.bullet_type = 2U, .shooter_number = 3U, .launching_frequency = 1U, .initial_speed = 15.5f},
};

static const projectile_allowance_t projectile_allowance_frames[UART_TEST_FRAME_COUNT] = {
    {.projectile_allowance_17mm = 400U, .projectile_allowance_42mm = 0U, .remaining_gold_coin = 400U, .projectile_allowance_fortress = 0U},
    {.projectile_allowance_17mm = 250U, .projectile_allowance_42mm = 0U, .remaining_gold_coin = 650U, .projectile_allowance_fortress = 100U},
    {.projectile_allowance_17mm = 0U, .projectile_allowance_42mm = 5U, .remaining_gold_coin = 50U, .projectile_allowance_fortress = 300U},
};

static const rfid_status_t rfid_frames[UART_TEST_FRAME_COUNT] = {
    {0x00000000UL, 0x0000U},
    {0x00000101UL, 0x0001U},
    {0x80C01234UL, 0x00A5U},
};

static const ground_robot_position_t ground_position_frames[UART_TEST_FRAME_COUNT] = {
    {.heavyengineer_x = 1.0f, .heavyengineer_y = 2.0f, .standard_3_x = 3.0f, .standard_3_y = 4.0f, .standard_4_x = 5.0f, .standard_4_y = 6.0f},
    {.heavyengineer_x = 10.5f, .heavyengineer_y = 7.25f, .standard_3_x = 12.75f, .standard_3_y = 3.5f, .standard_4_x = 8.125f, .standard_4_y = 9.0f},
    {.heavyengineer_x = 20.25f, .heavyengineer_y = 13.5f, .standard_3_x = 0.5f, .standard_3_y = 14.75f, .standard_4_x = 27.0f, .standard_4_y = 0.25f},
};

static const sentry_info_t sentry_info_frames[UART_TEST_FRAME_COUNT] = {
    {0x00000000UL, 0x0000U, 0x0000000000000000ULL},
    {0x12345678UL, 0x0ABCU, 0x0123456789ABCDEFULL},
    {0xFEDCBA98UL, 0x07FFU, 0xA5A5A5A55A5A5A5AULL},
};

/* 周期取自 2027 通信协议表 1-5 的常规链路发送频率，3 Hz 取 333 ms；
 * 0x0002、0x0206、0x0207 为事件触发、协议无固定频率，测试中按 1 Hz 发送。 */
#define TEST_PACKET(id, period, array) {id, period, sizeof((array)[0]), (const uint8_t *)(array)}
static const test_packet_t judge_packets[UART_TEST_JUDGE_COUNT] = {
    TEST_PACKET(0x0001U, 1000U, game_status_frames),
    TEST_PACKET(0x0002U, 1000U, game_result_frames),
    TEST_PACKET(0x0003U, 333U, robot_hp_frames),
    TEST_PACKET(0x0101U, 1000U, event_frames),
    TEST_PACKET(0x0104U, 1000U, warning_frames),
    TEST_PACKET(0x0105U, 1000U, dart_info_frames),
    TEST_PACKET(0x0201U, 100U, robot_status_frames),
    TEST_PACKET(0x0202U, 100U, power_heat_frames),
    TEST_PACKET(0x0203U, 1000U, robot_pos_frames),
    TEST_PACKET(0x0204U, 333U, buff_frames),
    TEST_PACKET(0x0206U, 1000U, hurt_frames),
    TEST_PACKET(0x0207U, 1000U, shoot_frames),
    TEST_PACKET(0x0208U, 100U, projectile_allowance_frames),
    TEST_PACKET(0x0209U, 333U, rfid_frames),
    TEST_PACKET(0x020BU, 1000U, ground_position_frames),
    TEST_PACKET(0x020DU, 1000U, sentry_info_frames),
};
#undef TEST_PACKET

/* DMA 发送缓冲区：仅在 huart1.gState 回到 READY（上一帧结束或已中止）后改写。 */
static uint8_t tx_buffer[UART_TEST_TX_BUFFER_SIZE];
static uint32_t due_tick[UART_TEST_JUDGE_COUNT + 1U];
static uint8_t frame_index[UART_TEST_JUDGE_COUNT + 1U];

void UartTest_Task(void *argument)
{
    (void)argument;

    uint32_t next_tick = osKernelGetTickCount();
    uint32_t tx_start_tick = next_tick;
    uint8_t next_judge_packet = 0U;
    for (uint8_t index = 0U; index <= UART_TEST_JUDGE_COUNT; index++)
    {
        due_tick[index] = next_tick;
        frame_index[index] = 0U;
    }

    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();
        if (huart1.gState != HAL_UART_STATE_READY)
        {
            if (now_tick - tx_start_tick >= UART_TEST_TX_TIMEOUT_MS)
            {
                (void)HAL_UART_AbortTransmit(&huart1);
            }
        }
        else
        {
            /* 每个周期最多起发一帧：状态帧到期优先，其余按轮询顺序取第一个到期的裁判数据包。 */
            uint8_t selected = UART_TEST_STATE_INDEX;
            if ((int32_t)(now_tick - due_tick[UART_TEST_STATE_INDEX]) < 0)
            {
                for (uint8_t checked = 0U; checked < UART_TEST_JUDGE_COUNT; checked++)
                {
                    selected = (uint8_t)((next_judge_packet + checked) % UART_TEST_JUDGE_COUNT);
                    if ((int32_t)(now_tick - due_tick[selected]) >= 0)
                    {
                        break;
                    }
                    selected = UART_TEST_STATE_INDEX;
                }
            }

            if ((int32_t)(now_tick - due_tick[selected]) >= 0)
            {
                uint16_t length = sizeof(uart_send_cmd_t);
                uint16_t period_ms = UART_TEST_STATE_PERIOD_MS;
                if (selected == UART_TEST_STATE_INDEX)
                {
                    memcpy(tx_buffer, &state_frames[frame_index[selected]], sizeof(uart_send_cmd_t));
                    memcpy(&tx_buffer[offsetof(uart_send_cmd_t, frame_timestamp)], &now_tick, sizeof(now_tick));
                }
                else
                {
                    /* 与 uart_commun.c 的裁判转发帧一致：0x5A、载荷长度、小端命令码、4 字节 tick、载荷、0xA5。 */
                    const test_packet_t *packet = &judge_packets[selected];
                    tx_buffer[0] = 0x5AU;
                    tx_buffer[1] = packet->length;
                    tx_buffer[2] = (uint8_t)packet->command_id;
                    tx_buffer[3] = (uint8_t)(packet->command_id >> 8U);
                    memcpy(&tx_buffer[4], &now_tick, sizeof(now_tick));
                    memcpy(&tx_buffer[8], &packet->frames[(uint16_t)frame_index[selected] * packet->length], packet->length);
                    tx_buffer[8U + packet->length] = 0xA5U;
                    length = (uint16_t)(8U + packet->length + 1U);
                    period_ms = packet->period_ms;
                }

                if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, length) == HAL_OK)
                {
                    tx_start_tick = now_tick;
                    due_tick[selected] += period_ms;
                    frame_index[selected] = (uint8_t)((frame_index[selected] + 1U) % UART_TEST_FRAME_COUNT);
                    if (selected != UART_TEST_STATE_INDEX)
                    {
                        next_judge_packet = (uint8_t)((selected + 1U) % UART_TEST_JUDGE_COUNT);
                    }
                }
            }
        }

        next_tick += UART_TEST_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
