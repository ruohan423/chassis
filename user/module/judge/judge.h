//
// Created by mot on 2026/11/1.
//

#ifndef SENTRY_CHASSIS_JUDGE_H
#define SENTRY_CHASSIS_JUDGE_H

#include <stdbool.h>
#include <stdint.h>

/* 常规链路当前支持的整帧最大 127 字节，交互载荷最多 118 字节。 */
#define JUDGE_FRAME_HEADER_SIZE 5U
#define JUDGE_FRAME_OVERHEAD 9U
#define JUDGE_FRAME_MAX_SIZE 127U
#define JUDGE_FRAME_DATA_SIZE (JUDGE_FRAME_MAX_SIZE - JUDGE_FRAME_OVERHEAD)
#define JUDGE_LINK_TIMEOUT_MS 200U
#define JUDGE_TX_QUEUE_DEPTH 4U
#define JUDGE_COMMAND_QUEUE_DEPTH 4U
#define JUDGE_TX_MAX_HZ 30U
#define JUDGE_TX_DEFAULT_BYTE_LIMIT 3000U
#define JUDGE_TX_MAX_AGE_MS 500U
#define JUDGE_TX_TIMEOUT_MS 50U


typedef struct __attribute__((packed))
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} frame_header_t;

typedef struct __attribute__((packed))
{
    uint16_t  crc16;        //2-byte，CRC16，整包校验
} frame_tail_t;

/* 已解码整帧，data 的有效长度由 header.data_length 给出。
 * 成员可能含内存填充，收发都按字节编解码，不能直接发送 sizeof(judge_frame_t)。 */
typedef struct
{
    frame_header_t header;
    uint16_t cmd_id;
    uint8_t data[JUDGE_FRAME_DATA_SIZE];
    frame_tail_t tail;
} judge_frame_t;

/* ID: 0x0001   Byte:   11  比赛状态数据 */
typedef struct __attribute__((packed))
{
    uint8_t game_type : 4;      //比赛类型
    uint8_t game_progress : 4;  //当前比赛阶段
    uint16_t stage_remain_time; //当前阶段剩余时间，单位：秒
    uint64_t sync_timestamp;     //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;

/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef struct __attribute__((packed))
{
    uint8_t winner;
} game_result_t;

/* ID: 0x0003	Byte:	24	机器人血量数据 */
typedef struct __attribute__((packed))
{
    uint16_t ally_1_robot_hp;       //己方 1 号重装机器人血量
    uint16_t reserved;              //保留位
    uint16_t ally_3_robot_hp;       //己方 3 号步兵机器人血量
    uint16_t ally_4_robot_hp;       //己方 4 号步兵机器人血量
    int16_t damage_difference;      //己方全队总伤害与对方全队总伤害之差
    uint16_t ally_7_robot_hp;       //己方 7 号哨兵机器人血量
    uint16_t outpost_r_hp;          //R 前哨血量
    uint16_t ally_base_hp;          //己方基地血量
    uint16_t outpost_b_hp;          //B 前哨血量
    uint16_t enemy_base_hp;         //对方基地血量
    uint16_t ally_control_degree;   //己方累计控制度
    uint16_t enemy_control_degree;  //对方累计控制度
} game_robot_hp_t;

/* ID: 0x0101	Byte:	8	场地事件数据 */
typedef struct __attribute__((packed))
{
    uint32_t event_data;        //
    uint32_t event_data_2;
} event_data_t;

/* ID: 0x0104   Byte:    3  裁判警告数据 */
typedef struct __attribute__((packed))
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
}referee_warning_t;

/* ID: 0x0105   Byte:   3   飞镖发射相关数据*/
// typedef struct __attribute__((packed))
// {
//     uint8_t dart_remaining_time;
//     uint16_t dart_info;
// }dart_info_t;

/* ID: 0x0201   Byte:   17 	机器人性能体系数据 */
typedef struct __attribute__((packed))
{
    uint8_t robot_id;            //本机器人 ID
    uint8_t robot_level;         //机器人等级
    uint16_t current_hp;         //机器人当前血量
    uint16_t maximum_hp;         //机器人血量上限
    uint16_t shooter_barrel_cooling_value;   //机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;      //机器人枪口热量上限
    uint16_t chassis_power_limit;            //机器人底盘功率上限
    float bullet_speed_limit;                //机器人射击初速度上限
    uint8_t power_management_gimbal_output : 1; //gimbal 口输出
    uint8_t power_management_chassis_output : 1; //chassis 口输出
    uint8_t power_management_shooter_output : 1;//shooter 口输出
} robot_status_t;

/* ID: 0x0202   Byte:   14 	实时底盘缓冲能量和射击热量数据 */
typedef struct __attribute__((packed))
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;        //保留位
    uint16_t buffer_energy;     //缓冲能量（单位：J）
    uint16_t shooter_17mm_barrel_heat;         //17mm 发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;           //42mm 发射机构的射击热量

} power_heat_data_t;

/* ID: 0x0203   Byte:   12 	机器人位置数据 */
typedef struct __attribute__((packed))
{
    float x;        //本机器人位置 x 坐标，单位：m
    float y;        //本机器人位置 y 坐标，单位：m
    float angle;    //本机器人测速模块的朝向，单位：度，正北为 0 度
} robot_pos_t;

/* ID: 0x0204   Byte:   7 机器人增益数据 */
typedef struct __attribute__((packed))
{
    uint8_t recovery_buff;      //机器人回血增益
    uint16_t cooling_buff;      //机器人射击热量冷却增益具体值
    uint8_t defence_buff;       //机器人防御增益
    uint8_t vulnerability_buff; //机器人负防御增益
    uint16_t attack_buff;       //机器人攻击增益
} buff_t;

/* ID: 0x0206   Byte:   7 	伤害状态数据 */
typedef struct __attribute__((packed))
{
    uint8_t armor_id : 4;               //bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该 4 bit组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t hp_deduction_reason : 4;    //bit 4-7：血量变化类型
    uint16_t hit_sequence;              //命中事件序号
    int16_t hit_position_x;             //命中位置 x 坐标（单位：mm）
    int16_t hit_position_y;             //命中位置 y 坐标（单位：mm）
} hurt_data_t;

/* ID: 0x0207   Byte:   7 	实时射击数据 */
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;            //弹丸类型：1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_number;         //发射机构 ID：1：17mm 发射机构 2：保留位 3：42mm 发射机构
    uint8_t launching_frequency;    //弹丸射速（单位：Hz）
    float initial_speed;            //弹丸初速度（单位：m/s）
} shoot_data_t;

/* ID: 0x0208   Byte:   8 	允许发弹量 */
typedef struct __attribute__((packed))
{
    uint16_t projectile_allowance_17mm;         //机器人自身拥有的 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;         //42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;               //剩余金币数量
    uint16_t projectile_allowance_fortress;     //堡垒增益点提供的储备允许发弹量；该值与机器人是否实际激活堡垒无关
} projectile_allowance_t;

/* ID: 0x0209   Byte:   6 	机器人RFID模块状态 */
typedef struct __attribute__((packed))
{
    uint32_t rfid_status;
    uint16_t rfid_status_2;
} rfid_status_t;

/* ID: 0x020B   Byte:   40 	地面机器人位置数据 */
typedef struct __attribute__((packed))
{
    float heavyengineer_x;  //己方重装机器人位置 x 轴坐标，单位：m
    float heavyengineer_y;  //己方重装机器人位置 y 轴坐标，单位：m
    float reserved1;        //保留位
    float reserved2;        //保留位
    float standard_3_x;     //己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_y;     //己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_x;     //己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_y;     //己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float reserved3;        //保留位
    float reserved4;        //保留位
} ground_robot_position_t;

/* ID: 0x020D   Byte:   14 	哨兵自主决策信息同步 */
typedef struct __attribute__((packed))
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3;
} sentry_info_t;

//哨兵自主决策指令：0x0120
typedef struct __attribute__((packed))
{
    uint32_t sentry_cmd;
} sentry_cmd_t;

/* ID: 0x0301 数据段最多 118 字节，包含 6 字节交互头。 */
#define JUDGE_INTERACTION_DATA_SIZE 112U
typedef struct __attribute__((packed))
{
    uint16_t data_cmd_id;       //子内容 ID
    uint16_t sender_id;         //发送者 ID
    uint16_t receiver_id;       //接收者 ID
    uint8_t user_data[JUDGE_INTERACTION_DATA_SIZE];       //内容数据段 x 最大为 112
} robot_interaction_data_t;


typedef struct __attribute__((packed))
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} judge_dart_info_t;

typedef struct __attribute__((packed))
{
    float target_position_x;       /* 单位：m */
    float target_position_y;       /* 单位：m */
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} map_command_t;

typedef enum
{
    JUDGE_DATA_GAME_STATUS,
    JUDGE_DATA_GAME_RESULT,
    JUDGE_DATA_ROBOT_HP,
    JUDGE_DATA_EVENT,
    JUDGE_DATA_WARNING,
    JUDGE_DATA_DART_INFO,
    JUDGE_DATA_ROBOT_STATUS,
    JUDGE_DATA_POWER_HEAT,
    JUDGE_DATA_ROBOT_POS,
    JUDGE_DATA_BUFF,
    JUDGE_DATA_HURT,
    JUDGE_DATA_SHOOT,
    JUDGE_DATA_PROJECTILE_ALLOWANCE,
    JUDGE_DATA_RFID,
    JUDGE_DATA_GROUND_POSITION,
    JUDGE_DATA_SENTRY_INFO,
    JUDGE_DATA_INTERACTION,
    JUDGE_DATA_MAP_COMMAND,
    JUDGE_DATA_COUNT
} judge_data_id_t;

typedef struct
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_hp_t robot_hp;
    event_data_t event;
    referee_warning_t warning;
    judge_dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat;
    robot_pos_t robot_pos;
    buff_t buff;
    hurt_data_t hurt;
    shoot_data_t shoot;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid;
    ground_robot_position_t ground_position;
    sentry_info_t sentry_info;
    robot_interaction_data_t interaction;
    uint16_t interaction_length;    /* user_data 的有效字节数 */
    map_command_t map_command;
    uint32_t valid_mask;
    uint32_t update_tick[JUDGE_DATA_COUNT];
} judge_data_package_t;

extern judge_data_package_t judge_data;


typedef struct
{
    uint16_t cmd_id;
    uint16_t data_length;
    uint8_t sequence;
    uint32_t received_tick;
    uint8_t data[JUDGE_FRAME_DATA_SIZE];
} judge_received_command_t;

typedef enum
{
    JUDGE_COMMAND_EMPTY = 0,
    JUDGE_COMMAND_READY,
    JUDGE_COMMAND_LOST,
} judge_command_result_t;

typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint16_t data_length;          /* data 的有效字节数 */
    uint32_t queued_tick;
    uint8_t data[JUDGE_INTERACTION_DATA_SIZE];
} judge_tx_request_t;

typedef enum
{
    JUDGE_TX_IDLE = 0,
    JUDGE_TX_QUEUED,
    JUDGE_TX_READY,
    JUDGE_TX_ACTIVE,
    JUDGE_TX_DONE,
    JUDGE_TX_INVALID,
    JUDGE_TX_QUEUE_FULL,
    JUDGE_TX_OFFLINE,
    JUDGE_TX_CONFIG_ERROR,
    JUDGE_TX_EXPIRED,
    JUDGE_TX_DMA_ERROR,
    JUDGE_TX_TIMEOUT,
    JUDGE_TX_UART_BUSY,
    JUDGE_TX_NOT_READY,
} judge_tx_result_t;

typedef struct
{
    judge_tx_result_t result;
    uint16_t data_cmd_id;
    uint16_t receiver_id;
    uint8_t sequence;
    uint16_t byte_limit_per_second;
} judge_tx_status_t;

/* 结果字段仅 Judge_Task 写；字节预算仅由 judge_set_tx_bandwidth() 修改。 */
extern judge_tx_status_t judge_tx_status;

/* 接收、清空、拼帧复位仅由 Judge_Task 调用；时间单位为 CMSIS-OS2 tick。
 * 接收返回是否识别到支持的有效命令，队列满时仍维持接收链路时效。 */
bool judge_receive(const uint8_t *data, uint16_t length, uint32_t now_tick);
void judge_clear(void);
void judge_reset_parser(void);

bool judge_data_is_fresh(const judge_data_package_t *data, judge_data_id_t id, uint32_t now_tick);
/* 0x0301/0x0303 保序入队，满时丢新；LOST 表示队列溢出或接收流重置（包括初始化）。 */
judge_command_result_t judge_read_command(judge_received_command_t *command);

/* 仅入队，不代表已发送或服务器已执行；发送者 ID 从新鲜的哨兵状态中取得。
 * 支持 0x0120（4 字节，服务器 0x8080）和 0x0200～0x02FF（己方机器人）。 */
judge_tx_result_t judge_send_interaction(uint16_t data_cmd_id, uint16_t receiver_id, const uint8_t *data, uint16_t length);
judge_tx_result_t judge_send_sentry(const sentry_cmd_t *command);
/* 本机整帧字节/秒预算，0 暂停新帧；不能替代队伍对接收方共享额度的分配。 */
void judge_set_tx_bandwidth(uint16_t bytes_per_second);
void judge_read_tx_status(judge_tx_status_t *status);

/* 发送消费者接口仅由 Judge_Task 使用；buffer 至少 JUDGE_FRAME_MAX_SIZE 字节。 */
bool judge_take_tx(judge_tx_request_t *request);
judge_tx_result_t judge_prepare_tx(const judge_tx_request_t *request, uint8_t sequence, uint32_t now_tick,
                                    uint8_t *buffer, uint16_t *length);

#endif
