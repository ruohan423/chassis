//#ifndef USB_TASK_H
//#define USB_TASK_H

//#include "Task_Init.h"
//#include "stdbool.h"
//#include "FreeRTOS.h"
//#include "task.h"

//#define SEND_SOF       ((uint8_t)0x5A)
//#define SEND_EOF       ((uint8_t)0xA5)
//#define RECEIVE_SOF    ((uint8_t)0x5A)
//#define RECEIVE_EOF    ((uint8_t)0xA5)

//#define FRAME_FIXED_SIZE       ((uint8_t)0x08)

///******************************* 发送id *********************************/

//#define ROBOT_INFO_DATA_SEND_ID       ((uint8_t)0x06)
//#define GAME_STATUS_SEND_ID           ((uint8_t)0x07)
//#define SELF_ROBOT_HP_SEND_ID         ((uint8_t)0x08)
//#define ROBOT_LOCATION_SEND_ID        ((uint8_t)0x09)
//#define IMU_DATA_SEND_ID              ((uint8_t)0x10)
//#define RFID_DATA_SEND_ID             ((uint8_t)0x13)
//#define ENEMY_LOCATION_SEND_ID        ((uint8_t)0x14)

///******************************* 接收id *********************************/

//#define ROBOT_CMD_DATA_RECEIVE_ID     ((uint8_t)0x11)
//#define ROBOT_POSTURE_DATA_RECEIVE_ID ((uint8_t)0x12)


///******************************* 帧头 *********************************/

//typedef struct __attribute__((packed))
//{
//    uint8_t sof;
//    uint8_t len;
//    uint8_t id;
//} Frame_Header_t;


///******************************* 发送数据 *********************************/

//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct
//    {
//        float gimbal_yaw;
//        float yaw_chassis;
//        float yaw;
//    } data;

//    uint8_t eof;

//} IMU_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        uint8_t id;
//        uint8_t color;
//        bool attacked;
//        uint16_t hp;
//        uint16_t heat;
//        uint16_t heat_limit;
//        uint16_t projectile_allowance_17mm;
//        uint8_t posture;
//        bool nav_status;
//        bool start_save_map;
//    } data;

//    uint8_t eof;

//} RobotInfo_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        uint8_t game_progress;
//        uint16_t stage_remain_time;
//    } data;

//    uint8_t eof;

//} GameStatus_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        uint16_t hero_hp;
//        uint16_t engineer_hp;
//        uint16_t standard_3_hp;
//        uint16_t standard_4_hp;
//        uint16_t sentry_hp;
//        uint16_t outpost_hp;
//        uint16_t base_hp;
//    } data;

//    uint8_t eof;

//} SelfRobotHp_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        float hero_x;
//        float hero_y;
//        float engineer_x;
//        float engineer_y;
//        float standard_3_x;
//        float standard_3_y;
//        float standard_4_x;
//        float standard_4_y;
//        float sentry_x;
//        float sentry_y;
//    } data;

//    uint8_t eof;

//} RobotLocation_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        uint32_t rfid_status;
//        uint8_t rfid_status_2;
//    } data;

//    uint8_t eof;

//} Rfid_Data_Send_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        float hero_x;
//        float hero_y;
//        float engineer_x;
//        float engineer_y;
//        float standard_3_x;
//        float standard_3_y;
//        float standard_4_x;
//        float standard_4_y;
//        float sentry_x;
//        float sentry_y;
//    } data;

//    uint8_t eof;

//} Enemy_Location_Send_t;


///******************************* 接收数据 *********************************/

//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        struct __attribute__((packed))
//        {
//            float vx;
//            float vy;
//            float wz;
//        } speed_vector;

//        struct __attribute__((packed))
//        {
//            float yaw;
//            float yaw_angle;
//            uint8_t posture;
//            bool follow_gimbal_big;
//            bool track_status;
//            bool perception_status;
//            bool start_gimbal_big_spin;
//        } gimbal;

//    } data;

//    uint8_t eof;

//} RobotCmdData_t;


//typedef struct __attribute__((packed))
//{
//    Frame_Header_t frame_header;

//    uint32_t time_stamp;

//    struct __attribute__((packed))
//    {
//        uint8_t posture;
//        bool follow_gimbal_big;
//        bool track_status;
//        bool perception_status;
//        bool start_gimbal_big_spin;
//    } data;

//    uint8_t eof;

//} RobotPostureData_t;


///******************************* USB TX *********************************/

//#define USB_TX_BUFFER_NUM       7
//#define USB_TX_DOUBLE_BUFFER    2
//#define USB_TX_MAX_SIZE        64


//typedef struct
//{
//    uint8_t buffer[USB_TX_DOUBLE_BUFFER][USB_TX_MAX_SIZE];

//    uint16_t len[USB_TX_DOUBLE_BUFFER];

//    uint8_t write_index;
//    uint8_t send_index;

//    bool ready[USB_TX_DOUBLE_BUFFER];

//    bool sending;

//} USB_TxSlot_t;


///******************************* 全局变量 *********************************/

//extern USB_TxSlot_t USB_TxSlot[USB_TX_BUFFER_NUM];

//extern RobotCmdData_t ROBOT_CMD_DATA;
//extern RobotCmdData_t ROBOT_CMD_LASTDATA;

//extern RobotPostureData_t ROBOT_POSTURE_DATA;

//extern TaskHandle_t usb_task_handle;


///******************************* 函数 *********************************/

//void USB_Task_Init(void);

//void usb_task(void *pvParameters);


///* 各种数据填充函数 */

//void USB_Send_RobotInfo(RobotInfo_Data_Send_t *data);

//void USB_Send_GameStatus(GameStatus_Data_Send_t *data);

//void USB_Send_SelfRobotHP(SelfRobotHp_Data_Send_t *data);

//void USB_Send_RobotLocation(RobotLocation_Data_Send_t *data);

//void USB_Send_IMU(IMU_Data_Send_t *data);

//void USB_Send_RFID(Rfid_Data_Send_t *data);

//void USB_Send_EnemyLocation(Enemy_Location_Send_t *data);


///* USB真正发送接口 */

//uint8_t USB_Transmit(uint8_t *Buf, uint16_t Len);


//#endif