#ifndef USB_TASK_H
#define USB_TASK_H

#include "Task_Init.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define SEND_EOF	((uint8_t)0xA5)
#define RECEIVE_SOF ((uint8_t)0x5A)
#define RECEIVE_EOF	((uint8_t)0xA5)

/* Wire frame overhead: SOF/LEN/ID + timestamp + EOF. LEN counts DATA only. */
#define FRAME_FIXED_SIZE       ((uint8_t)0x08)
/******************************* 发送id *********************************/
#define ROBOT_INFO_DATA_SEND_ID    ((uint8_t)0x06)
#define GAME_STATUS_SEND_ID        ((uint8_t)0x07)
#define SELF_ROBOT_HP_SEND_ID       ((uint8_t)0x08)
#define ROBOT_LOCATION_SEND_ID	   ((uint8_t)0x09)
#define IMU_DATA_SEND_ID           ((uint8_t)0x10)
#define RFID_DATA_SEND_ID		   ((uint8_t)0x13)
#define ENEMY_LOCATION_SEND_ID	   ((uint8_t)0x14)
/******************************* 接收id *********************************/
#define ROBOT_CMD_DATA_RECEIVE_ID       ((uint8_t)0x11)
#define ROBOT_POSTURE_DATA_RECEIVE_ID	((uint8_t)0x12)

#define USB_RX_CHUNK_SIZE       64
#define USB_RX_QUEUE_SIZE       32

#define USB_PARSE_BUFFER_SIZE   256



typedef struct
{
    uint16_t len;

    uint8_t data[USB_RX_CHUNK_SIZE];

} USB_RxChunk_t;





typedef struct __attribute__((packed))
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
}Frame_Header_t;





typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header;
    uint32_t time_stamp;
    struct{
		float gimbal_yaw;  
		float yaw_chassis;	// rad		
        float yaw;    // rad
    }data;
	uint8_t eof;
} IMU_Data_Send_t;


// 机器人信息数据包
typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct __attribute__((packed))
    {
        uint8_t id;		// 机器人id
        uint8_t color;	// 0-red 1-blue 2-unknown
        bool attacked;	// 是否被击打
        uint16_t hp;	// 机器人剩余血量
        uint16_t heat;	// 机器人枪管热量
		uint16_t heat_limit;	//热量限制
		uint16_t projectile_allowance_17mm;	// 17mm弹丸剩余量
		uint8_t posture;	// 机器人姿态 1进攻 、2防御 、3移动
		bool nav_status;	// 1-已完成归中 0-未完成归中
		bool start_save_map;// 机器人是否开始保存地图(是否保存odin1地图)
    } data;
    uint8_t eof;
} RobotInfo_Data_Send_t;

// 比赛信息数据包
typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;
    struct __attribute__((packed))
    {
        uint8_t game_progress;	// 当前比赛阶段
        uint16_t stage_remain_time;	// 当前阶段剩余时间
    } data;
    uint8_t eof;
} GameStatus_Data_Send_t;

// 己方机器人hp信息数据包
typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    struct __attribute__((packed))
    {
        uint16_t hero_hp;
        uint16_t engineer_hp;
        uint16_t standard_3_hp;
        uint16_t standard_4_hp;
        uint16_t sentry_hp;
        uint16_t outpost_hp;
        uint16_t base_hp;
    } data;
    uint8_t eof;
} SelfRobotHp_Data_Send_t;

// 机器人位置数据包
typedef struct __attribute__((packed))
{
	Frame_Header_t frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;
	struct __attribute__((packed))
	{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
		float sentry_x;
		float sentry_y;
	} data;
	uint8_t eof;
} RobotLocation_Data_Send_t;

typedef struct __attribute__((packed))
{
	Frame_Header_t frame_header;  // 数据段id = 0x13
	uint32_t time_stamp;
	struct __attribute__((packed))
	{
		uint32_t rfid_status;
		uint8_t rfid_status_2;
	} data;
	uint8_t eof;
} Rfid_Data_Send_t;

typedef struct __attribute__((packed))
{
	Frame_Header_t frame_header;  // 数据段id = 0x14
	uint32_t time_stamp;
	struct __attribute__((packed))
	{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
		float sentry_x;
		float sentry_y;
	} data;	
	uint8_t eof;
} Enemy_Location_Send_t;


/*-------------------- Receive --------------------*/

typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header; 
    uint32_t time_stamp;
    struct __attribute__((packed))
    {
        struct __attribute__((packed))
        {
            float vx;
            float vy;
            float wz;
        } speed_vector;  // 底盘速度期望值

        struct __attribute__((packed))
        {
            float yaw;
			float yaw_angle;
			uint8_t posture;			// 机器人姿态 1进攻 、2防御 、3移动
		    bool follow_gimbal_big; 	//是否跟随大云台 0不跟随 、1跟随
		    bool track_status;			//是否启动履带 0 不启动 1 启动
		    bool perception_status; 	// 大云台是否跟随全向感知和云台联动
		    bool start_gimbal_big_spin;	//是否开始大云台旋转 0 不旋转 1 旋转
        } gimbal;
    } data;
	uint8_t eof;
} RobotCmdData_t;
extern RobotCmdData_t ROBOT_CMD_DATA ;
extern RobotCmdData_t ROBOT_CMD_LASTDATA ;

typedef struct __attribute__((packed))
{
    Frame_Header_t frame_header; 
    uint32_t time_stamp;	
	struct __attribute__((packed))
	{
		uint8_t posture;			// 机器人姿态 1进攻 、2防御 、3移动
		bool follow_gimbal_big; 	//是否跟随大云台 0不跟随 、1跟随
		bool track_status;			//是否启动履带 0 不启动 1 启动
		bool perception_status; 	// 大云台是否跟随全向感知和云台联动
		bool start_gimbal_big_spin;	//是否开始大云台旋转 0 不旋转 1 旋转
	} data;
	uint8_t eof;
} RobotPostureData_t;
extern RobotPostureData_t ROBOT_POSTURE_DATA;

extern TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters);



void USB_Rx_ReceiveFromISR(uint8_t *buf, uint16_t len);
void USB_Rx_Init(void);
void USB_Rx_Process(void);
#endif

