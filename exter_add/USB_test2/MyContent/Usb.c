#include "usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "WatchDog_Task.h"
#include "arm_math.h"

#define USB_OFFLINE_THRESHOLD 1000  // ms USB离线时间阈值
#define USB_CONNECT_CNT 10

// 发送时间间隔，每次发送经过以下时间后发送消息

#define SEND_DURATION_AllRobotHp    10  // ms
#define SEND_DURATION_GameStatus    10  // ms 

#define SEND_DURATION_GameBegin     10  // ms 
#define SEND_DURATION_Vulnerable	10	// ms
#define SEND_DURATION_Radar_Mark    10  // ms

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

// 判断USB连接状态用到的一些变量
bool USB_OFFLINE = true;
uint32_t RECEIVE_TIME = 0;
uint32_t LATEST_RX_TIMESTAMP = 0;
uint32_t CONTINUE_RECEIVE_CNT = 0;


uint32_t usb_R_ok_cnt = 0;
uint32_t usb_R_busy_cnt = 0;
uint32_t usb_R_fail_cnt = 0;

uint32_t usb_G_ok_cnt = 0;
uint32_t usb_G_busy_cnt = 0;
uint32_t usb_G_fail_cnt = 0;

uint32_t usb_M_ok_cnt = 0;
uint32_t usb_M_busy_cnt = 0;
uint32_t usb_M_fail_cnt = 0;
uint32_t usb_flag_cnt = 0;

uint32_t usb_L_ok_cnt = 0;
uint32_t usb_L_busy_cnt = 0;
uint32_t usb_L_fail_cnt = 0;

uint32_t usb_S_ok_cnt = 0;
uint32_t usb_S_busy_cnt = 0;
uint32_t usb_S_fail_cnt = 0;

uint32_t usb_F_ok_cnt = 0;
uint32_t usb_F_busy_cnt = 0;
uint32_t usb_F_fail_cnt = 0;

uint32_t usb_E_ok_cnt = 0;
uint32_t usb_E_busy_cnt = 0;
uint32_t usb_E_fail_cnt = 0;


volatile uint32_t usb_rx_queue_full = 0;

volatile uint32_t usb_rx_overflow = 0;

volatile uint32_t usb_rx_bad_sof = 0;

volatile uint32_t usb_rx_bad_id = 0;

volatile uint32_t usb_rx_bad_len = 0;

volatile uint32_t usb_rx_bad_eof = 0;

volatile uint32_t usb_rx_success = 0;





// 数据发送结构体
static IMU_Data_Send_t            IMU_Data_Send;             // 发送IMU数据
static RobotInfo_Data_Send_t	  RobotInfo_Data_Send;
static GameStatus_Data_Send_t	  GameStatus_Data_Send;
static SelfRobotHp_Data_Send_t	  SelfRobotHp_Data_Send;
static RobotLocation_Data_Send_t  RobotLocation_Data_Send;
static Rfid_Data_Send_t 		  Rfid_Data_Send;
static Enemy_Location_Send_t	  Enemy_Location_Send;

// 机器人控制指令数据（视觉导航发来的x,y,z）
RobotCmdData_t ROBOT_CMD_DATA;
RobotCmdData_t ROBOT_CMD_LASTDATA;
RobotPostureData_t ROBOT_POSTURE_DATA;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
//static void UsbReceiveData(void);
static void UsbInit(void);
/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
static void UsbSendIMUData(void);
static void UsbSendRobotInfoData(void);
static void UsbSendGameStatusData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendRfidData(void);
static void UsbSendRobotLocationData(void);
static void UsbSendEnemyLocationData(void);
/*******************************************************************************/
/* Receive Function                                                               */
/*******************************************************************************/
static uint8_t parse_buffer[USB_PARSE_BUFFER_SIZE];

static uint16_t parse_len = 0;
/******************************************************************/
/* Task                                                           */
/******************************************************************/
/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
 
TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters)
{
    UsbInit();

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /*
         * Drain received packets before telemetry transmission.
         * USB_Transmit is non-blocking, so RX processing remains responsive.
         */
        USB_Rx_Process();
        UsbSendData();

        if (usb_rx_success == 0U ||
            (HAL_GetTick() - RECEIVE_TIME) >= USB_OFFLINE_THRESHOLD)
        {
            USB_OFFLINE = true;
        }
        else
        {
            USB_OFFLINE = false;
        }

        vTaskDelayUntil(&xLastWakeTime,2);
    }
}


/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
QueueHandle_t usb_rx_queue;

void USB_Rx_Init(void)
{
    if (usb_rx_queue == NULL)
    {
        usb_rx_queue = xQueueCreate(USB_RX_QUEUE_SIZE, sizeof(USB_RxChunk_t));
        configASSERT(usb_rx_queue != NULL);
    }
}

static void UsbInit(void)
{
    RECEIVE_TIME = 0U;
	
    parse_len = 0;

    memset(&ROBOT_CMD_DATA,0,sizeof(ROBOT_CMD_DATA));


    memset(&ROBOT_CMD_LASTDATA,0,sizeof(ROBOT_CMD_LASTDATA));
	
	memset(&ROBOT_POSTURE_DATA,0,sizeof(ROBOT_POSTURE_DATA));
	
    // 数据置零
	memset(&IMU_Data_Send, 0, sizeof(IMU_Data_Send_t));
	memset(&SelfRobotHp_Data_Send, 0, sizeof(SelfRobotHp_Data_Send_t));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
	memset(&ROBOT_POSTURE_DATA, 0, sizeof(RobotPostureData_t));
	
    IMU_Data_Send.frame_header.sof = SEND_SOF;
    IMU_Data_Send.frame_header.len = (uint8_t)(sizeof(IMU_Data_Send.data));
    IMU_Data_Send.frame_header.id = IMU_DATA_SEND_ID;
	IMU_Data_Send.eof = SEND_EOF;
	
	RobotInfo_Data_Send.frame_header.sof = SEND_SOF;
	RobotInfo_Data_Send.frame_header.len = (uint8_t)(sizeof(RobotInfo_Data_Send.data));
	RobotInfo_Data_Send.frame_header.id = ROBOT_INFO_DATA_SEND_ID;
	RobotInfo_Data_Send.eof = SEND_EOF;
	
	GameStatus_Data_Send.frame_header.sof = SEND_SOF;
	GameStatus_Data_Send.frame_header.len = (uint8_t)(sizeof(GameStatus_Data_Send.data));
	GameStatus_Data_Send.frame_header.id = GAME_STATUS_SEND_ID;
	GameStatus_Data_Send.eof = SEND_EOF;
	
	SelfRobotHp_Data_Send.frame_header.sof = SEND_SOF;
	SelfRobotHp_Data_Send.frame_header.len = (uint8_t)(sizeof(SelfRobotHp_Data_Send.data));
	SelfRobotHp_Data_Send.frame_header.id = SELF_ROBOT_HP_SEND_ID;
	SelfRobotHp_Data_Send.eof = SEND_EOF;
	
	RobotLocation_Data_Send.frame_header.sof = SEND_SOF;
	RobotLocation_Data_Send.frame_header.len = (uint8_t)(sizeof(RobotLocation_Data_Send.data));
	RobotLocation_Data_Send.frame_header.id = ROBOT_LOCATION_SEND_ID;
	RobotLocation_Data_Send.eof = SEND_EOF;
	
	Rfid_Data_Send.frame_header.sof = SEND_SOF;
	Rfid_Data_Send.frame_header.len = (uint8_t)(sizeof(Rfid_Data_Send.data));
	Rfid_Data_Send.frame_header.id = RFID_DATA_SEND_ID;
	Rfid_Data_Send.eof = SEND_EOF;
	
	Enemy_Location_Send.frame_header.sof = SEND_SOF;
	Enemy_Location_Send.frame_header.len = (uint8_t)(sizeof(Enemy_Location_Send.data));
	Enemy_Location_Send.frame_header.id = ENEMY_LOCATION_SEND_ID;
	Enemy_Location_Send.eof = SEND_EOF;
	
	ROBOT_CMD_DATA.frame_header.sof = RECEIVE_SOF;
	ROBOT_CMD_DATA.frame_header.id = ROBOT_CMD_DATA_RECEIVE_ID;
	ROBOT_CMD_DATA.eof = RECEIVE_EOF;

	ROBOT_POSTURE_DATA.frame_header.sof = RECEIVE_SOF;
	ROBOT_POSTURE_DATA.frame_header.id = ROBOT_POSTURE_DATA_RECEIVE_ID;
	ROBOT_POSTURE_DATA.eof = RECEIVE_EOF;

}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
	UsbSendIMUData();
	UsbSendRobotInfoData();
	UsbSendGameStatusData();
	UsbSendAllRobotHpData();
	UsbSendRobotLocationData();
	UsbSendRfidData();
	UsbSendEnemyLocationData();
}
/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
/**
 * @brief 发送IMU数据
 */
static void UsbSendIMUData(void)
{
	IMU_Data_Send.time_stamp = HAL_GetTick();
	IMU_Data_Send.data.gimbal_yaw = 12;
	IMU_Data_Send.data.yaw_chassis = 13;
	IMU_Data_Send.data.yaw = 14;
	uint8_t ret = USB_Transmit((uint8_t *)&IMU_Data_Send, sizeof(IMU_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_M_ok_cnt++;
			usb_flag_cnt = 1;
			break;

		case USBD_BUSY:
			usb_M_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_M_fail_cnt++;
			break;
	}
}
/**
 * @brief 发送机器人信息数据
 */
static void UsbSendRobotInfoData(void)
{
    RobotInfo_Data_Send.time_stamp = HAL_GetTick();

    RobotInfo_Data_Send.data.id = 7;
	RobotInfo_Data_Send.data.color = (RobotInfo_Data_Send.data.id == 7) ? 0 : 1;
	RobotInfo_Data_Send.data.attacked = 10;
    RobotInfo_Data_Send.data.hp = 500;
	RobotInfo_Data_Send.data.heat = 20;
	RobotInfo_Data_Send.data.heat_limit = 40;
	RobotInfo_Data_Send.data.projectile_allowance_17mm = 300;
	RobotInfo_Data_Send.data.posture = 10;
	RobotInfo_Data_Send.data.nav_status = 5;
	RobotInfo_Data_Send.data.start_save_map = 3;
	
    uint8_t ret = USB_Transmit((uint8_t *)&RobotInfo_Data_Send, sizeof(RobotInfo_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_R_ok_cnt++;
			break;

		case USBD_BUSY:
			usb_R_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_R_fail_cnt++;
			break;
	}
}

/**
 * @brief 发送比赛信息数据
 */
static void UsbSendGameStatusData(void)
{
    GameStatus_Data_Send.time_stamp = HAL_GetTick();

    GameStatus_Data_Send.data.game_progress = 40;
    GameStatus_Data_Send.data.stage_remain_time = 200;

    uint8_t ret = USB_Transmit((uint8_t *)&GameStatus_Data_Send, sizeof(GameStatus_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_G_ok_cnt++;
			break;

		case USBD_BUSY:
			usb_G_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_G_fail_cnt++;
			break;
	}
}

/**
 * @brief 发送全场机器人hp信息数据
 */
static void UsbSendAllRobotHpData(void)
{
    SelfRobotHp_Data_Send.time_stamp = HAL_GetTick();
	
	SelfRobotHp_Data_Send.data.hero_hp = 101;
	SelfRobotHp_Data_Send.data.engineer_hp = 102;
	SelfRobotHp_Data_Send.data.standard_3_hp = 103;
	SelfRobotHp_Data_Send.data.standard_4_hp = 104;
	SelfRobotHp_Data_Send.data.sentry_hp = 105;
	SelfRobotHp_Data_Send.data.base_hp = 106;
	SelfRobotHp_Data_Send.data.outpost_hp = 107;


	uint8_t ret = USB_Transmit((uint8_t *)&SelfRobotHp_Data_Send, sizeof(SelfRobotHp_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_S_ok_cnt++;
			usb_flag_cnt = 0;
			break;

		case USBD_BUSY:
			usb_S_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_S_fail_cnt++;
			break;
	}
}

/**
 * @brief 发送机器人位置数据
 */
static void UsbSendRobotLocationData(void)
{
    RobotLocation_Data_Send.time_stamp = HAL_GetTick();

	RobotLocation_Data_Send.data.hero_x = 40;
    RobotLocation_Data_Send.data.hero_y = 20;
    RobotLocation_Data_Send.data.engineer_x = 45;
	RobotLocation_Data_Send.data.engineer_y = 25;
	RobotLocation_Data_Send.data.standard_3_x = 50;
    RobotLocation_Data_Send.data.standard_3_y = 30;	
    RobotLocation_Data_Send.data.standard_4_x = 55;
    RobotLocation_Data_Send.data.standard_4_y = 35;
    RobotLocation_Data_Send.data.sentry_x = 60;
    RobotLocation_Data_Send.data.sentry_y = 40;
	
    uint8_t ret = USB_Transmit((uint8_t *)&RobotLocation_Data_Send, sizeof(RobotLocation_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_L_ok_cnt++;
			break;

		case USBD_BUSY:
			usb_L_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_L_fail_cnt++;
			break;
	}
}

/**
 * @brief 发送机器人Rfid数据
 */
static void UsbSendRfidData(void)
{
    Rfid_Data_Send.time_stamp = HAL_GetTick();

	Rfid_Data_Send.data.rfid_status = 1;
    Rfid_Data_Send.data.rfid_status_2 = 2;

    uint8_t ret = USB_Transmit((uint8_t *)&Rfid_Data_Send, sizeof(Rfid_Data_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_F_ok_cnt++;
			break;

		case USBD_BUSY:
			usb_F_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_F_fail_cnt++;
			break;
	}
}

/**
 * @brief 发送敌人位置数据
 */
static void UsbSendEnemyLocationData(void)
{
    Enemy_Location_Send.time_stamp = HAL_GetTick();

	Enemy_Location_Send.data.hero_x = (float)(2000 / 100.f);
    Enemy_Location_Send.data.hero_y = (float)(1900 / 100.f);
	Enemy_Location_Send.data.engineer_x = (float)(1800 / 100.f);
    Enemy_Location_Send.data.engineer_y = (float)(1700 / 100.f);
	Enemy_Location_Send.data.standard_3_x = (float)(1600 / 100.f);
    Enemy_Location_Send.data.standard_3_y = (float)(1500 / 100.f);
	Enemy_Location_Send.data.standard_4_x = (float)(1400 / 100.f);
    Enemy_Location_Send.data.standard_4_y = (float)(1300 / 100.f);
	Enemy_Location_Send.data.sentry_x = (float)(1200 / 100.f);
    Enemy_Location_Send.data.sentry_y = (float)(1100 / 100.f);	

    uint8_t ret = USB_Transmit((uint8_t *)&Enemy_Location_Send, sizeof(Enemy_Location_Send_t));
	switch(ret)
	{
		case USBD_OK:
			usb_E_ok_cnt++;
			break;

		case USBD_BUSY:
			usb_E_busy_cnt++;
			break;

		case USBD_FAIL:
			usb_E_fail_cnt++;
			break;
	}
}


//接收函数
 void USB_Rx_ReceiveFromISR(uint8_t *buf,uint16_t len)
{
    if (usb_rx_queue == NULL)
    {
        return;
    }

    if (len == 0)
    {
        return;
    }

    if (len > USB_RX_CHUNK_SIZE)
    {
        return;
    }

    USB_RxChunk_t chunk;

    /* 保存这一次USB收到多少字节 */

    chunk.len = len;


    /* 保存USB数据 */
    memcpy(chunk.data,buf,len);
    /* 放进Queue */
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendFromISR(usb_rx_queue, &chunk,
                          &higher_priority_task_woken) != pdPASS)
    {
        usb_rx_queue_full++;
    }
    else
    {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}


/* ========================================================
 * 删除parse_buffer最前面的N个字节
 * ======================================================== */

static void USB_Parse_RemoveFront(uint16_t count)
{
    if (count >= parse_len)
    {
        parse_len = 0;

        return;
    }
    memmove(parse_buffer, &parse_buffer[count], parse_len - count);
    parse_len -= count;
}


/* ========================================================
 * 向parse_buffer里面添加数据
 * ======================================================== */

static void USB_Parse_Append(uint8_t *data,uint16_t len)
{
    if (len == 0)
    {
        return;
    }
    /* 数据太多，parse_buffer放不下 */
    if ((parse_len + len) > USB_PARSE_BUFFER_SIZE)
    {
        /*
         * 当前解析状态无法保证正确
         *
         * 清空重新开始
         */
        parse_len = 0;
        usb_rx_overflow++;
        /*
         * 如果这一批数据本身没有超过缓冲区
         * 就把这一批重新放进去
         */
        if (len <= USB_PARSE_BUFFER_SIZE)
        {
            memcpy( parse_buffer,data,len);
            parse_len = len;
        }
        return;
    }
    memcpy(&parse_buffer[parse_len],data,len);
    parse_len += len;
}


/* ========================================================
 * 找SOF
 *
 * 返回：
 *
 * >= 0  找到了
 *
 * -1    没找到
 * ======================================================== */

static int USB_FindSOF(void)
{
    for (uint16_t i = 0; i < parse_len; i++)
    {
        if (parse_buffer[i] == RECEIVE_SOF)
        {
            return i;
        }
    }
    return -1;
}

/* ========================================================
 * 根据ID获得这个包应该有多长
 *
 * 返回的是：
 *
 * 完整帧长度
 *
 * Header + Data + EOF
 * ======================================================== */
static uint16_t USB_GetExpectedFrameLength(uint8_t id)
{
    switch (id)
    {
        case ROBOT_CMD_DATA_RECEIVE_ID:
            return sizeof(RobotCmdData_t);
        case ROBOT_POSTURE_DATA_RECEIVE_ID:
            return sizeof(RobotPostureData_t);
        default:
            return 0;
    }
}


/* ========================================================
 * 处理完整数据包
 * ======================================================== */
static void USB_HandleFrame(uint8_t *frame,uint16_t frame_len)
{
    Frame_Header_t header;

    if (frame_len < sizeof(Frame_Header_t))
    {
        return;
    }
    if (frame_len != USB_GetExpectedFrameLength(frame[2]))
    {
        usb_rx_bad_len++;
        return;
    }

    memcpy(&header,frame,sizeof(Frame_Header_t));
    switch (header.id)
    {
        /* =================================================
         * Robot Command
         * ================================================= */

        case ROBOT_CMD_DATA_RECEIVE_ID:

            /*
             * 先把当前数据保存成上一帧
             */
            memcpy(&ROBOT_CMD_LASTDATA, &ROBOT_CMD_DATA,sizeof(RobotCmdData_t));
            /*
             * 再更新当前数据
             */
            memcpy(&ROBOT_CMD_DATA,frame,sizeof(RobotCmdData_t));
			Feed_Dog(&WatchDog.CMD_DATA_WatchDog);
            break;


        /* =================================================
         * Robot Posture
         * ================================================= */

        case ROBOT_POSTURE_DATA_RECEIVE_ID:
            memcpy(&ROBOT_POSTURE_DATA,frame,sizeof(RobotPostureData_t));
			Feed_Dog(&WatchDog.POSTURE_DATA_WatchDog);
            break;
        default:
            usb_rx_bad_id++;
            return;
    }
    RECEIVE_TIME = HAL_GetTick();
    usb_rx_success++;
}



/* ========================================================
 * 核心解析函数
 * ======================================================== */

static void USB_Parse(void)
{
    while (1)
    {
        /* =================================================
         * 第一步：
         *
         * 连Header都不够
         *
         * 等下一次USB数据
         * ================================================= */
        if (parse_len < sizeof(Frame_Header_t))
        {
            return;
        }
        /* =================================================
         * 第二步：
         *
         * 寻找SOF
         * ================================================= */
        int sof_index = USB_FindSOF();
        /*
         * 整个buffer都没有SOF
         */
        if (sof_index < 0)
        {
            /*
             * 全部都是垃圾数据
             */
            parse_len = 0;
            usb_rx_bad_sof++;
            return;
        }
        /* =================================================
         * 第三步：
         *
         * SOF前面有垃圾数据
         * ================================================= */
        if (sof_index > 0)
        {
            /*
             * 删除SOF前面的垃圾
             */
            USB_Parse_RemoveFront(sof_index);
        }

        /* Removing noise may leave only a partial header. Keep it for next RX. */
        if (parse_len < sizeof(Frame_Header_t))
        {
            return;
        }
        /*
         * 到这里：
         *
         * parse_buffer[0]一定是SOF
         */
        /* =================================================
         * 第四步：
         *
         * 读取Header
         * ================================================= */

        Frame_Header_t header;
        memcpy( &header,parse_buffer,sizeof(Frame_Header_t));
        /* =================================================
         * 第五步：
         *
         * 检查SOF
         * ================================================= */
        if (header.sof != RECEIVE_SOF)
        {
            /*
             * 理论上不会发生
             *
             * 防御性处理
             */
            USB_Parse_RemoveFront(1);
            usb_rx_bad_sof++;
            continue;
        }
        /* =================================================
         * 第六步：
         *
         * 根据ID判断是什么包
         * ================================================= */
        uint16_t expected_frame_len =USB_GetExpectedFrameLength(header.id);
        /*
         * ID不存在
         */
        if (expected_frame_len == 0)
        {
            /*
             * 当前SOF对应的是错误数据
             *
             * 丢掉这个SOF
             *
             * 继续找下一个SOF
             */

            USB_Parse_RemoveFront(1);
            usb_rx_bad_id++;
            continue;
        }
        /* ================================================
         * 第七步：
         *
         * 检查LEN
         * ================================================= */

        /*
         * Header  = 3字节
         * Data    = header.len
         * EOF     = 1字节
         *
         * 总长度：
         *
         * 3 + sizeof(uint32_t) + len + 1
         */
        /* LEN counts data only; the wire frame also contains a 4-byte timestamp. */
        uint16_t actual_frame_len = sizeof(Frame_Header_t) +
                                    sizeof(uint32_t) + header.len + sizeof(uint8_t);


        /*
         * ID对应的结构体长度
         *
         * 必须和LEN算出来的长度一致
         */

        if (actual_frame_len != expected_frame_len)
        {
            /*
             * LEN错误
             */
            USB_Parse_RemoveFront(1);
            usb_rx_bad_len++;
            continue;
        }
        /* =================================================
         * 第八步：
         *
         * 判断完整帧有没有收到
         * ================================================= */
        if (parse_len < actual_frame_len)
        {
            /*
             * 非常重要：
             *
             * 不能丢！
             *
             * 当前帧只是还没收完
             */
            return;
        }
        /* =================================================
         * 第九步：
         *
         * 检查EOF
         * ================================================= */
        if (parse_buffer[actual_frame_len - 1]!= RECEIVE_EOF)
        {
            /*
             * 数据已经收完整了
             *
             * 但是EOF错误
             *
             * 说明这个帧坏了
             */
            USB_Parse_RemoveFront(1);
            usb_rx_bad_eof++;
            continue;
        }


        /* =================================================
         * 第十步：
         *
         * 这是一个完整的正确数据包
         * ================================================= */

        USB_HandleFrame(parse_buffer,actual_frame_len);
        /* =================================================
         * 第十一步：
         *
         * 删除已经处理的完整数据
         * ================================================= */

        USB_Parse_RemoveFront(actual_frame_len);
    }
}


/* ========================================================
 * USB任务调用
 * ======================================================== */

void USB_Rx_Process(void)
{
    USB_RxChunk_t chunk;

    if (usb_rx_queue == NULL)
    {
        return;
    }

    /*
     * 一直把Queue里面已经存在的数据拿出来
     */

    while (xQueueReceive( usb_rx_queue, &chunk,0 ) == pdPASS )
    {
        /*
         * 第一步：
         *
         * 加入连续数据缓存
         */
        USB_Parse_Append(chunk.data,chunk.len);
        /*
         * 第二步：
         *
         * 尝试解析
         */
        USB_Parse();
    }
}
