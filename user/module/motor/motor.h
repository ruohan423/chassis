#ifndef SENTRY_CHASSIS_MOTOR_H
#define SENTRY_CHASSIS_MOTOR_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RM3508_LIMIT 16384
#define GM6020_LIMIT 30000
#define RM3510_LIMIT 32767
#define GM3510_LIMIT 29000
#define M2006_LIMIT 10000
#define RM6623_LIMIT 32767
#define DM4310_LIMIT 50


typedef struct
{
    uint16_t MchanicalAngle;
    int16_t Speed;
    int16_t TorqueCurrent;
    uint8_t temp;
    float Power;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    uint8_t flag;
} RM3508_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t Speed;
    int16_t TorqueCurrent;
    uint8_t temp;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    uint8_t flag;
} GM6020_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t Speed;
} RM3510_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t OutputTorque;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    uint8_t flag;
} GM3510_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t Speed;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    uint8_t flag;
} M2006_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t TorqueCurrent;
    int16_t SetTorqueCurrent;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    uint8_t flag;
} RM6623_TypeDef;

typedef struct
{
    uint16_t MchanicalAngle;
    int16_t Speed;
    int16_t TorqueCurrent;
    uint8_t temp;
    uint8_t PCBtemp;
    uint16_t LsatAngle;
    int16_t r;
    int32_t Angle;
    float Angle_DEG;
    float Speed_DEG;
    float MchanicalAngle_DEG;
    uint8_t flag;
} DM4310_TypeDef;


//C620电调错误码枚举
typedef enum
{
    C620_err_none=0,//无错误
    C620_err_storage=1,//存储错误
    C620_err_voltage=2,//电压错误
    C620_err_phase=3,//相位错误
    C620_err_sensor=4,//传感器错误
    C620_err_temp=5,//温度错误
    C620_err_calib=7,//校准错误
    C620_err_overhot=8,//过温错误
   }C620_err_t;

void RM6623_Receive(RM6623_TypeDef *Dst, const uint8_t *Data);
void RM3510_Receive(RM3510_TypeDef *Dst, const uint8_t *Data);
void RM3508_Receive(RM3508_TypeDef *Dst, const uint8_t *Data);
void GM6020_Receive(GM6020_TypeDef *Dst, const uint8_t *Data);
void M2006_Receive(M2006_TypeDef *Dst, const uint8_t *Data);
void GM3510_Receive(GM3510_TypeDef *Dst, const uint8_t *Data);
void DM4310_Receive(DM4310_TypeDef *Dst, const uint8_t *Data);
int16_t QuickCentering(uint16_t Mch, uint16_t Exp);

#ifdef HAL_CAN_MODULE_ENABLED
HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data);
#endif

#ifdef __cplusplus
}
#endif

#endif
