#include "motor.h"

static int16_t motor_angle_difference(uint16_t angle, uint16_t last_angle)
{
    return (int16_t)(angle - last_angle);
}

void RM6623_Receive(RM6623_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->TorqueCurrent = (int16_t)((Data[2] << 8) | Data[3]);
    Dst->SetTorqueCurrent = (int16_t)((Data[4] << 8) | Data[5]);
}

void RM3510_Receive(RM3510_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->Speed = (int16_t)((Data[2] << 8) | Data[3]);
}

void RM3508_Receive(RM3508_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->Speed = (int16_t)((Data[2] << 8) | Data[3]);
    Dst->TorqueCurrent = (int16_t)((Data[4] << 8) | Data[5]);
    Dst->temp = Data[6];

    int16_t diff = motor_angle_difference(Dst->MchanicalAngle, Dst->LsatAngle);
    if (diff != (int16_t)Dst->MchanicalAngle)
        Dst->flag = 1U;
    if (Dst->flag == 1U)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->Power = 0.0f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

void GM3510_Receive(GM3510_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->OutputTorque = (int16_t)((Data[2] << 8) | Data[3]);

    int16_t diff = motor_angle_difference(Dst->MchanicalAngle, Dst->LsatAngle);
    if (diff != (int16_t)Dst->MchanicalAngle)
        Dst->flag = 1U;
    if (Dst->flag == 1U)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

void GM6020_Receive(GM6020_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->Speed = (int16_t)((Data[2] << 8) | Data[3]);
    Dst->TorqueCurrent = (int16_t)((Data[4] << 8) | Data[5]);
    Dst->temp = Data[6];

    int16_t diff = motor_angle_difference(Dst->MchanicalAngle, Dst->LsatAngle);
    if (diff != (int16_t)Dst->MchanicalAngle)
        Dst->flag = 1U;
    if (Dst->flag == 1U)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

void M2006_Receive(M2006_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->Speed = (int16_t)((Data[2] << 8) | Data[3]);

    int16_t diff = motor_angle_difference(Dst->MchanicalAngle, Dst->LsatAngle);
    if (diff != (int16_t)Dst->MchanicalAngle)
        Dst->flag = 1U;
    if (Dst->flag == 1U)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

void DM4310_Receive(DM4310_TypeDef *Dst, const uint8_t *Data)
{
    Dst->MchanicalAngle = (uint16_t)((Data[0] << 8) | Data[1]);
    Dst->Speed = (int16_t)((Data[2] << 8) | Data[3]) / 100;
    Dst->TorqueCurrent = (int16_t)((Data[4] << 8) | Data[5]) / 1000;
    Dst->temp = Data[6];
    Dst->PCBtemp = Data[7];

    int16_t diff = motor_angle_difference(Dst->MchanicalAngle, Dst->LsatAngle);
    if (diff != (int16_t)Dst->MchanicalAngle)
        Dst->flag = 1U;
    if (Dst->flag == 1U)
    {
        if (diff > 4000)
            Dst->r--;
        if (diff < -4000)
            Dst->r++;
    }
    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
    Dst->Speed_DEG = Dst->Speed * 0.106f;
    Dst->MchanicalAngle_DEG = Dst->MchanicalAngle * 0.0439453125f;
    Dst->LsatAngle = Dst->MchanicalAngle;
}

#ifdef HAL_CAN_MODULE_ENABLED
HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data)
{
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = StdId,
        .ExtId = 0U,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8U,
        .TransmitGlobalTime = DISABLE,
    };

    if (hcan == NULL || Data == NULL || StdId > 0x7FFU){return HAL_ERROR;}
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U){return HAL_BUSY;}

    for (uint8_t index = 0U; index < 4U; index++)
    {
        tx_data[index * 2U] = (uint8_t)(Data[index] >> 8);
        tx_data[index * 2U + 1U] = (uint8_t)Data[index];
    }

    return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}
#endif

int16_t QuickCentering(uint16_t Mch, uint16_t Exp)
{
    uint16_t Exp_ = (uint16_t)((Exp + 4095U) % 8192U);
    if (Exp_ < Exp)
        return Mch < Exp_ ? (int16_t)(Exp - 8192) : (int16_t)Exp;
    return Mch > Exp_ ? (int16_t)(Exp + 8192) : (int16_t)Exp;
}
