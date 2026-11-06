#ifndef SENTRY_CHASSIS_USER_CAN_H
#define SENTRY_CHASSIS_USER_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#define USER_CAN_DATA_LENGTH 8U

typedef struct
{
    uint32_t id;
    uint8_t data[USER_CAN_DATA_LENGTH];
    uint8_t length;
    bool extended;
} user_can_frame_t;

typedef void (*user_can_rx_callback_t)(CAN_HandleTypeDef *handle, const user_can_frame_t *frame);

bool user_can_start(CAN_HandleTypeDef *handle);
bool user_can_stop(CAN_HandleTypeDef *handle);
bool user_can_is_started(const CAN_HandleTypeDef *handle);
void user_can_register_rx_callback(CAN_HandleTypeDef *handle, user_can_rx_callback_t callback);
HAL_StatusTypeDef user_can_send(CAN_HandleTypeDef *handle, uint32_t standard_id,
                                const uint8_t *data, uint8_t length);

#endif
