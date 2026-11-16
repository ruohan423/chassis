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

/* frame 指向接收回调栈上的只读帧，仅在本次回调期间有效，回调不得保存该指针。 */
typedef void (*user_can_rx_callback_t)(CAN_HandleTypeDef *handle, const user_can_frame_t *frame);

/* handle 由 HAL/CubeMX 持有并在 CAN 使用期间持续有效；注册的 callback 会由模块保存，直到被替换。 */
bool user_can_start(CAN_HandleTypeDef *handle);
bool user_can_stop(CAN_HandleTypeDef *handle);
bool user_can_is_started(const CAN_HandleTypeDef *handle);
void user_can_register_rx_callback(CAN_HandleTypeDef *handle, user_can_rx_callback_t callback);
/* data 为调用方持有的只读载荷，length 不得超过 USER_CAN_DATA_LENGTH；函数返回前已复制并提交到 CAN 邮箱。 */
HAL_StatusTypeDef user_can_send(CAN_HandleTypeDef *handle, uint32_t standard_id,
                                const uint8_t *data, uint8_t length);

#endif
