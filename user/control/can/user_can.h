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
/* 启动或恢复总线：总线可用时直接返回 true；否则按 handle->Init 重新初始化控制器、配置过滤器并启动，
 * 用于首次启动、启动失败后的重试和 bus-off 后的软件恢复。失败返回 false，由调用任务延时后重试。
 * 仅由拥有该总线的任务调用；HAL 进入、退出初始化模式各有 10 ms 忙等超时，总线被拉死时单次调用最长约 20 ms。 */
bool user_can_start(CAN_HandleTypeDef *handle);
bool user_can_stop(CAN_HandleTypeDef *handle);
/* 总线可用：控制器已启动且不处于 bus-off。bus-off 期间控制器不收不发，调用方应停止发送并用 user_can_start 恢复。 */
bool user_can_is_online(const CAN_HandleTypeDef *handle);
void user_can_register_rx_callback(CAN_HandleTypeDef *handle, user_can_rx_callback_t callback);
/* data 为调用方持有的只读载荷，length 不得超过 USER_CAN_DATA_LENGTH；函数返回前已复制并提交到 CAN 邮箱。
 * 总线未启动或处于 bus-off 时返回 HAL_ERROR，邮箱已满时返回 HAL_BUSY，两种情况都不入邮箱。 */
HAL_StatusTypeDef user_can_send(CAN_HandleTypeDef *handle, uint32_t standard_id,
                                const uint8_t *data, uint8_t length);

#endif
