#ifndef VQF_CALIBRATION_SESSION_H
#define VQF_CALIBRATION_SESSION_H

#include "calibration.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAL_SESSION_IDLE = 0,
    CAL_SESSION_WARMUP,
    CAL_SESSION_SAMPLE,
    CAL_SESSION_DONE,
    CAL_SESSION_FAIL
} cal_session_state_t;

/* 完整校准会话前置和执行期间使用的状态。 */
typedef struct
{
    float target_c;
    float temp_tol_c;
    uint32_t stable_n;
    float obs_period_s;
    float warmup_max_s;
    float max_gap_s;
    gyro_cal_cfg_t stationary;
} cal_session_cfg_t;

/* 预热阶段和静止采集阶段的运行时状态。 */
typedef struct
{
    cal_session_cfg_t config;
    gyro_cal_t gyro_cal;
    cal_session_state_t state;

    uint32_t stable_n;
    uint32_t start_ms;
    uint32_t last_ms;
    uint32_t last_obs_ms;
    float ref_temp_c;
    bool has_ref_temp;
    bool has_obs;
    bool has_last;

    gyro_bias_t result;
} cal_session_t;

/* config 为调用方持有的输出对象，仅在调用期间写入且不保存指针。 */
void cal_session_default(cal_session_cfg_t *config);

/* session 为调用方持有的可写状态；config 仅在调用期间只读借用并复制，函数不会保存指针。 */
void cal_session_init(cal_session_t *session, const cal_session_cfg_t *config);

/* session 必须保持有效到会话结束，函数会修改其内部状态但不会保存额外指针。 */
void cal_session_start(cal_session_t *session);

/* 向当前会话输入一个国际单位制 IMU 样本；输入数组仅在调用期间借用，不会被保存或修改。 */
cal_status_t cal_session_push(cal_session_t *session, uint32_t time_ms, const float *gyr,
                              const float *acc, bool acc_ready, float temp_c);

/* session 仅在调用期间只读借用。 */
cal_session_state_t cal_session_state(const cal_session_t *session);

/* 返回 session 内部只读结果，不转移所有权；仅在 cal_session_push 返回 CAL_OK 后且 session 有效期间可用。 */
const gyro_bias_t *cal_session_result(const cal_session_t *session);

#endif
