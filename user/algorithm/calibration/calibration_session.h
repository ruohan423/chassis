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

/* 填充 BMI088/Rust 兼容的预热和静止校准默认配置。 */
void cal_session_default(cal_session_cfg_t *config);

/* 初始化会话。 */
void cal_session_init(cal_session_t *session, const cal_session_cfg_t *config);

/* 启动温度预热和静止采集流程。 */
void cal_session_start(cal_session_t *session);

/* 向当前会话输入一个国际单位制 IMU 样本。 */
cal_status_t cal_session_push(cal_session_t *session, uint32_t time_ms, const float gyr[CAL_AXES],
                              const float acc[CAL_AXES], bool acc_ready, float temp_c);

/* 返回当前会话状态。 */
cal_session_state_t cal_session_state(const cal_session_t *session);

/* 返回校准结果，仅在 cal_session_push 返回 CAL_OK 后有效。 */
const gyro_bias_t *cal_session_result(const cal_session_t *session);

#endif
