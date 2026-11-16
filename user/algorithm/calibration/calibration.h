#ifndef VQF_CALIBRATION_H
#define VQF_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define CAL_AXES 3U

/* BMI088 传感器坐标系下测得的静止陀螺仪零偏。 */
typedef struct
{
    float bias[CAL_AXES];
    float sigma;
    float ref_temp_c;
} gyro_bias_t;

/* 单次静止校准窗口的质量限制。 */
typedef struct
{
    uint32_t samples;
    float duration_s;
    float max_gap_s;
    float max_gyr_norm;
    float max_gyr_sigma;
    float acc_norm_tol_ms2;
    float max_acc_sigma_ms2;
    float max_temp_drift;
} gyro_cal_cfg_t;

/* config 为调用方持有的输出对象，仅在调用期间写入且不保存指针。 */
void gyro_cal_default(gyro_cal_cfg_t *config);

typedef enum
{
    CAL_OK = 0,
    CAL_BUSY,
    CAL_GYR_MOTION,
    CAL_GYR_NOISE,
    CAL_ACC_MOTION,
    CAL_TEMP,
    CAL_GAP,
    CAL_INVALID
} cal_status_t;

/* 在线 Welford 统计计算使用的累加状态。 */
typedef struct
{
    gyro_cal_cfg_t config;

    uint32_t gyr_n;
    uint32_t acc_n;
    float gyr_mean[CAL_AXES];
    float gyr_m2[CAL_AXES];
    float acc_mean[CAL_AXES];
    float acc_m2[CAL_AXES];

    float temp_mean;
    float temp_min;
    float temp_max;
    uint32_t temp_count;

    uint32_t start_ms;
    uint32_t last_ms;
    bool has_time;
} gyro_cal_t;

/* cal 为调用方持有的可写会话状态；config 仅在调用期间只读借用并复制，函数不会保存指针。 */
void gyro_cal_init(gyro_cal_t *cal, const gyro_cal_cfg_t *config);

/* 向校准窗口加入一个国际单位制 IMU 样本；输入数组仅在调用期间借用，不会被保存或修改。 */
cal_status_t gyro_cal_push(gyro_cal_t *cal, uint32_t time_ms, const float *gyr,
                           const float *acc, bool acc_ready, float temp_c);

/* cal 仅在调用期间只读借用；result 为调用方输出对象，仅在成功时写入且不保存指针。 */
cal_status_t gyro_cal_finish(const gyro_cal_t *cal, gyro_bias_t *result);

#endif
