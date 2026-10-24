#ifndef VQF_BIAS_CFG_H
#define VQF_BIAS_CFG_H

/* 使用静止校准结果替换这些值，单位为 rad/s。 */
#define GYR_BIAS_X 0.00335047115f
#define GYR_BIAS_Y -0.00236658799f
#define GYR_BIAS_Z 0.000909629394f
#define GYR_BIAS_SIGMA 0.0175000001f
#define GYRO_SIGMA_MIN 0.0175f

/* 需要启动静止校准时，由板级或遥控启动模式使用该配置。 */
#define CAL_ON_STARTUP 0U

/* VQF 采样周期，单位为秒。 */
#define VQF_GYR_DT 0.001f
#define VQF_ACC_DT 0.0025f

/* IMU 加热器配置。 */
#define TEMP_TARGET_C 36.0f
#define TEMP_LIMIT_C 60.0f
#define HEATER_CONTROL_PERIOD_S 0.25f
#define TEMP_OBS_PERIOD_S 1.28f
#define TEMP_TOL_C 0.5f
#define TEMP_STABLE_N 5U
#define HEATER_INTEGRAL_LIMIT_RATIO 0.5f
#define HEATER_OUT_MAX 1000.0f
#define HEATER_KP 300.0f
#define HEATER_KI 20.0f
#define HEATER_KD 0.0f

/* 静止校准默认配置，加速度限制使用国际单位制。 */
#define CAL_DURATION_S 10.0f
#define CAL_RATE_HZ 1000.0f
#define CAL_SAMPLES 10000U
#define CAL_WARMUP_S 180.0f
#define CAL_GAP_S 0.02f
#define CAL_TEMP_DRIFT_C 0.5f
#define CAL_ACC_TOL_MS2 0.5f
#define CAL_ACC_SIGMA_MS2 0.15f
#define CAL_GYR_MAX_RS 0.05f
#define GRAVITY_MS2 9.80665f

#endif
