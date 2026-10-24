#include "calibration.h"
#include "bias_config.h"

#include <math.h>
#include <stddef.h>

static bool is_valid_config(const gyro_cal_cfg_t *config)
{
    return config
        && config->samples >= 2U
        && isfinite(config->duration_s)
        && config->duration_s > 0.0f
        && isfinite(config->max_gap_s)
        && config->max_gap_s > 0.0f
        && isfinite(config->max_gyr_norm)
        && config->max_gyr_norm > 0.0f
        && isfinite(config->max_gyr_sigma)
        && config->max_gyr_sigma > 0.0f
        && isfinite(config->acc_norm_tol_ms2)
        && config->acc_norm_tol_ms2 > 0.0f
        && isfinite(config->max_acc_sigma_ms2)
        && config->max_acc_sigma_ms2 > 0.0f
        && isfinite(config->max_temp_drift)
        && config->max_temp_drift > 0.0f;
}

void gyro_cal_default(gyro_cal_cfg_t *config)
{
    if (!config) {
        return;
    }

    *config = (gyro_cal_cfg_t){
        .samples = CAL_SAMPLES,
        .duration_s = CAL_DURATION_S,
        .max_gap_s = CAL_GAP_S,
        .max_gyr_norm = CAL_GYR_MAX_RS,
        .max_gyr_sigma = GYR_BIAS_SIGMA,
        .acc_norm_tol_ms2 = CAL_ACC_TOL_MS2,
        .max_acc_sigma_ms2 = CAL_ACC_SIGMA_MS2,
        .max_temp_drift = CAL_TEMP_DRIFT_C,
    };
}

static void update_statistics(uint32_t gyr_n,const float sample[CAL_AXES],float mean[CAL_AXES],float m2[CAL_AXES])
{
    for (size_t axis = 0; axis < CAL_AXES; ++axis) {
        const float error_from_mean = sample[axis] - mean[axis];
        mean[axis] += error_from_mean / (float)gyr_n;
        m2[axis] += error_from_mean * (sample[axis] - mean[axis]);
    }
}

static float max_sigma(uint32_t n, const float m2[CAL_AXES])
{
    const float denom = (float)(n - 1U);
    float max_sigma = 0.0f;

    for (size_t axis = 0; axis < CAL_AXES; ++axis) {
        const float variance = m2[axis] / denom;
        max_sigma = fmaxf(max_sigma, sqrtf(fmaxf(variance, 0.0f)));
    }

    return max_sigma;
}

void gyro_cal_init(gyro_cal_t *gyro_cal, const gyro_cal_cfg_t *config)
{
    if (!gyro_cal) {
        return;
    }

    *gyro_cal = (gyro_cal_t){0};
    if (!is_valid_config(config)) {
        gyro_cal->rejected = true;
        return;
    }

    gyro_cal->config = *config;
    gyro_cal->temp_min = INFINITY;
    gyro_cal->temp_max = -INFINITY;
}

cal_status_t gyro_cal_push(gyro_cal_t *gyro_cal, uint32_t time_ms, const float gyr[CAL_AXES],
                           const float accel_ms2[CAL_AXES], bool acc_ready, float temp_c)
{
    if (!gyro_cal || !gyr || (acc_ready && !accel_ms2)
        || !isfinite(gyr[0]) || !isfinite(gyr[1])
        || !isfinite(gyr[2])
        || !isfinite(temp_c) || gyro_cal->rejected) {
        return CAL_INVALID;
    }

    if (acc_ready
        && (!isfinite(accel_ms2[0]) || !isfinite(accel_ms2[1]) || !isfinite(accel_ms2[2]))) {
        return CAL_INVALID;
    }

    if (gyro_cal->has_time) {
        const uint32_t elapsed_ms = time_ms - gyro_cal->last_ms;
        if ((float)elapsed_ms * 0.001f > gyro_cal->config.max_gap_s) {
            gyro_cal->rejected = true;
            return CAL_GAP;
        }
    } else {
        gyro_cal->start_ms = time_ms;
        gyro_cal->has_time = true;
    }
    gyro_cal->last_ms = time_ms;

    const float gyro_norm = sqrtf(gyr[0] * gyr[0]
        + gyr[1] * gyr[1] + gyr[2] * gyr[2]);
    if (gyro_norm > gyro_cal->config.max_gyr_norm) {
        gyro_cal->rejected = true;
        return CAL_GYR_MOTION;
    }

    if (acc_ready) {
        const float accel_norm = sqrtf(accel_ms2[0] * accel_ms2[0]
            + accel_ms2[1] * accel_ms2[1] + accel_ms2[2] * accel_ms2[2]);
        if (fabsf(accel_norm - GRAVITY_MS2) > gyro_cal->config.acc_norm_tol_ms2) {
            gyro_cal->rejected = true;
            return CAL_ACC_MOTION;
        }
    }

    if (gyro_cal->temp_count > 0U) {
        const float next_min_temperature = fminf(gyro_cal->temp_min, temp_c);
        const float next_max_temperature = fmaxf(gyro_cal->temp_max, temp_c);
        if (next_max_temperature - next_min_temperature
            > gyro_cal->config.max_temp_drift) {
            gyro_cal->rejected = true;
            return CAL_TEMP;
        }
    }

    gyro_cal->gyr_n++;
    update_statistics(gyro_cal->gyr_n, gyr, gyro_cal->gyr_mean, gyro_cal->gyr_m2);
    if (acc_ready) {
        gyro_cal->acc_n++;
        update_statistics(gyro_cal->acc_n, accel_ms2, gyro_cal->acc_mean, gyro_cal->acc_m2);
    }

    gyro_cal->temp_count++;
    gyro_cal->temp_mean += (temp_c - gyro_cal->temp_mean)
        / (float)gyro_cal->temp_count;
    gyro_cal->temp_min = fminf(gyro_cal->temp_min, temp_c);
    gyro_cal->temp_max = fmaxf(gyro_cal->temp_max, temp_c);

    return gyro_cal->gyr_n >= gyro_cal->config.samples
        ? CAL_OK : CAL_BUSY;
}

cal_status_t gyro_cal_finish(const gyro_cal_t *gyro_cal, gyro_bias_t *result)
{
    if (!gyro_cal || !result || gyro_cal->rejected
        || gyro_cal->gyr_n < gyro_cal->config.samples
        || gyro_cal->gyr_n < 2U || gyro_cal->acc_n < 2U
        || gyro_cal->temp_count < 2U
        || !gyro_cal->has_time) {
        return CAL_INVALID;
    }

    if ((float)(gyro_cal->last_ms - gyro_cal->start_ms) * 0.001f
        + (1.0f / CAL_RATE_HZ)
        < gyro_cal->config.duration_s) {
        return CAL_INVALID;
    }

    const float accel_sigma = max_sigma(gyro_cal->acc_n, gyro_cal->acc_m2);
    if (accel_sigma > gyro_cal->config.max_acc_sigma_ms2) {
        return CAL_ACC_MOTION;
    }

    const float gyro_sigma = max_sigma(gyro_cal->gyr_n, gyro_cal->gyr_m2);
    if (gyro_sigma > gyro_cal->config.max_gyr_sigma) {
        return CAL_GYR_NOISE;
    }

    *result = (gyro_bias_t){0};
    for (size_t axis = 0; axis < CAL_AXES; ++axis) result->bias[axis] = gyro_cal->gyr_mean[axis];
    result->sigma = gyro_sigma;
    result->ref_temp_c = gyro_cal->temp_mean;

    return CAL_OK;
}
