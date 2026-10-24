#include "calibration_session.h"
#include "bias_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool is_valid_config(const cal_session_cfg_t *config)
{
    return config
        && isfinite(config->target_c)
        && isfinite(config->temp_tol_c)
        && config->temp_tol_c > 0.0f
        && config->stable_n > 0U
        && isfinite(config->obs_period_s)
        && config->obs_period_s > 0.0f
        && isfinite(config->warmup_max_s)
        && config->warmup_max_s > 0.0f
        && isfinite(config->max_gap_s)
        && config->max_gap_s > 0.0f
        && config->stationary.samples >= 2U
        && isfinite(config->stationary.max_gap_s)
        && config->stationary.max_gap_s > 0.0f;
}

void cal_session_default(cal_session_cfg_t *config)
{
    if (!config) {
        return;
    }

    *config = (cal_session_cfg_t){.target_c = TEMP_TARGET_C, .temp_tol_c = TEMP_TOL_C,
        .stable_n = TEMP_STABLE_N, .obs_period_s = TEMP_OBS_PERIOD_S,
        .warmup_max_s = CAL_WARMUP_S, .max_gap_s = CAL_GAP_S};
    gyro_cal_default(&config->stationary);
}

void cal_session_init(cal_session_t *session, const cal_session_cfg_t *config)
{
    if (!session) {
        return;
    }

    memset(session, 0, sizeof(*session));
    session->state = CAL_SESSION_FAIL;
    if (!is_valid_config(config)) {
        return;
    }

    session->config = *config;
    session->state = CAL_SESSION_IDLE;
}

bool cal_session_start(cal_session_t *session)
{
    if (!session || session->state == CAL_SESSION_FAIL) {
        return false;
    }

    gyro_cal_init(&session->gyro_cal, &session->config.stationary);
    session->state = CAL_SESSION_WARMUP;
    session->stable_n = 0U;
    session->start_ms = 0U;
    session->last_obs_ms = 0U;
    session->ref_temp_c = 0.0f;
    session->has_ref_temp = false;
    session->has_obs = false;
    session->has_last = false;

    if (session->gyro_cal.rejected) {
        session->state = CAL_SESSION_FAIL;
        return false;
    }

    return true;
}

void cal_session_cancel(cal_session_t *session)
{
    if (!session) {
        return;
    }

    session->state = CAL_SESSION_IDLE;
    session->stable_n = 0U;
    session->has_ref_temp = false;
    session->has_obs = false;
    session->has_last = false;
}

cal_status_t cal_session_push(
    cal_session_t *session,
    uint32_t time_ms,
    const float gyr[CAL_AXES],
    const float accel_ms2[CAL_AXES],
    bool acc_ready,
    float temp_c)
{
    if (!session || !gyr || (acc_ready && !accel_ms2)
        || !isfinite(gyr[0]) || !isfinite(gyr[1])
        || !isfinite(gyr[2])
        || !isfinite(temp_c)) {
        return CAL_INVALID;
    }
    if (acc_ready && (!isfinite(accel_ms2[0]) || !isfinite(accel_ms2[1]) || !isfinite(accel_ms2[2]))) {
        return CAL_INVALID;
    }

    if (session->state == CAL_SESSION_WARMUP) {
        if (session->has_last) {
            const uint32_t elapsed_ms = time_ms - session->last_ms;
            if ((float)elapsed_ms * 0.001f > session->config.max_gap_s) {
                session->state = CAL_SESSION_FAIL;
                return CAL_GAP;
            }
        }

        session->last_ms = time_ms;
        session->has_last = true;

        if (!session->has_obs) {
            session->start_ms = time_ms;
            session->last_obs_ms = time_ms;
            session->has_obs = true;
        }

        if ((float)(time_ms - session->start_ms) * 0.001f > session->config.warmup_max_s) {
            session->state = CAL_SESSION_FAIL;
            return CAL_TEMP;
        }

        if ((float)(time_ms - session->last_obs_ms) * 0.001f < session->config.obs_period_s) {
            return CAL_BUSY;
        }
        session->last_obs_ms = time_ms;

        const float temp_err = fabsf(temp_c - session->config.target_c);
        if (temp_err <= session->config.temp_tol_c) {
            session->stable_n++;
        } else {
            session->stable_n = 0U;
        }

        if (session->has_ref_temp && fabsf(temp_c - session->ref_temp_c) > session->config.temp_tol_c) {
            session->stable_n = 0U;
        }
        if (session->stable_n >= session->config.stable_n) {
            if (!session->has_ref_temp) {
                session->ref_temp_c = temp_c;
                session->has_ref_temp = true;
                session->stable_n = 0U;
                return CAL_BUSY;
            }
            gyro_cal_init(&session->gyro_cal, &session->config.stationary);
            session->state = CAL_SESSION_SAMPLE;
            session->has_last = false;
        }

        return CAL_BUSY;
    }

    if (session->state != CAL_SESSION_SAMPLE) {
        return CAL_INVALID;
    }

    if (session->has_last) {
        const uint32_t elapsed_ms = time_ms - session->last_ms;
        if ((float)elapsed_ms * 0.001f > session->config.max_gap_s) {
            session->state = CAL_SESSION_FAIL;
            return CAL_GAP;
        }
    }
    session->last_ms = time_ms;
    session->has_last = true;

    const cal_status_t status = gyro_cal_push(&session->gyro_cal, time_ms, gyr, accel_ms2, acc_ready, temp_c);
    if (status >= CAL_GYR_MOTION) {
        session->state = CAL_SESSION_FAIL;
        return status;
    }

    if (status == CAL_OK) {
        const cal_status_t finish_status = gyro_cal_finish(&session->gyro_cal, &session->result);
        session->state = finish_status == CAL_OK
            ? CAL_SESSION_DONE : CAL_SESSION_FAIL;
        return finish_status;
    }

    return status;
}

cal_session_state_t cal_session_state(const cal_session_t *session)
{
    return session ? session->state : CAL_SESSION_FAIL;
}

const gyro_bias_t *cal_session_result(const cal_session_t *session)
{
    if (!session || session->state != CAL_SESSION_DONE) return NULL;

    return &session->result;
}
