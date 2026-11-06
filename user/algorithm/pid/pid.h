#ifndef USER_PID_H
#define USER_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float sample_period_s;
    float error;
    float previous_error;
    float integral;
    float output;
} pid_position_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;
    float integral_limit;
    float sample_period_s;
    float error;
    float previous_error;
    float integral;
    float previous_reference;
    float output;
} pid_ffb_t;


typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float sample_period_s;
    float error;
    float integral;
    float previous_feedback;
    float output;
} pid_dom_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float sample_period_s;
    float error;
    float previous_error;
    float previous_previous_error;
    float increment;
} pid_increment_t;

/* sample_period_s 单位为秒；ki 作用于误差积分，kd 作用于每秒误差变化率。 */
void pid_position_init(pid_position_t *pid, float kp, float ki, float kd, float integral_limit,
                       float sample_period_s);
float pid_position_calculate(pid_position_t *pid, float feedback, float reference);

/* 前馈项为 kf * d(reference)/dt，kf 不隐含控制周期。 */
void pid_ffb_init(pid_ffb_t *pid, float kp, float ki, float kd, float kf, float integral_limit,
                  float sample_period_s);
float pid_ffb_calculate(pid_ffb_t *pid, float feedback, float reference);

/* 微分项使用测量值变化率，避免设定值阶跃引起微分冲击。 */
void pid_dom_init(pid_dom_t *pid, float kp, float ki, float kd, float integral_limit,
                  float sample_period_s);
float pid_dom_calculate(pid_dom_t *pid, float feedback, float reference);

void pid_increment_init(pid_increment_t *pid, float kp, float ki, float kd,
                        float sample_period_s);
float pid_increment_calculate(pid_increment_t *pid, float feedback, float reference);

float float_constrain(float value, float min_value, float max_value);
float float_rounding(float value);

#endif
