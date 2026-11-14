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

/* PidCalculator：输入为误差，积分按控制周期累加，微分取相邻周期误差之差，都不含采样时间。
 * ki、kd 因此与控制周期绑定，周期变化后须重新换算；限幅字段初始化为 ±FLT_MAX 表示不限幅，
 * 积分限幅与积分分离阈值由调用者按需改写。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_min;       /* 误差累加和下限 */
    float integral_max;       /* 误差累加和上限 */
    float integral_split_min; /* 误差低于该值时清零积分且本周期不计积分项 */
    float integral_split_max; /* 误差高于该值时清零积分且本周期不计积分项 */
    float output_min;         /* 输出下限 */
    float output_max;         /* 输出上限 */
    float last_error;         /* 上次误差；NaN 表示复位后尚无上次误差，本周期不计微分 */
    float integral;           /* 误差累加和 */
} pid_calculator_t;

/* 初始化增益并复位积分与上次误差，限幅字段置为不限幅。 */
void pid_calculator_init(pid_calculator_t *pid, float kp, float ki, float kd);
/* 只复位积分与上次误差，保留增益和限幅。 */
void pid_calculator_reset(pid_calculator_t *pid);
/* 误差非有限值时返回 NaN 且不改变内部状态。 */
float pid_calculator_update(pid_calculator_t *pid, float error);

float float_constrain(float value, float min_value, float max_value);
float float_rounding(float value);

#endif
