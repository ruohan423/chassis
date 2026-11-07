#ifndef VQF_BIAS_CFG_H
#define VQF_BIAS_CFG_H

/* 使用静止校准结果替换这些值，单位为 rad/s。 */
#define GYR_BIAS_X     0.00335047115f   /* 不执行启动校准时使用的陀螺仪 X 轴零偏，单位 rad/s */
#define GYR_BIAS_Y     -0.00236658799f  /* 不执行启动校准时使用的陀螺仪 Y 轴零偏，单位 rad/s*/
#define GYR_BIAS_Z     0.000909629394f  /* 不执行启动校准时使用的陀螺仪 Z 轴零偏，单位 rad/s*/
#define GYR_BIAS_SIGMA 0.0175000001f    /* 固定零偏先验的标准差，同时作为静止校准三轴最大标准差上限，单位 rad/s*/
#define GYRO_SIGMA_MIN 0.0175f          /* 传入姿态滤波器的零偏标准差下限，单位 rad/s，须为正 */

/* 需要启动静止校准时，由板级或遥控启动模式使用该配置。 */
#define CAL_ON_STARTUP 0U  /* 启动静止校准开关：0 使用保存的固定零偏，非零执行启动校准 */

/* VQF 采样周期，单位为秒。 */
#define VQF_GYR_DT 0.001f   /* 姿态滤波器的陀螺采样周期，单位 s，当前值对应 1000 Hz。 */
#define VQF_ACC_DT 0.0025f  /* 姿态滤波器的加速度采样周期，单位 s，当前值对应 400 Hz。 */

/* IMU 加热器配置。 */
#define TEMP_TARGET_C               36.0f    /* 加热目标及预热稳定判断的目标温度，单位 °C */
#define TEMP_LIMIT_C                60.0f    /* 加热安全温度上限，单位 °C*/
#define HEATER_CONTROL_PERIOD_S     0.25f    /* 温控闭环的最小更新间隔，单位 s */
#define TEMP_OBS_PERIOD_S           1.28f    /* 预热阶段温度观测反馈的最小间隔，单位 s */
#define TEMP_TOL_C                  0.5f     /* 预热温度相对目标和已建立参考温度的允许偏差，单位 °C */
#define TEMP_STABLE_N               5U       /* 每阶段需要的连续合格温度观测次数 */
#define HEATER_INTEGRAL_LIMIT_RATIO 0.5f     /* 温控积分项限幅占输出上限的非负比例，无量纲 */
#define HEATER_OUT_MAX              1000.0f  /* 温控器输出的正幅值上限，单位为归一化控制量 */
#define HEATER_KP                   300.0f   /* 温差比例增益，单位为控制量/°C */
#define HEATER_KI                   20.0f    /* 温差积分增益，单位为控制量/(°C·s) */
#define HEATER_KD                   0.0f     /* 温差微分增益，单位为控制量·s/°C*/

/* 静止校准默认配置，加速度限制使用国际单位制。 */
#define CAL_DURATION_S    10.0f     /* 静止校准采集窗口的最短时长，单位 s*/
#define CAL_RATE_HZ       1000.0f   /* 校准器期望的有效陀螺采样率，单位 Hz*/
#define CAL_SAMPLES       10000U    /* 完成静止统计所需的陀螺样本数*/
#define CAL_WARMUP_S      180.0f    /* 预热和温度稳定确认阶段的最长允许时间*/
#define CAL_GAP_S         0.02f     /* 校准会话相邻推送的最大允许间隔，单位 s*/
#define CAL_TEMP_DRIFT_C  0.5f      /* 静止采样窗口内温度最高值与最低值的最大允许差，单位 °C */
#define CAL_ACC_TOL_MS2   0.5f      /* 单个有效加速度样本模长偏离重力参考的最大容差，单位 m/s²*/
#define CAL_ACC_SIGMA_MS2 0.15f     /* 采集结束后三轴加速度样本标准差中最大值的上限，单位 m/s²*/
#define CAL_GYR_MAX_RS    0.05f     /* 三轴角速度向量模长的上限，单位 rad/s*/
#define GRAVITY_MS2       9.80665f  /* 静止加速度模长的重力参考值，单位 m/s²*/

#endif
