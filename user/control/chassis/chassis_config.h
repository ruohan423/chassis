#ifndef SENTRY_CHASSIS_CHASSIS_CONFIG_H
#define SENTRY_CHASSIS_CHASSIS_CONFIG_H

/* 全向轮自动导航哨兵底盘，控制链路移植自 RMCS-main 的
 * ChassisController、ChassisPowerController 和 OmniWheelController。
 * 上车前逐项核对：接线/方向 → 几何/质量 → 速度环 → 功率模型。 */

/* 底层适配：逻辑轮序与 RMCS 一致，固定为左前、左后、右后、右前。
 * 四个 ID 必须属于同一个四电机控制组；方向同步作用于反馈速度和输出指令。 */
#define CHASSIS_CAN_BUS                   2U      /* 底盘电机使用的总线编号，1 为 CAN1、2 为 CAN2 */
#define CHASSIS_MOTOR_COUNT               4U      /* 底盘轮电机数量，控制器按四轮实现，不能单独修改 */
#define CHASSIS_MOTOR_GROUP_FIRST_ID      1U      /* 四电机控制组的起始电机编号；当前组从 1 开始 */
#define CHASSIS_MOTOR_COMMAND_ID          0x200U  /* 四轮电流指令的标准帧标识 */
#define CHASSIS_MOTOR_FEEDBACK_ID_BASE    0x201U  /* 控制组首个电机的反馈标准帧标识 */
#define CHASSIS_LF_MOTOR_ID               2U      /* 逻辑左前轮的实际电机编号，须按实车接线核对 */
#define CHASSIS_LB_MOTOR_ID               3U      /* 逻辑左后轮的实际电机编号，须按实车接线核对 */
#define CHASSIS_RB_MOTOR_ID               4U      /* 逻辑右后轮的实际电机编号，须按实车接线核对 */
#define CHASSIS_RF_MOTOR_ID               1U      /* 逻辑右前轮的实际电机编号，须按实车接线核对 */
#define CHASSIS_LF_MOTOR_DIRECTION        1.0f    /* 左前轮安装方向系数，只取 +1 或 -1，待实车核对 */
#define CHASSIS_LB_MOTOR_DIRECTION        1.0f    /* 左后轮安装方向系数，只取 +1 或 -1，待实车核对 */
#define CHASSIS_RB_MOTOR_DIRECTION        1.0f    /* 右后轮安装方向系数，只取 +1 或 -1，待实车核对 */
#define CHASSIS_RF_MOTOR_DIRECTION        1.0f    /* 右前轮安装方向系数，只取 +1 或 -1，待实车核对 */
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 40U     /* 电机反馈在线判据的超时时间，单位 ms；任一轮离线时四轮零输出 */

/* 电机：四个同型号、同减速比的 M3508 + C620。
 * C620 原始指令 ±16384 对应 ±20 A，需核对电调协议与实际限流。 */
#define CHASSIS_MOTOR_KA_NM_PER_A          0.0156f   /* 转子侧力矩常数，单位 Nm/A，不含减速比 */
#define CHASSIS_MOTOR_REDUCTION_RATIO      19.0f     /* 电机侧转速与轮侧转速之比，须按实际减速箱核对 */
#define CHASSIS_MOTOR_CURRENT_FULL_SCALE_A 20.0f     /* 电调协议满量程对应的电流幅值，单位 A */
#define CHASSIS_MOTOR_COMMAND_FULL_SCALE   16384.0f  /* 电调协议的电流指令满量程幅值，单位为原始指令值 */
#define CHASSIS_MOTOR_CURRENT_LIMIT_A      20.0f     /* 本工程允许的单轮电流幅值上限，单位 A，不大于协议满量程 */

/* 原始输出限幅，由允许电流除以协议满量程电流再乘指令满量程派生，不直接修改。 */
#define CHASSIS_MOTOR_OUTPUT_LIMIT (CHASSIS_MOTOR_CURRENT_LIMIT_A / CHASSIS_MOTOR_CURRENT_FULL_SCALE_A * CHASSIS_MOTOR_COMMAND_FULL_SCALE)
/* 每 Nm 轮侧力矩对应的原始指令，由力矩常数、减速比和协议刻度派生，不直接修改。 */
#define CHASSIS_MOTOR_RAW_PER_NM   (CHASSIS_MOTOR_COMMAND_FULL_SCALE / (CHASSIS_MOTOR_CURRENT_FULL_SCALE_A * CHASSIS_MOTOR_KA_NM_PER_A * CHASSIS_MOTOR_REDUCTION_RATIO))

/* 机械：四轮 45° 对称布局全向轮。坐标系与 RMCS 一致：x 向前、y 向左，w 逆时针为正。
 * 质量、转动惯量和摩擦系数为 RMCS 示例值，不是本车实测结果。 */
#define CHASSIS_WHEEL_RADIUS_M       0.050f  /* 车轮有效滚动半径，单位 m */
#define CHASSIS_RADIUS_X_M           0.200f  /* 轮接地点到车体中心的前后向距离，单位 m */
#define CHASSIS_RADIUS_Y_M           0.166f  /* 轮接地点到车体中心的左右向距离，单位 m */
#define CHASSIS_MASS_KG              22.0f   /* 整车质量，单位 kg；RMCS 示例值，须实车称量 */
#define CHASSIS_INERTIA_KGM2         4.08f   /* 整车绕竖直轴的转动惯量，单位 kg·m²；RMCS 示例值，待标定 */
#define CHASSIS_FRICTION_COEFFICIENT 0.6f    /* 轮地摩擦系数，无量纲，决定单轮防滑力矩上限；RMCS 示例值，待实测 */
#define CHASSIS_GRAVITY_MPS2         9.81f   /* 重力加速度，单位 m/s² */

/* 运动约束：平移按合速度模长限幅，自转按模式取值并限幅。 */
#define CHASSIS_MAX_TRANSLATION_MPS 2.2f  /* 平移目标合速度上限，单位 m/s；导航指令与摇杆叠加后限幅，摇杆满偏对应该速度 */
#define CHASSIS_MAX_ANGULAR_RAD_S   8.0f  /* 目标自转角速度幅值上限，单位 rad/s；同时是对齐与跟随角度环的输出限幅 */
#define CHASSIS_SPIN_FAST_RATIO     0.6f  /* 快速小陀螺目标自转占角速度上限的比例，范围 0～1；取自 RMCS SPIN_FAST */
#define CHASSIS_SPIN_SLOW_RATIO     0.3f  /* 慢速小陀螺目标自转占角速度上限的比例，范围 0～1；取自 RMCS SPIN_SLOW */

/* 小陀螺卡死换向：实测 yaw 角速度低于目标自转的一定比例并持续一段时间后反转自转方向。
 * 时间取自 RMCS 在 1 kHz 更新率下的周期计数，由控制层按实际控制周期折算为周期数。 */
#define CHASSIS_SPIN_STUCK_SPEED_RATIO  0.2f  /* 判为卡死的实测角速度与目标自转之比上限，范围 0～1；取自 RMCS */
#define CHASSIS_SPIN_STUCK_CONFIRM_S    0.3f  /* 持续低于上述比例后换向的时间，单位 s；取自 RMCS 的 300 个 1 ms 周期 */
#define CHASSIS_SPIN_REVERSE_COOLDOWN_S 1.0f  /* 换向后暂停卡死检测的时间，单位 s；取自 RMCS 的 1000 个 1 ms 周期 */

/* 任务输入：DBUS 通道由任务换算，去中位后除以满偏值得到 -1～1，再由控制层乘平移速度上限。
 * 中心值使用 remote.h 的 RC_STICK_OFFSET。 */
#define CHASSIS_STICK_X_DIRECTION 1.0f    /* 前后摇杆方向系数，只取 +1 或 -1；使换算后的正值表示向前，待实车核对 */
#define CHASSIS_STICK_Y_DIRECTION (-1.0f) /* 左右摇杆方向系数，只取 +1 或 -1；使换算后的正值表示向左，待实车核对 */
#define CHASSIS_STICK_FULL_SCALE  660.0f  /* 去中位后的通道满偏值，单位为通道原始量，须大于零；按遥控器实际行程核对 */

/* 速度环：增益取自 RMCS 全向轮控制器。平移、自转环输出加速度，再乘质量或转动惯量折算到轮侧力矩；
 * 轮速环以实测底盘速度反解的轮速为目标，抑制单轮打滑，输出轮侧力矩。
 * 积分按控制周期累加、微分取相邻周期误差之差，都不含采样时间（RMCS 语义），因此 ki、kd 与控制周期绑定：
 * RMCS 更新率为 1 kHz，本工程控制周期见 chassis_task.c；周期为 RMCS 的 n 倍时，等效的 ki 须乘 n、kd 须除以 n。
 * 这些环在 RMCS 中不设积分限幅和输出限幅，启用积分前须先在 chassis.c 补设积分限幅。 */
#define CHASSIS_TRANSLATION_KP 5.0f  /* 平移速度环比例增益，单位 (m/s²)/(m/s) */
#define CHASSIS_TRANSLATION_KI 0.0f  /* 平移速度环积分增益，作用于按周期累加的速度误差和，单位 (m/s²)/(m/s)；0 关闭积分 */
#define CHASSIS_TRANSLATION_KD 0.0f  /* 平移速度环微分增益，作用于相邻周期速度误差之差，单位 (m/s²)/(m/s)；0 关闭微分 */
#define CHASSIS_ANGULAR_KP     5.0f  /* 自转速度环比例增益，单位 (rad/s²)/(rad/s) */
#define CHASSIS_ANGULAR_KI     0.0f  /* 自转速度环积分增益，作用于按周期累加的角速度误差和，单位 (rad/s²)/(rad/s)；0 关闭积分 */
#define CHASSIS_ANGULAR_KD     0.0f  /* 自转速度环微分增益，作用于相邻周期角速度误差之差，单位 (rad/s²)/(rad/s)；0 关闭微分 */
#define CHASSIS_WHEEL_KP       0.6f  /* 轮速环比例增益，单位 Nm/(rad/s) */
#define CHASSIS_WHEEL_KI       0.0f  /* 轮速环积分增益，作用于按周期累加的轮速误差和，单位 Nm/(rad/s)；0 关闭积分 */
#define CHASSIS_WHEEL_KD       0.0f  /* 轮速环微分增益，作用于相邻周期轮速误差之差，单位 Nm/(rad/s)；0 关闭微分 */

/* 对齐与跟随角度环：对齐运动方向、对齐云台和爬坡跟随共用，输入为角度误差，输出为目标自转角速度，
 * 并限幅到 CHASSIS_MAX_ANGULAR_RAD_S。积分、微分同样按控制周期累加。 */
#define CHASSIS_FOLLOW_KP 7.0f  /* 角度环比例增益，单位 (rad/s)/rad；取自 RMCS 哨兵参数 following_velocity_kp */
#define CHASSIS_FOLLOW_KI 0.0f  /* 角度环积分增益，作用于按周期累加的角度误差和，单位 (rad/s)/rad；0 关闭积分 */
#define CHASSIS_FOLLOW_KD 0.0f  /* 角度环微分增益，作用于相邻周期角度误差之差，单位 (rad/s)/rad；0 关闭微分 */

/* 功率模型 P = k1·Στ² + Σ(τ·ω) + k2·Σω² + 空载功率，τ 为轮侧力矩 Nm，ω 为轮侧角速度 rad/s。
 * k1、k2 为 RMCS 辨识值，对应其电机与减速比；更换电机或减速比后必须重新辨识。 */
#define CHASSIS_POWER_K1        2.958580f     /* 力矩平方损耗系数，单位 W/Nm²，须为正；待本车辨识 */
#define CHASSIS_POWER_K2        3.082190e-3f  /* 角速度平方损耗系数，单位 W/(rad/s)²；待本车辨识 */
#define CHASSIS_POWER_NO_LOAD_W 2.7f          /* 整车空载功率，单位 W；模型中计入可用功率，待本车辨识 */

/* 功率预算：虚拟缓冲能量按 (期望上限 - 实测功率) 积分，并受裁判缓冲能量限制。
 * 实测功率来自超级电容；电容离线时不积分，仅按裁判缓冲能量折减。 */
#define CHASSIS_POWER_BUFFER_LIMIT_J  30.0f  /* 虚拟缓冲能量上限，单位 J；取自 RMCS */
#define CHASSIS_POWER_EXCESS_W        15.0f  /* 虚拟缓冲满时在功率上限之上额外允许的功率，单位 W；取自 RMCS */
#define CHASSIS_POWER_BOOST_EXCESS_W  80.0f  /* boost 生效时在裁判上限之上额外允许的功率，单位 W；取自 RMCS */
#define CHASSIS_POWER_OFFLINE_LIMIT_W 85.0f  /* 裁判离线时的固定功率上限，单位 W；沿用原工程哨兵 100 W × 0.85，须按规则核对 */

/* 超级电容：分档取自 RMCS ChassisPowerController。boost 的额外功率随电容电压从基准线到控制线线性放开；
 * 充电功率上限随裁判缓冲能量分两段放开。电容通讯未接线前电容按离线处理，boost 不生效。 */
#define CHASSIS_SUPERCAP_VOLTAGE_BASE_V          12.0f   /* 电容电压不高于该值时 boost 额外功率完全关闭，单位 V；取自 RMCS，须按实车电容组核对 */
#define CHASSIS_SUPERCAP_VOLTAGE_CONTROL_V       12.5f   /* 电容电压达到该值时 boost 额外功率完全放开，单位 V，须大于基准线；取自 RMCS，须按实车电容组核对 */
#define CHASSIS_SUPERCAP_CHARGE_BUFFER_DEAD_J    0.0f    /* 充电功率上限降到零对应的裁判缓冲能量，单位 J；取自 RMCS */
#define CHASSIS_SUPERCAP_CHARGE_BUFFER_BASE_J    30.0f   /* 充电功率上限达到裁判功率上限对应的裁判缓冲能量，单位 J，须大于死区线；取自 RMCS */
#define CHASSIS_SUPERCAP_CHARGE_BUFFER_CONTROL_J 120.0f  /* 额外充电功率完全放开对应的裁判缓冲能量，单位 J，须大于基准线；取自 RMCS，须按当年规则的缓冲能量上限核对 */
#define CHASSIS_SUPERCAP_CHARGE_EXCESS_W         35.0f   /* 缓冲能量充足时在裁判上限之上额外允许的充电功率，单位 W；取自 RMCS */

#endif
