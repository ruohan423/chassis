#ifndef SENTRY_CHASSIS_CHASSIS_CONFIG_H
#define SENTRY_CHASSIS_CHASSIS_CONFIG_H  /* 本配置头文件的重复包含保护，不作调参项。 */

/* 统一配置入口。以下沿用当前工程值，不代表实车已标定。
 * 完成移植需要逐项核对：接线/方向 → 几何/换算 → 轮速环 → 跟随环 → 功率模型/能量环。
 * 具体数据来源、缺失输入与验证步骤见 CHASSIS_PORTING.md。 */

/* 底层适配：逻辑轮序固定为右前、左前、左后、右后。
 * CAN 总线选 1 或 2；当前为 CAN2。四个 ID 必须属于同一个四电机控制组。
 * 每轮方向只允许 +1 或 -1，同步作用于反馈速度、反馈力矩和输出指令。 */

#define CHASSIS_CAN_BUS                   2U      /* 底盘电机使用的总线编号，1 为 CAN1、2 为 CAN2；须与实车接线和生成配置一致。 */
#define CHASSIS_MOTOR_COUNT               4U      /* 底盘轮电机数量，单位为个；当前控制器和控制报文固定支持四轮，不能仅改此值扩展。 */
#define CHASSIS_MOTOR_GROUP_FIRST_ID      1U      /* 四电机控制组的起始电机编号；当前组从 1 开始，须与控制报文和四轮编号配套。 */
#define CHASSIS_MOTOR_COMMAND_ID          0x200U  /* 四轮电流指令的标准帧标识；当前为电调 1～4 号组，换组须核对电调协议。 */
#define CHASSIS_MOTOR_FEEDBACK_ID_BASE    0x201U  /* 控制组首个电机的反馈标准帧标识；其余电机按组内编号差递增，须与电调协议一致。 */
#define CHASSIS_RF_MOTOR_ID               1U      /* 逻辑右前轮的实际电机编号；四轮编号须互不重复并属于所选控制组。 */
#define CHASSIS_LF_MOTOR_ID               2U      /* 逻辑左前轮的实际电机编号；四轮编号须互不重复并属于所选控制组。 */
#define CHASSIS_LB_MOTOR_ID               3U      /* 逻辑左后轮的实际电机编号；四轮编号须互不重复并属于所选控制组。 */
#define CHASSIS_RB_MOTOR_ID               4U      /* 逻辑右后轮的实际电机编号；四轮编号须互不重复并属于所选控制组。 */
#define CHASSIS_RF_MOTOR_DIRECTION        1.0f    /* 右前轮安装方向系数，只取 +1 或 -1；同时变换反馈速度、反馈力矩和输出指令。 */
#define CHASSIS_LF_MOTOR_DIRECTION        1.0f    /* 左前轮安装方向系数，只取 +1 或 -1；同时变换反馈速度、反馈力矩和输出指令。 */
#define CHASSIS_LB_MOTOR_DIRECTION        1.0f    /* 左后轮安装方向系数，只取 +1 或 -1；同时变换反馈速度、反馈力矩和输出指令。 */
#define CHASSIS_RB_MOTOR_DIRECTION        1.0f    /* 右后轮安装方向系数，只取 +1 或 -1；同时变换反馈速度、反馈力矩和输出指令。 */
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 40U     /* 电机反馈在线判据的超时时间，单位 ms；超过该时长无反馈后对应输出清零。 */
#define CHASSIS_CAN_RETRY_MS              100U    /* 总线初始化或发送失败后的任务重试等待时间，单位 ms；结合实际恢复时序确定。 */

/* 当前仅实现四个同型号、同减速比的 M3508 + C620。换电机还需修改 port 层编解码。
 * KA 为转子侧 Nm/A；减速比为电机转速/轮侧转速。
 * C620 原始指令和电流反馈 ±16384 对应 ±20 A，需核对电调协议与实际限流。 */
#define CHASSIS_MOTOR_KA_NM_PER_A                 0.0156f   /* 转子侧力矩常数，单位 Nm/A，不含减速比；当前参考值须按实际电机确认或标定。 */
#define CHASSIS_MOTOR_REDUCTION_RATIO             19.0f     /* 电机侧转速与轮侧转速之比，无量纲且须大于零；用于转速和轮侧力矩换算。 */
#define CHASSIS_MOTOR_CURRENT_FULL_SCALE_A        20.0f     /* 电调协议满量程对应的电流幅值，单位 A；属于物理刻度，不能随期望限流一起修改。 */
#define CHASSIS_MOTOR_COMMAND_FULL_SCALE          16384.0f  /* 电调协议的电流指令满量程幅值，单位为原始指令值；更换电调时按协议确认。 */
#define CHASSIS_MOTOR_FEEDBACK_CURRENT_FULL_SCALE 16384.0f  /* 电调电流反馈满量程的原始幅值；与实际安培刻度配套，不随输出限流改变。 */
#define CHASSIS_MOTOR_CURRENT_LIMIT_A             20.0f     /* 本工程允许的单轮电流幅值上限，单位 A；应为正且不超过协议满量程，须结合硬件能力确认。 */
/* 限流只改变输出上限，不改变反馈电流的物理刻度。 */
/* 原始输出限幅，由允许电流除以安培满量程再乘指令满量程派生；通常不直接修改。 */
#define CHASSIS_MOTOR_OUTPUT_LIMIT    (CHASSIS_MOTOR_CURRENT_LIMIT_A / CHASSIS_MOTOR_CURRENT_FULL_SCALE_A * CHASSIS_MOTOR_COMMAND_FULL_SCALE)
/* 每个反馈原始单位对应的电流，单位 A/原始值；由安培满量程和反馈满量程派生，不独立整定。 */
#define CHASSIS_MOTOR_CURRENT_PER_RAW (CHASSIS_MOTOR_CURRENT_FULL_SCALE_A / CHASSIS_MOTOR_FEEDBACK_CURRENT_FULL_SCALE)

/* 机械：默认四轮 45° 全向轮，改为 1 选择当前控制器支持的麦轮矩阵。
 * 轮径、半长、半宽均为实际接地点尺寸，单位 m。
 * 全向轮的自转力臂是轮心位置到驱动方向的垂距；只有当前对称 45° 布局才可用下式。 */
#ifndef USE_MECANUM_CHASSIS
#define USE_MECANUM_CHASSIS 0  /* 轮型开关：0 使用当前四轮全向矩阵，1 使用麦轮矩阵；须与实际轮型和布局一致。 */
#endif
#define CHASSIS_WHEEL_DIAMETER_M    0.100f                                                          /* 车轮有效滚动直径，单位 m，须大于零；应按实际轮胎和负载测量。 */
#define CHASSIS_HALF_LENGTH_M       0.200f                                                          /* 前后轮接地点间距的一半，单位 m；影响麦轮轮距和全向轮等效自转力臂。 */
#define CHASSIS_HALF_WIDTH_M        0.166f                                                          /* 左右轮接地点间距的一半，单位 m；影响麦轮轮距和全向轮等效自转力臂。 */
#define CHASSIS_WHEEL_TO_CENTRE_M   ((CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M) * 0.70710678f)  /* 全向矩阵使用的等效自转力臂，单位 m；按半长与半宽之和除以根号二派生，改变布局须复核。 */
#define CHASSIS_FRONT_BACK_LENGTH_M (2.0f * CHASSIS_HALF_LENGTH_M)                                  /* 麦轮前后轮距，单位 m；由底盘半长乘二派生，通常只修改对应半长。 */
#define CHASSIS_RIGHT_LEFT_LENGTH_M (2.0f * CHASSIS_HALF_WIDTH_M)                                   /* 麦轮左右轮距，单位 m；由底盘半宽乘二派生，通常只修改对应半宽。 */

/* 调度：实际 tick 频率由 osKernelGetTickFreq() 传入，周期按 tick 向上取整。
 * PID 采样时间和时间差换算使用同一实际周期/频率。 */
#define CHASSIS_CONTROL_PERIOD_MS      2U     /* 底盘控制任务请求周期，单位 ms，须大于零；实际按时钟计数向上取整，轮速与跟随环共用该周期。 */
#define CHASSIS_POWER_PERIOD_MS        1U     /* 功率任务请求周期，单位 ms，须大于零；实际按时钟计数向上取整，能量环使用对应采样时间。 */
#define CHASSIS_POWER_STARTUP_DELAY_MS 1000U  /* 功率任务开始周期计算前的等待时间，单位 ms；等待结束后重置能量积分时间基准。 */
/* 以下属性仅由任务创建处解释，优先级名称使用 CMSIS-OS2 定义。 */
#define CHASSIS_TASK_STACK_SIZE_BYTES       2048U                  /* 底盘任务栈大小，单位字节；须按最坏调用路径和上板栈余量核对。 */
#define CHASSIS_TASK_PRIORITY               osPriorityAboveNormal  /* 底盘任务的 CMSIS-OS2 优先级；当前高于普通任务，调整时须检查其他任务的调度延迟。 */
#define CHASSIS_POWER_TASK_STACK_SIZE_BYTES 2048U                  /* 功率任务栈大小，单位字节；须按最坏调用路径和上板栈余量核对。 */
#define CHASSIS_POWER_TASK_PRIORITY         osPriorityNormal       /* 功率任务的 CMSIS-OS2 优先级；当前为普通优先级，须保证实际周期满足能量环要求。 */

/* 运动约束：x 向右、y 向前，单位 m/s；w 逆时针为正，单位 rad/s。
 * 需按电机允许转速检查平移与旋转叠加后的单轮转速，不能只看单轴上限。 */
#define CHASSIS_MAX_SPEED_X_MPS     2.2f   /* 向右为正的机体横向目标速度幅值上限，单位 m/s；须核对与纵移、自转叠加后的各轮转速。 */
#define CHASSIS_MAX_SPEED_Y_MPS     2.2f   /* 向前为正的机体纵向目标速度幅值上限，单位 m/s；须核对与横移、自转叠加后的各轮转速。 */
#define CHASSIS_MAX_SPIN_RAD_S      8.0f   /* 机体目标自转角速度幅值上限，单位 rad/s；逆时针为正，须结合轮速与稳定性确认。 */
#define CHASSIS_MAX_ACC_MPS2        20.0f  /* 横向和纵向目标速度各自的变化率限幅，单位 m/s²；需按负载、附着和驱动能力整定。 */
#define CHASSIS_MAX_SPIN_ACC_RAD_S2 60.0f  /* 目标自转角速度变化率限幅，单位 rad/s²；需按整车稳定性和驱动能力整定。 */

/* 任务输入：DBUS 通道由任务换算；中心值使用 remote.h 的 RC_STICK_OFFSET。
 * 正方向须与实际摇杆核对，输入除数单位为通道原始量/(m/s)。 */
#define CHASSIS_STICK_X_DIRECTION 1.0f    /* 横向摇杆输入的方向系数，只取 +1 或 -1；使换算后的横向正值表示向右。 */
#define CHASSIS_STICK_Y_DIRECTION 1.0f    /* 纵向摇杆输入的方向系数，只取 +1 或 -1；使换算后的纵向正值表示向前。 */
#define CHASSIS_STICK_DIVISOR     300.0f  /* 去中位后的通道原始量除以此值得到 m/s，单位为通道原始量/(m/s)；须大于零并按实际量程标定。 */

/* 轮速 PID：输入为轮侧 rad/s，输出为 C620 原始指令单位。
 * 当前四轮共用一组参数；先架空单轮确认反馈负反馈方向，再整定。 */
#define CHASSIS_SPEED_KP             300.0f   /* 轮速环比例增益，单位为原始指令/(rad/s)；作用于轮侧速度误差，须实车整定。 */
#define CHASSIS_SPEED_KI             0.0f     /* 轮速环积分增益，单位为原始指令/rad；0 关闭积分项，须结合积分限幅整定。 */
#define CHASSIS_SPEED_KD             0.0f     /* 轮速环微分增益，单位为原始指令/(rad/s²)；0 关闭微分项，须评估反馈噪声。 */
#define CHASSIS_SPEED_INTEGRAL_LIMIT 5000.0f  /* 轮速误差积分状态的幅值上限，单位 rad，取非负值；不是电流指令输出限幅。 */

/* 跟随 PID：输入为云台相对底盘角度 deg，输出为 rad/s。
 * 角度来源未接入时，任务须传 gimbal_valid=false，跟随模式保持零输出。 */
#define CHASSIS_GYRO_SPIN_RAD_S       (-2.0f)  /* 小陀螺模式的有符号自转目标，单位 rad/s；负值表示顺时针，仍受自转速度和加速度限制。 */
#define CHASSIS_FOLLOW_VW_LIMIT       6.0f     /* 云台跟随环生成的自转角速度幅值上限，单位 rad/s；最终还受底盘统一运动限制。 */
#define CHASSIS_FOLLOW_KP             0.02f    /* 跟随环比例增益，单位为 (rad/s)/deg；以相对角零位为目标，须核对角度方向并整定。 */
#define CHASSIS_FOLLOW_KI             0.0f     /* 跟随环积分增益，单位为 (rad/s)/(deg·s)；0 关闭积分项，须结合积分限幅整定。 */
#define CHASSIS_FOLLOW_KD             0.001f   /* 跟随环微分增益，单位为 (rad/s)/(deg/s)；0 关闭微分项，须评估角度噪声与采样周期。 */
#define CHASSIS_FOLLOW_INTEGRAL_LIMIT 100.0f   /* 跟随角度误差积分状态的幅值上限，单位 deg·s，取非负值；不是角速度输出限幅。 */

/* 功率控制开关：电容接收尚未接入时会按离线处理，置 1 不代表反馈已经有效。
 * 兵种：0 步兵、1 英雄、2 哨兵；上限模式：0 裁判上限、1 允许能量环额外释放。 */
#ifndef USE_POWER_CONTROLLER
#define USE_POWER_CONTROLLER 1  /* 功率限制开关：0 关闭功率分配和守护计算，1 启用；关闭后仍保留电流限幅与反馈离线保护。 */
#endif
#ifndef USE_SUPER_CAPACITOR
#define USE_SUPER_CAPACITOR 1  /* 超级电容反馈支持开关：0 不使用，1 使用；启用后仍须具备真实在线、能量和功率数据。 */
#endif
#define CHASSIS_POWER_DIVISION    2U  /* 功率降级参考所用兵种编号：0 步兵、1 英雄、2 哨兵；应与实际机器人一致。 */
#define CHASSIS_POWER_LIMIT_MODE  1U  /* 初始用户功率上限选择：0 取当时裁判上限，1 取当时额外释放允许上限；仍受现有最小限值约束。 */
#define CHASSIS_POWER_RLS_ENABLED 1U  /* 递推辨识的初始开关：0 禁用、1 启用；故障粘滞关闭后不会仅因链路恢复自动重启。 */

/* 功率模型 P = Σ(τ·ω + k1·|ω| + k2·τ²) + k3。
 * τ 是含减速比的轮轴 Nm，ω 为轮侧 rad/s；k3 为整车固定损耗 W。
 * k1 单位 W/(rad/s)，k2 单位 W/Nm²。更换电机、减速比或反馈单位后必须重新标定。 */
#define CHASSIS_POWER_K1                  0.22f     /* 角速度绝对值损耗的初始系数，单位 W/(rad/s)；须用真实功率测量和轮侧角速度标定。 */
#define CHASSIS_POWER_K2                  1.2f      /* 轮侧力矩平方损耗的初始系数，单位 W/Nm²；求解中作为除数，须保持正值并实测标定。 */
#define CHASSIS_POWER_K3_W                2.78f     /* 整车功率模型的固定损耗，单位 W；当前不参与在线辨识，须用真实静态功耗标定。 */
#define CHASSIS_POWER_RLS_DELTA           1e-5f     /* 递推辨识初始矩阵尺度，须大于零；当前实现为初始矩阵等于该值乘单位矩阵，不取其倒数。 */
#define CHASSIS_POWER_RLS_LAMBDA          0.99999f  /* 递推辨识遗忘因子，须满足 0 < λ ≤ 1；越接近 1 越保留历史信息，需结合采样周期整定。 */
#define CHASSIS_POWER_RLS_MIN_MEASURED_W  5.0f      /* 允许辨识更新的实测功率绝对值门限，单位 W；低于或等于门限时不更新参数。 */
#define CHASSIS_POWER_RLS_MIN_COEFFICIENT 1e-5f     /* 辨识后两个损耗系数的正数下限，单位分别随对应模型项；用于避免负系数及零除数。 */

/* 能量环：裁判阈值单位 J；电容反馈为 0～255 归一化值，不是 J。
 * 电容满量程能量须按实车标定，当前移植参考值为 2100 J。
 * PD 的输入为能量反馈/目标平方根差，输出为 W，换归一化方式后须重新整定。 */
#define CHASSIS_POWER_REFEREE_FULL_ENERGY_J 60.0f    /* 裁判缓冲能量高目标值，单位 J，也用于满缓冲判断；须按实际协议和规则核对。 */
#define CHASSIS_POWER_REFEREE_BASE_ENERGY_J 50.0f    /* 裁判缓冲能量低目标值，单位 J；应不高于高目标值，与高目标一起决定能量环调节区间。 */
#define CHASSIS_POWER_CAP_FULL_ENERGY_RAW   230.0f   /* 电容归一化能量高目标值，当前尺度为 0～255，无量纲；不是能量的焦耳值。 */
#define CHASSIS_POWER_CAP_BASE_ENERGY_RAW   30.0f    /* 电容归一化能量低目标值，无量纲；应不高于高目标，须与电容反馈尺度一致。 */
#define CHASSIS_POWER_CAP_CAPACITY_J        2100.0f  /* 电容反馈满量程对应的可用能量，单位 J，须按实车标定；用于归一化换算和能量估计限幅。 */
#define CHASSIS_POWER_CAP_ENERGY_RAW_MAX    255.0f   /* 电容能量反馈满量程原始值，无量纲；当前字段为 8 位，应随协议确认，不能单独改变反馈尺度。 */
#define CHASSIS_POWER_ENERGY_KP             50.0f    /* 能量平方根误差的比例增益，输出单位 W；裁判能量与电容归一化反馈的尺度不同，须实车整定。 */
#define CHASSIS_POWER_ENERGY_KD             0.2f     /* 能量平方根误差变化率的微分增益，输出单位 W；须结合功率任务实际周期和测量噪声整定。 */
#define CHASSIS_POWER_CAP_MAX_OUTPUT_W      300.0f   /* 允许电容额外释放的功率上限，单位 W，同时用于能量环输出限幅；须核对电容硬件能力。 */

/* 离线降级、功率分配和模型记忆。
 * 1000 ms 内仍以离线电机的末次速度/力矩估计耗能，但电机控制输出始终清零。
 * 两个分配阈值是四轮速度误差绝对值之和，单位 rad/s。 */
#define CHASSIS_POWER_CAP_RUNOUT_THRESHOLD_W          43.0f  /* 判定电容离线耗尽所用实测功率门限，单位 W；当前也作为裁判反馈功率上限的最小取值。 */
#define CHASSIS_POWER_CAP_OFFLINE_TARGET_W            37.0f  /* 缺少可信功率测量时估计电容能量所用的参考输入功率，单位 W；是降级假设，不是实测值。 */
#define CHASSIS_POWER_BOTH_OFFLINE_RATIO              0.85f  /* 裁判与电容有效反馈均缺失时的功率降额系数，无量纲；乘离线参考额定功率确定降级上限。 */
#define CHASSIS_POWER_MIN_LIMIT_RATIO                 0.8f   /* 最小用户功率及能量环下限相对裁判功率上限的比例，无量纲；需结合功率上限策略确认。 */
#define CHASSIS_POWER_INITIAL_MIN_W                   30.0f  /* 功率模块初始化阶段的临时最小用户功率，单位 W；守护运行后由参考功率及最小比例更新。 */
#define CHASSIS_POWER_MOTOR_HOLD_MS                   1000U  /* 反馈离线后继续用末次速度、力矩估计耗能的时长，单位 ms；该期间对应电机输出仍清零。 */
#define CHASSIS_POWER_ERROR_DISTRIBUTION_RAD_S        20.0f  /* 四轮速度误差绝对值之和的高阈值，单位 rad/s；超过后优先按误差分配功率，须大于低阈值。 */
#define CHASSIS_POWER_PROPORTIONAL_DISTRIBUTION_RAD_S 15.0f  /* 四轮速度误差绝对值之和的低阈值，单位 rad/s；不超过时按需求功率比例分配，与高阈值间插值。 */

/* 裁判离线时的参考功率表，需按本赛季实际规则核对；不是联网裁判值。
 * 表按等级 1～10，哨兵使用独立常量。 */
#define CHASSIS_POWER_LEVEL_COUNT       10U                                                   /* 离线功率参考表覆盖的等级数量，单位为级；须与两张表项数和实际有效等级范围一致。 */
#define CHASSIS_POWER_HERO_LIMITS_W     {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U}  /* 英雄机器人各等级的离线参考功率表，单位 W，依次对应等级 1～10；须按实际参赛规则核对。 */
#define CHASSIS_POWER_INFANTRY_LIMITS_W {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U}   /* 步兵机器人各等级的离线参考功率表，单位 W，依次对应等级 1～10；须按实际参赛规则核对。 */
#define CHASSIS_POWER_SENTRY_LIMIT_W    100U                                                  /* 哨兵在裁判离线时使用的参考功率上限，单位 W；须按实际参赛规则核对。 */

#endif
