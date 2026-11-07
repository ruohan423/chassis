# 底盘分层与实车配置

统一参数入口是同目录的 `chassis_config.h`。文件中保留当前工程数值，并标明单位和适用条件；这些值不是已通过实车标定的结论。配置齐全也不能替代实际轮向、通信和闭环验证。

## 两组适配接口

```text
user/control/chassis/
    chassis_config.h                 硬件与算法配置，任务调度参数归各任务文件
    chassis.h / chassis.c            给 task 提供接口、构造控制器、生成行为目标
    chassis_port.h / chassis_port.c  给两个 controller 提供反馈数据与电机输出适配
    chassis_controller/
        chassis_controller.h / .c    移植的运动学、斜坡、轮速 PID
    power_contorller/
        power_controller.h / .c      移植的功率分配、能量环、RLS、离线降级
```

原来的 `chassis_external/chassis.c`、`chassis_command.c/.h`、`chassis_port/chassis_port.c` 六个适配文件合并为四个：

- **控制器信息部分 `chassis_port/chassis_port.c`**：管理 CAN 接线、方向、归一化电机速度/力矩、反馈在线状态和指令下发；复制裁判快照、转换电容状态并进行有效性判断。这里拥有两个控制器共同读取的输入格式和数据对象，不依赖任务接口。
- **任务接口部分 `chassis_external/chassis.c`**：唯一装配轮速 PID、跟随 PID、运动学与功率控制器；将模式与输入转换为目标速度，执行停止、更新和发送流程。原 `chassis_command` 的逻辑变为本文件内部函数，不再导出独立命令模块。

`chassis_port/chassis_port.h` 声明 `chassis_motors[]`、功率反馈输入类型、`chassis_power_sources` 和反馈刷新接口。`chassis_controller` 和 `power_controller` 只接触这些适配后的量，不读取 HAL 句柄或原始裁判/电容协议。`power_controller.h` 从 `chassis_port/chassis_port.h` 获取输入类型，port 不反向包含控制器头文件。

任务只包含 `chassis_external/chassis.h`，不直接使用 port、协议模块或移植控制器。功率任务调用 `chassis_power_update()` 时，先刷新 port 中的反馈对象，再执行功率控制周期。统一配置仍独立保存在 `chassis_config.h`。

纯函数 PID、ramp 和 RLS 继续保留在 `user/algorithm`。两个有状态控制器保留当前 `chassis_controller/` 和 `power_controller` 目录；本次不改变其算法流程和调度周期。

## 参数配置清单

| 配置组 | 头文件中的参数 | 需要确认的实物或来源 |
| --- | --- | --- |
| 总线与报文 | `CHASSIS_CAN_BUS`、`MOTOR_GROUP_FIRST_ID`、`MOTOR_COMMAND_ID`、`MOTOR_FEEDBACK_ID_BASE` | CAN1/CAN2 接线，C620 的控制组与反馈 ID；外设初始化、波特率和引脚仍由 CubeMX 管理 |
| 轮序与方向 | `CHASSIS_RF/LF/LB/RB_MOTOR_ID`、对应 `MOTOR_DIRECTION` | 逻辑顺序为右前、左前、左后、右后；四个不同 ID 必须覆盖同一个四电机控制组，方向只能为 ±1 |
| 电机物理量 | `MOTOR_KA_NM_PER_A`、`MOTOR_REDUCTION_RATIO` | 转子侧力矩常数和实测减速比；不能把轮侧力矩常数再次乘减速比 |
| 电流刻度与限幅 | `MOTOR_CURRENT_FULL_SCALE_A`、`MOTOR_COMMAND_FULL_SCALE`、`MOTOR_FEEDBACK_CURRENT_FULL_SCALE`、`MOTOR_CURRENT_LIMIT_A` | 电调协议刻度和允许的连续/峰值电流，单位 A 或原始指令单位 |
| 通信保护 | `MOTOR_FEEDBACK_TIMEOUT_MS`、`CAN_RETRY_MS` | 实际反馈频率、任务调度延迟、允许断连时间 |
| 轮型与几何 | `USE_MECANUM_CHASSIS`、`WHEEL_DIAMETER_M`、`HALF_LENGTH_M`、`HALF_WIDTH_M`、`WHEEL_TO_CENTRE_M` | 实测有效轮径和轮接地点位置；全向轮须满足当前 45° 对称布局 |
| 运动上限 | `MAX_SPEED_X/Y_MPS`、`MAX_SPIN_RAD_S`、`MAX_ACC_MPS2`、`MAX_SPIN_ACC_RAD_S2` | 电机允许转速、负载、轮胎附着和传动强度，含平动与旋转同时输入的情况 |
| 输入标定 | `STICK_X/Y_DIRECTION`、`STICK_DIVISOR` | 摇杆实际通道、方向、中位和有效量程；中位沿用 `remote.h` 的 `RC_STICK_OFFSET` |
| 轮速 PID | `SPEED_KP/KI/KD`、`SPEED_INTEGRAL_LIMIT` | 输入轮侧 rad/s、输出电流原始指令；积分限幅作用于误差积分，单位 rad |
| 跟随与自转 | `FOLLOW_KP/KI/KD`、`FOLLOW_INTEGRAL_LIMIT`、`FOLLOW_VW_LIMIT`、`GYRO_SPIN_RAD_S` | 云台相对角的零位、正方向和环绕处理；输入 deg、输出 rad/s，积分限幅为 deg·s |
| 功率模式 | `USE_POWER_CONTROLLER`、`USE_SUPER_CAPACITOR`、`POWER_DIVISION`、`POWER_LIMIT_MODE` | 是否接入电容、实际兵种，以及是否允许能量环额外释放功率 |
| 功率模型 | `POWER_K1`、`POWER_K2`、`POWER_K3_W` | 由真实电压/电流或功率反馈标定，必须匹配轮侧力矩和轮侧角速度的单位 |
| RLS | `POWER_RLS_ENABLED`、`DELTA`、`LAMBDA`、`MIN_MEASURED_W`、`MIN_COEFFICIENT` | 有效功率反馈、采样周期、样本激励与噪声；lambda 现在确实传入 RLS |
| 能量反馈 | 裁判 `FULL/BASE_ENERGY_J`、电容 `FULL/BASE_ENERGY_RAW`、`CAP_CAPACITY_J`、`CAP_ENERGY_RAW_MAX` | 裁判缓冲为 J，电容输入为 0～255 归一化数；需核对电容容量、协议尺度及可用能量定义 |
| 能量环 | `POWER_ENERGY_KP/KD`、`CAP_MAX_OUTPUT_W` | 功率测量延迟、电容能力和整车实际能量响应；当前 PID 没有参考实现的 D 项滤波 |
| 离线策略 | `CAP_RUNOUT_THRESHOLD_W`、`CAP_OFFLINE_TARGET_W`、`BOTH_OFFLINE_RATIO`、`MIN_LIMIT_RATIO`、`INITIAL_MIN_W`、`MOTOR_HOLD_MS` | 反馈丢失后的允许功率、保守系数、离线电机耗能记忆时间 |
| 功率分配 | `ERROR_DISTRIBUTION_RAD_S`、`PROPORTIONAL_DISTRIBUTION_RAD_S` | 四轮速度误差绝对值之和，单位 rad/s，不是 W |
| 离线功率表 | `POWER_HERO_LIMITS_W`、`INFANTRY_LIMITS_W`、`SENTRY_LIMIT_W` | 按实际参赛规则核对；这些是联网裁判失效后的参考值 |

表内省略的公共前缀均为 `CHASSIS_`。所有派生量在头文件中计算：输出指令限幅由允许电流与协议满量程计算，反馈电流刻度独立于限流，麦轮全长/全宽由半长/半宽计算。

当前 `port` 和功率模型要求四轮同型号、同减速比。换成其他电机协议、异构四轮或非对称布局，必须改对应适配/运动学，不能只改配置值。

## 必须补齐的输入

1. **电机实际接线和转向**：当前 ID 1～4、方向均为 +1，仅延续原工程。调换线序后只改四轮 ID/方向，不能再交换运动学矩阵的行。发送和反馈在 `chassis_port/chassis_port.c` 使用相同方向变换，避免只改一侧形成正反馈。
2. **云台相对底盘角**：`Chassis_Task` 目前传入 `gimbal_valid=false`，跟随模式立即停止。接入真实来源后，传入零位一致、方向正确且已处理 ±180° 环绕的角度，在线/超时状态同时作为有效标志。普通和小陀螺模式在角度无效时使用底盘参考系。
3. **超级电容通信**：现有模块仅存储状态，尚未实现实际接收。接收层需要向 `super_capacitor_update()` 提供 DCDC、错误码、归一化能量及实测底盘功率，未接入时始终按离线处理。
4. **真实功率测量**：2027 裁判协议 `0x0202` 没有底盘功率字段。`chassis_port/chassis_port.c` 明确设置 `chassisPowerValid=0`，不会把保留字段、占位零值或模型估计当成真实 RLS 观测。完整在线辨识需要接通电容实测功率或另外实现真实测量来源。
5. **规则相关值**：兵种、离线功率表、缓冲能量阈值和电容输出上限需要人工核对。当前数值来自已有移植代码，不作为规则正确性的保证。

RLS 保留原来的粘滞关闭行为：无可信反馈等情况关闭后，不会仅因链路恢复自动重启。重新使能需要上层明确调用控制器的 `Power_setRLSEnabled()`；统一配置中的开关控制初始化状态，不能替代恢复策略。上板时应确认无实测功率不会更新 RLS，恢复真实测量后再验证重新使能策略。

## 任务接入与保护

线程和公共信号量由用户在 CubeMX 中创建并重新生成。`Core/Src/freertos.c` 不手写对象属性、句柄、创建调用或业务初始化，已有生成区保持由 CubeMX 维护。具体属性同时写在各任务源文件开头。

| 文件 | 创建名称 / 入口 | 优先级 | 栈大小 |
| --- | --- | --- | --- |
| `user/task/chassis_task/chassis_task.c` | `chassis_task` / `Chassis_Task` | `osPriorityAboveNormal` | 2048 字节 |
| `user/task/power_task/power_task.c` | `power_task` / `Power_Task` | `osPriorityNormal` | 2048 字节 |

若 CubeMX 以 32 位字填写栈大小，两项均填 512。入口绑定当前文件的实现，避免生成重复的强定义。未创建任务不会自动启动；启用功率控制时，底盘会等待功率任务首轮计算完成，再允许非零控制输出。

底盘周期和重试间隔保留在 `chassis_task.c`，功率周期和启动等待保留在 `power_task.c`。`chassis_config.h` 不再放任务创建属性、周期或启动重试参数。任务将实际时钟频率及取整后的周期计数传入控制器，PID 采样时间与调度周期同源。

`Chassis_Task` 调用 `chassis_init(now_tick, tick_frequency_hz, period_ticks)` 完成装配，每周期换算输入后调用 `chassis_update()`；CAN 启动或发送失败由任务等待后重试。

`Power_Task` 入口调用 `chassis_power_init(now_tick, tick_frequency_hz, period_ticks)`，完成启动等待后重置积分起点，再周期调用 `chassis_power_update()`。不依赖线程创建顺序，也不在 `freertos.c` 初始化功率模块。

重构同时修正了几处影响验证的边界：

- 电机必须收到过合法 CAN 反馈，避免上电前 40 ms 被误判在线；CAN 反馈检查标准帧和 8 字节长度。
- 停止立即清零输出、轮速 PID 和目标斜坡历史，恢复时从零目标重新爬升；关闭功率控制也保留电机反馈超时保护。
- 全向轮速度正解的 x 符号与逆解保持一致；实车需按当前轮型核对三个轴的方向、轮侧速度和正逆解一致性。
- 首次调用或同 tick 停止不再除以零；功率效率在零测量时有明确值。
- 离线电机仍在配置的记忆时长内计入功率估计，但不允许输出；记忆时长改用实际 tick 时间，不再假定守护任务永远为 1 ms。

`measuredDisplacement` 仍是搬运控制器中机体速度分量的积分，不能当作世界坐标位置。需要世界坐标里程计时还须接入姿态/定位融合，配置几何参数不能替代该实现。

本次仅移除 `freertos.c` 用户区手写装配，不改生成区与 `.ioc`。裁判任务和信号量按 `user/task/judge_task/judge_task.c` 注释由用户创建；其余串口配置见 `user/module/judge/JUDGE_USAGE.md`。

## 验证与整定顺序

1. 架空底盘，逐个确认 ID、反馈在线状态及正负方向，发送限流从低值开始；反馈方向和输出方向同时归一化。
2. 核对原始电流、轮侧 rad/s 和轮侧 Nm 的换算，确认减速比只使用一次。
3. 逐轴确认 x 向右、y 向前、w 逆时针；测试停止、遥控超时、单轮反馈丢失、云台角无效。
4. 在实际负载和轮径下整定轮速 PID、速度及加速度限制。按逆解检查组合输入时每个电机的转速，当前只限制机体各轴目标，没有额外的单轮最高转速缩放。
5. 接入真实功率/电容数据后标定 k1/k2/k3，验证静态能量环，再评估 RLS 的激励、收敛、异常数据和恢复策略。
6. 最后验证跟随模式、能量释放和离线降级，检查最坏任务执行时间及栈余量。

```sh
cmake --build build/Debug --parallel 4
git diff --check
```

上板验证使用实际电机反馈、裁判串口和电容测量，不以模拟在线状态或固定功率代替真实输入。先架空底盘并采用低限流，确认停止与断连保护，再检查运动方向、负载响应、功率限制、任务周期和栈余量。
