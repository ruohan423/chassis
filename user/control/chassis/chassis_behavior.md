# 底盘行为说明

本文描述 `chassis_behavior_t` 各行为的定义：平移指令在哪个坐标系、自转如何产生，以及上位机如何选择行为。
除“跟随云台”外，带坐标换算的行为一律以**上电世界系**为基准，底盘与云台解耦。

> 状态（2026-11-19）：`chassis.c`、`chassis_port.c`、`chassis_task.c` 和 USB 控制帧已按本文实现，**尚未上板验证**。
> 上车前必须先核对第 6 节的“待确认”，其中 IMU yaw 方向不对会让朝向环变成正反馈。

## 1. 坐标系

| 坐标系 | 定义 | 角度来源 |
|---|---|---|
| 底盘系 | 固连车体，x 向前、y 向左，wz 俯视逆时针为正 | 底盘 IMU 的**实时** yaw（`chassis_yaw`）：底盘系相对上电世界系转过的角度，每个控制周期取最新值 |
| 上电世界系 | 姿态解算启动时刻的底盘系，此后固定不动 | `chassis_yaw` 的零点；不另存角度 |
| 云台系 | x 为云台正前方、y 向左 | 云台相对底盘的 yaw 角（CAN 云台反馈，0～2π，记作 `gimbal_yaw_angle`） |

`chassis_yaw`：单位 rad，姿态解算启动时为 0，俯视逆时针为正，范围 -π～π（过 ±π 时跳变，代码只用它的正余弦和折算后的差值，不受跳变影响）。
数据链：`INS_Task`（1 kHz，VQF 6D 欧拉角）→ `usb_commun_update_imu` → `chassis_port_read_feedback` 写入 `chassis_feedback_t.chassis_yaw`，
与 yaw 角速度同一次快照，共用有效标志 `imu_valid`。同一个 yaw 也经 USB 状态帧上报给上位机。

世界系速度转到底盘系（每个控制周期用当前 `chassis_yaw` 计算，在底盘板上做）：

```
vx_底盘 =  cos(chassis_yaw)·vx_世界 + sin(chassis_yaw)·vy_世界
vy_底盘 = -sin(chassis_yaw)·vx_世界 + cos(chassis_yaw)·vy_世界
```

云台系速度转到底盘系（旋转方向与上式相反，因为 `gimbal_yaw_angle` 是云台相对底盘的角）：

```
vx_底盘 = cos(gimbal_yaw_angle)·vx_云台 - sin(gimbal_yaw_angle)·vy_云台
vy_底盘 = sin(gimbal_yaw_angle)·vx_云台 + cos(gimbal_yaw_angle)·vy_云台
```

朝向类行为用到的世界系目标朝向记作 `target_yaw`，单位 rad，与 `chassis_yaw` 同一基准。

## 2. 行为总表

编号即 `chassis_behavior_t` 的枚举值，也是上位机控制帧 `chassis_behavior` 字段的取值。

| 编号 | 行为 | 平移坐标系 | 自转 | 用到的请求 | 运动特点 |
|---|---|---|---|---|---|
| 0 | `STOP` | — | 0 | — | 原地不动 |
| 1 | `DIRECT` | 底盘系 | 直接取请求的 wz | vx、vy、wz | 给什么走什么，相对车头 |
| 2 | `TRANSLATE` | 世界系 | 0 | vx、vy | 只平移，车头不转 |
| 3 | `SPIN_SLOW` | 世界系 | 固定 0.3 × 角速度上限 | vx、vy | 慢转，同时走直线 |
| 4 | `SPIN_FAST` | 世界系 | 固定 0.6 × 角速度上限 | vx、vy | 快转，同时走直线 |
| 5 | `ALIGNMENT` | 世界系 | 最近的底盘 x 或 y 轴对齐平移方向 | vx、vy | 最近的一个侧面正对行进方向，最多转 45° |
| 6 | `STEP_DOWN` | 世界系 | 底盘正前或正后（取最近）对齐 `target_yaw` | vx、vy、target_yaw | 车头或车尾对准目标朝向，不掉头，最多转 90° |
| 7 | `LAUNCH_RAMP` | 世界系 | 底盘正前方对齐 `target_yaw` | vx、vy、target_yaw | 车头对准目标朝向 |
| 8 | `GIMBAL_FOLLOW` | **云台系** | 底盘正前方对齐云台 | vx、vy | 车头跟着云台转，相对云台走 |
| 9 | `CLIMB` | 底盘系（只沿 x） | 底盘正前方对齐爬坡朝向（世界系） | 爬坡输入 | 车头对准台阶，只前进或倒车 |

公共参数（`chassis_config.h`）：平移合速度上限 2.2 m/s，角速度上限 8 rad/s，角度环 kp = 7，输出限幅到角速度上限。
除 `CLIMB` 外，平移合速度超过上限时两轴等比例缩小。

**IMU 无效时**：编号 2～7 和 9 都依赖 `chassis_yaw`，`imu_valid` 为假时没有基准，一律按 `STOP` 目标处理（零目标，不复位控制器），
数据恢复后自动继续。`DIRECT` 和 `GIMBAL_FOLLOW` 不依赖 IMU，不受影响。

## 3. 各行为

### STOP 停止
平移、自转目标均为零。全自动下上位机离线、或世界系行为缺少 IMU 基准时使用。不复位控制器。

### DIRECT 直控
平移和自转请求都已在底盘系，直接作为速度目标；自转限幅到角速度上限。
遥控器自由控制使用。不属于世界系行为，不做坐标换算。

### TRANSLATE 只平移
平移请求在世界系，转到底盘系；自转为零，底盘朝向保持不动。

### SPIN_SLOW / SPIN_FAST 小陀螺
平移请求在世界系，每周期按当前 `chassis_yaw` 转到底盘系，因此底盘旋转的同时在世界系里走直线。
自转为固定角速度：慢速 2.4 rad/s，快速 4.8 rad/s，方向由卡死换向维护。

卡死换向：实测 yaw 角速度低于目标的 20% 持续 0.3 s，反转方向，之后 1.0 s 内不再检测；IMU 无效时不判定。

### ALIGNMENT 四面对齐
平移请求在世界系，转到底盘系后，计算平移方向到底盘 x 轴、y 轴的有符号夹角，取绝对值较小者进角度环。

效果：平移时底盘最多转 45°，让四个侧面中最近的一面正对行进方向，转到位后停止自转；不平移时不转。
平移方向恰在 45° 时两轴等距，选择会跳变，但两侧都收敛。
`ALIGNMENT_POWERED` 不单列：功率约束由模式（拨杆 s2）决定，行为不带功率属性。

### STEP_DOWN 两面对齐
平移请求在世界系。自转误差为 `target_yaw - chassis_yaw` 按 π 周期折算到 ±π/2，底盘正前方或正后方中较近的一面转向 `target_yaw`。
用于下台阶等前后对称、不想掉头的场合，最多转 90°。

### LAUNCH_RAMP 单面对齐
平移请求在世界系。自转误差为 `target_yaw - chassis_yaw` 按 2π 周期折算到 ±π，底盘正前方严格对准 `target_yaw`。
用于上坡、过洞：车体始终对正洞口，前进位置和速度由导航的世界系 x、y 控制。

### GIMBAL_FOLLOW 跟随云台
**唯一使用云台系的行为。** 平移请求在云台系，按 `gimbal_yaw_angle` 转到底盘系；
自转误差为 `gimbal_yaw_target_offset_rad + gimbal_yaw_angle` 按 2π 周期折算到 ±π，底盘正前方跟随云台朝向。
云台离线时相对角按 0 处理。

### CLIMB 上下台阶
平移只沿底盘 x 轴，速度取爬坡输入 `climb_speed`（上坡为正前进，下坡为负倒车），不做平移限幅。
自转误差为 `climb_direction - chassis_yaw` 按 2π 周期折算到 ±π，底盘正前方对准爬坡朝向；`climb_direction` 在世界系，与 `chassis_yaw` 同一基准。
仅全自动下 `climb_direction`、`climb_speed` 都有效时进入，优先于上位机指定的子行为。
上位机指定编号 9 而爬坡输入无效时按 `DIRECT` 处理。

## 4. 模式与行为的关系

模式（`chassis_mode_t`）由遥控器拨杆唯一决定，定输入来源和功率约束；行为只定运动方式。

| 模式 | 拨杆 | 行为 | 请求来源 | 摇杆平移坐标系 |
|---|---|---|---|---|
| `CHASSIS_STOP` | s2=2 | 停止，复位控制器 | — | — |
| `CHASSIS_FREE` / `CHASSIS_FREE_UNLIMITED` | s1=1 | `DIRECT` | 摇杆 | 底盘系 |
| `CHASSIS_SPIN` / `CHASSIS_SPIN_UNLIMITED` | s1=3 | `SPIN_FAST` | 摇杆 | 世界系 |
| `CHASSIS_FOLLOW_UNLIMITED` | s2=1、s1=2 | `GIMBAL_FOLLOW` | 摇杆 | 云台系 |
| `CHASSIS_AUTO` | s2=3、s1=2 | 爬坡输入有效为 `CLIMB`；否则上位机在线取其指定行为；离线为 `STOP` | 上位机 | — |

s2=3 受功率上限约束，s2=1 不受功率上限约束、只保留硬件保护。

遥控器小陀螺的摇杆方向现在相对上电时的车头，不再相对云台：推杆向前，车往上电时车头所指的方向走，与此刻云台和车体朝向无关。

## 5. 上位机接口

控制帧 `receive_cmd_t`（帧头 0x1002，**44 字节**，小端，定义在 `usb_commun.h`）：

| 偏移 | 字段 | 类型 | 含义 |
|---|---|---|---|
| 0 | `frame_header` | uint32 | 0x1002 |
| 4 | `frame_timestamp` | uint32 | 时间戳 |
| 8 | `x` | float | x 向速度 m/s，坐标系由行为决定（见第 2 节） |
| 12 | `y` | float | y 向速度 m/s，坐标系同 `x` |
| 16 | `w` | float | 自转角速度 rad/s，仅 `DIRECT` 采用 |
| 20 | `target_yaw` | float | 世界系目标朝向 rad，仅 `STEP_DOWN`、`LAUNCH_RAMP` 采用 |
| 24 | `chassis_behavior` | uint8 | 行为编号 0～9 |
| 25 | `pitch_cmd` | bool | 云台字段，未变 |
| 26 | `gimabal_model` | uint8 | 云台字段，未变 |
| 28 | `gimabal_tar_angle` | float | 云台字段，未变 |
| 32 | `gimabal_pat_angle` | float | 云台字段，未变 |
| 36 | `activate_energy` | bool | 云台字段，未变 |
| 40 | `eof` | uint8 | 0xA5 |

偏移 27、37～39、41～43 为对齐填充。相对旧帧（40 字节）：`chassis_spin` 换成 `chassis_behavior`，其前插入 `target_yaw`，之后各字段偏移加 4。
旧帧会因帧尾位置不符被整帧丢弃，不会被误解析；上位机未同步前底盘视为上位机离线，全自动下保持 `STOP`。

校验：行为编号大于 9、或 `x`、`y`、`w`、`target_yaw` 任一非有限值的帧不采用，也不刷新链路时间；200 ms 无有效帧判定离线。

状态帧 `send_cmd_t` 未改：`chassis_yaw` 就是本文的 `chassis_yaw`，`chassis_wz` 是 yaw 角速度。上位机算 `target_yaw` 和世界系速度时以它为基准。

注意：旧协议 `chassis_spin = 0` 表示直控，新协议编号 0 是 `STOP`，直控是 1。

## 6. 待确认与待接线

待确认（上车前）：

1. **IMU yaw 方向和轴向**：俯视逆时针转动底盘，上报的 `chassis_yaw` 应增大。方向相反时世界系平移会镜像，
   `STEP_DOWN`、`LAUNCH_RAMP`、`CLIMB` 的朝向环变成正反馈。姿态任务里没有安装方向的换算，yaw 直接取 BMI088 驱动输出的 z 轴。
2. **IMU 无效时的处置**：现在世界系行为按 `STOP` 处理，遥控器小陀螺也因此在 IMU 无效时不动（改动前不依赖 IMU）。自由控制不受影响。
3. **IMU 有效标志不判超时**：`imu_valid` 只表示姿态解算发布过数据，发布后不会再变假。姿态任务中途停止时 `chassis_yaw` 会停在旧值，
   底盘侧目前识别不出来。
4. **遥控器小陀螺改为世界系**：操作手感变了（见第 4 节），是否接受。
5. **`target_yaw` 链路超时后清零**：只在上位机离线时发生，此时全自动已是 `STOP`，不会据此转向。

待接线（接口已留，数据源缺失）：

- 爬坡输入 `climb_direction`、`climb_speed`：任务填 NAN，`CLIMB` 不会进入。接入时 `climb_direction` 用世界系；
  原来的 `climb_yaw` 输入已删除，底盘实测朝向统一取 `chassis_yaw`。
- 云台目标偏移 `gimbal_yaw_target_offset_rad`：恒为 0。
- 上位机 `boost` 字段：协议里没有，恒为假。

## 7. 世界系的使用约束

- 上电世界系只靠陀螺积分，会慢漂，与导航地图系不是同一个系。导航下发的速度和目标朝向应基于底盘上传的 `chassis_yaw` 换算，或周期性下发修正量。
- 世界系到底盘系的旋转必须在底盘板上按当前 yaw 做。上位机若用旧 yaw 预先换算，小陀螺 4.8 rad/s 下几十毫秒延迟就会让直线变成弧线。
- 只按 yaw 旋转相当于把世界系速度投到车体平面，坡度不大时可用。
- 底盘板复位后 `chassis_yaw` 重新从 0 开始，世界系随之重置，上位机需要重新对齐。
