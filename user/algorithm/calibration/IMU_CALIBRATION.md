# BMI088 陀螺仪校准流程

## 数据和调用链

`BMI088_read()` 返回经过单位换算、但未扣除零偏的传感器数据：陀螺仪为 `rad/s`，
加速度为 `m/s^2`，温度为 `°C`。校准器和 VQF 直接使用同一组 SI 单位，不做中间
单位转换。不要在 `BMI088driver.c` 中直接减去保存的陀螺仪零偏。

当前校准流程由 `INS_Task()` 在启动阶段调用：

1. `cal_session_default()` 加载温度稳定和静止采样参数。
2. `cal_session_init()` 初始化校准会话。
3. `cal_session_start()` 进入温度预热阶段。
4. 每次采样调用 `cal_session_push()`。
5. 温度稳定后进入静止采样阶段，内部调用 `gyro_cal_push()`。
6. 采样达到要求后，内部调用 `gyro_cal_finish()`。
7. 将返回的 `bias`、`sigma` 和 `ref_temp_c` 写入 `bias_config.h`。
8. 校准成功后，`INS_Task()` 初始化 VQF，并在 `initVqf()` 后只调用一次 `setBiasEstimate()`。
9. 校准失败时，`INS_Task()` 不进入 VQF 主循环，而是保持故障等待状态。

VQF 会把校准零偏作为初始先验，并继续进行运行时在线估计。运行期间不要重复调用
`setBiasEstimate()`，因为该调用会重新设置零偏协方差，覆盖在线估计的累积状态。

启动配置由 `CAL_ON_STARTUP` 控制：

- `CAL_ON_STARTUP != 0`：启动时执行校准，VQF 使用本次校准得到的 `bias` 和 `sigma`。
- `CAL_ON_STARTUP == 0`：跳过启动校准，VQF 使用 `GYR_BIAS_X`、`GYR_BIAS_Y`、
  `GYR_BIAS_Z` 和 `GYR_BIAS_SIGMA` 宏中的固定值。

## 当前采样参数

当前 BMI088 配置为陀螺仪 1000 Hz、加速度 400 Hz，`INS_Task()` 按 1 ms 周期运行。
BMI088 驱动直接输出陀螺仪 `rad/s`、加速度 `m/s^2` 和温度 `°C`；校准器不再做
`m/s^2` 与 `g` 之间的转换。
校准参数集中在 `bias_config.h`：

```c
#define CAL_DURATION_S 10.0f
#define CAL_RATE_HZ 1000.0f
#define CAL_SAMPLES 10000U
#define CAL_GAP_S 0.02f
#define CAL_TEMP_DRIFT_C 0.5f
#define CAL_ACC_TOL_MS2 0.5f
#define CAL_ACC_SIGMA_MS2 0.15f
#define CAL_GYR_MAX_RS 0.05f
```

`CAL_SAMPLES` 和 `CAL_DURATION_S` 应与实际提供给 `cal_session_push()` 的陀螺仪
采样频率保持一致。若改为由 DRDY 事件驱动，仍应保证每个陀螺仪样本只推送一次，
并重新确认 `CAL_RATE_HZ`、`CAL_SAMPLES` 和 `CAL_GAP_S`。

静止加速度幅值使用 `GRAVITY_MS2`（`9.80665 m/s^2`）作为重力参考，
`CAL_ACC_TOL_MS2` 和 `CAL_ACC_SIGMA_MS2` 也使用 `m/s^2`。

## 校准要求

校准期间必须保持设备机械静止。恒定的缓慢绕重力方向旋转无法与陀螺仪零偏区分，
因此应由操作人员保持静止，或通过运动检测拒绝这类数据。

温度必须先达到 `TEMP_TARGET_C`，并在 `TEMP_TOL_C` 范围内连续稳定后才开始静止
采样。若在采样期间出现运动、加速度幅值异常、温度漂移或采样间隔过大，会话应失败。

保存的 `sigma` 表示本次零偏测量的不确定度。调用 `setBiasEstimate()` 时应传入该值，
不要简单设置为零。

加热由公共 PID 控制模块统一执行。校准期间由校准任务更新 PID；校准成功后，INS
周期继续使用同一个 PID 状态和 PWM 输出维持目标温度。

加热器 `user/control/heater/heater.c` 和蜂鸣器 `user/module/buzzer/buzzer.c` 直接调用 HAL 定时器接口与比较值宏，不再单独封装 PWM。校准成功时蜂鸣器鸣叫 500 ms；
校准失败时蜂鸣器鸣 200 ms、停 200 ms，重复两次。校准成功后不会关闭加热 PWM。
