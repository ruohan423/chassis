# VQF-C 使用说明

本文说明本工程中 VQF-C 的调用顺序、传感器单位、采样周期、姿态输出、静止检测、陀螺零偏估计和磁力计功能。

## 当前工程的实际调用

当前工程只使用六轴模式，不使用磁力计。`INS_Task()` 的实际流程如下：

1. 启动时先完成 BMI088 静止陀螺仪校准；校准失败时不会进入 VQF 主循环。
2. 校准成功后调用 `initVqf(0.001f, 0.0025f, 0.0f)`。
3. 只在 VQF 初始化后调用一次 `setBiasEstimate()`，写入校准得到的陀螺仪零偏和不确定度；
   后续不重复调用，避免重置 VQF 的在线零偏估计协方差。
4. 任务每 1 ms 读取一次 BMI088，并调用 `updateGyr(gyro)`。
5. 只有 `Calib_IsAccelReady()` 返回真时才调用 `updateAcc(accel)`；加速度计实际配置为 400 Hz。
6. 每个周期调用 `getQuat6D(quat6d)` 获取六轴姿态。
7. 温度加热由公共 PWM 模块继续控制，不属于 VQF 的输入更新接口。

当前工程的数据单位为：陀螺仪 `rad/s`，加速度 `m/s^2`，温度 `°C`。传感器驱动、
校准器和 VQF 之间不做重复的单位往返转换。

## 1. 文件和接口说明

本文针对 `vqf/vqf.c` 和 `vqf/vqf.h` 版本。使用时编译该目录下的 `vqf.c`，并包含对应的 `vqf.h`。

当前头文件中默认定义：

```c
#define VQF_SINGLE_PRECISION
```

因此：

```c
typedef float vqf_real_t;
typedef float vqf_double_t;
```

## 2. 初始化

```c
void initVqf(vqf_real_t gyrTs,
             vqf_real_t accTs,
             vqf_real_t magTs);
```

参数是传感器的**采样周期**，单位为秒，不是采样频率。

例如：

```c
// 陀螺仪 1000 Hz，加速度计 100 Hz，磁力计 50 Hz
initVqf(0.001f, 0.01f, 0.02f);
```

当前工程的实际初始化调用是：

```c
initVqf(0.001f, 0.0025f, 0.0f);
```

其中磁力计周期为 `0.0f`，表示当前不使用磁力计。

频率和周期换算：

```text
Ts = 1 / Fs
Fs = 1 / Ts
```

示例：

```text
1000 Hz -> 0.001 s
200 Hz  -> 0.005 s
100 Hz  -> 0.01 s
50 Hz   -> 0.02 s
5 Hz    -> 0.2 s
```

### 默认陀螺仪周期

`vqf/vqf.c` 中定义了：

```c
#define TICK_INTERVAL (4.0f * 0.000313f)
```

其值为：

```text
0.001252 s = 1.252 ms
约 798.7 Hz
```

当前工程不会依赖这个默认值，因为 `INS_Task()` 明确传入 `0.001f`。当其他调用者传入
`gyrTs <= 0` 时，`vqf/vqf.c` 才会使用该兜底值。

## 3. 输入数据单位

### 3.1 陀螺仪

```c
void updateGyr(const vqf_real_t gyr[3]);
```

输入单位必须是：

```text
rad/s
```

如果传感器输出为 `deg/s`，需要转换：

```c
#define DEG_TO_RAD 0.017453292519943295f

gyr[0] = gyr_dps[0] * DEG_TO_RAD;
gyr[1] = gyr_dps[1] * DEG_TO_RAD;
gyr[2] = gyr_dps[2] * DEG_TO_RAD;
```

代码中陀螺仪积分为：

```c
angle = gyrNorm * gyrTs;
```

因此 `gyrNorm` 使用 `rad/s`，`gyrTs` 使用 `s`，最终角度为 `rad`。

### 3.2 加速度计

```c
void updateAcc(const vqf_real_t acc[3]);
```

三个轴可以使用以下任一种单位：

- `m/s^2`
- `g`

但三个轴必须使用同一种单位。若使用 `m/s^2`，静止时加速度模长约为：

```text
9.81 m/s^2
```

若使用 `g`，静止时加速度模长约为：

```text
1.0 g
```

当前工程向 VQF 传入 `m/s^2`，默认 `restThAcc = 0.5f`，它与加速度输入使用相同单位。
因此：

- 加速度用 `m/s^2` 时，阈值约为 `0.5 m/s^2`；
- 加速度用 `g` 时，建议根据噪声重新设置阈值，例如 `0.05g` 或其他实测值。

虽然 VQF 允许使用 `g`，当前工程不要再转换为 `g`，因为 BMI088 驱动和校准器已经统一
使用 `m/s^2`；若将来更换单位，必须同步调整 VQF 静止检测阈值。

### 3.3 磁力计

```c
void updateMag(const vqf_real_t mag[3]);
```

磁力计三个轴使用同一单位即可，常见单位为：

- `uT`
- `mG`
- 原始比例转换后的其他线性单位

VQF 主要使用磁场方向和模长相对变化，因此不要求必须使用 `uT`。必须保证：

- 三个轴单位一致；
- 磁力计已经完成硬铁校准；
- 最好完成软铁校准；
- 磁力计坐标轴方向与陀螺仪、加速度计统一；
- 磁力计安装方向固定。

## 4. 输入更新顺序

### 4.1 只有陀螺仪和加速度计

```c
updateGyr(gyr);
updateAcc(acc);

vqf_real_t quat6D[4];
getQuat6D(quat6D);
```

### 4.2 有外部磁力计

```c
updateGyr(gyr);
updateAcc(acc);
updateMag(mag);

vqf_real_t quat9D[4];
getQuat9D(quat9D);
```

建议按照各传感器实际到达时间调用，不要为了凑同步而重复使用旧数据。

如果陀螺仪频率最高，可以在陀螺仪数据到达时调用 `updateGyr()`；加速度计和磁力计数据到达时分别调用对应函数。
`setBiasEstimate()` 属于初始化接口，不应放在周期循环中反复调用。

## 5. 姿态输出

### 5.1 四元数接口

```c
void getQuat3D(vqf_real_t out[4]);
void getQuat6D(vqf_real_t out[4]);
void getQuat9D(vqf_real_t out[4]);
```

四元数顺序为：

```text
{w, x, y, z}
```

含义：

| 接口 | 使用的数据 | 说明 |
|---|---|---|
| `getQuat3D()` | 陀螺仪 | 纯陀螺仪积分，短期平滑，长期会漂移 |
| `getQuat6D()` | 陀螺仪 + 加速度计 | 可校正 roll/pitch，yaw 会漂移 |
| `getQuat9D()` | 陀螺仪 + 加速度计 + 磁力计 | 可校正 roll/pitch/yaw，磁场异常时会抑制磁力计影响 |

### 5.2 四元数转欧拉角

下面代码将四元数转换为 roll、pitch、yaw：

```c
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

void quatToEuler(const vqf_real_t q[4],
                vqf_real_t *roll,
                vqf_real_t *pitch,
                vqf_real_t *yaw)
{
    const vqf_real_t w = q[0];
    const vqf_real_t x = q[1];
    const vqf_real_t y = q[2];
    const vqf_real_t z = q[3];

    vqf_real_t sinPitch = 2.0f * (w * y - z * x);
    if (sinPitch > 1.0f) {
        sinPitch = 1.0f;
    } else if (sinPitch < -1.0f) {
        sinPitch = -1.0f;
    }

    *roll = atan2f(2.0f * (w * x + y * z),
                   1.0f - 2.0f * (x * x + y * y));
    *pitch = asinf(sinPitch);
    *yaw = atan2f(2.0f * (w * z + x * y),
                  1.0f - 2.0f * (y * y + z * z));

    *roll *= RAD_TO_DEG;
    *pitch *= RAD_TO_DEG;
    *yaw *= RAD_TO_DEG;
}
```

输出单位为度。若需要弧度，不要乘以 `RAD_TO_DEG`。

欧拉角存在旋转顺序和坐标系约定问题。若角度方向与实际安装方向相反，应优先检查传感器轴映射、轴符号和四元数使用约定。

## 6. 磁力计航向修正

```c
vqf_real_t getDelta(void);
```

`getDelta()` 返回磁力计对航向的修正量，单位为弧度。

没有磁力计时：

```c
getQuat6D(quat);
```

可以得到稳定的 roll/pitch，但 yaw 只由陀螺仪积分得到，必然会随时间漂移。静止检测和陀螺零偏估计只能降低漂移速度，不能提供绝对航向。

有磁力计且完成校准后：

```c
updateMag(mag);
getQuat9D(quat);
```

VQF 会使用磁力计建立航向参考，并在检测到磁场干扰时降低或关闭磁力计修正。

## 7. 静止检测

源码内置静止检测，默认参数包括：

```c
restBiasEstEnabled = true;
restMinT = 1.5f;
restFilterTau = 0.5f;
restThGyr = 2.0f;
restThAcc = 0.5f;
```

静止检测同时参考：

- 陀螺仪变化；
- 陀螺仪低通结果；
- 加速度变化；
- 连续静止时间。

默认需要连续约 `1.5 s` 满足条件，才会标记为静止。

静止检测依赖：

```c
updateGyr(gyr);
updateAcc(acc);
```

不能只调用 `updateGyr()` 期待完整的静止判断。

## 8. 陀螺仪零偏估计

静止时，VQF 会把低通后的陀螺仪值作为零偏观测，更新内部偏置估计。

```c
vqf_real_t bias[3];
vqf_real_t sigma;

sigma = getBiasEstimate(bias);
```

结果：

- `bias[0..2]`：陀螺仪零偏估计，单位 `rad/s`；
- 函数返回值 `sigma`：偏置估计标准差，单位 `rad/s`。

上电后建议将设备静止放置几秒：

```c
initVqf(0.001f, 0.01f, 0.02f);

// 持续读取传感器并调用 updateGyr/updateAcc
// 设备保持静止约 2~5 秒
```

这不是必须的单独校准流程，VQF 会在运行中持续估计偏置；但启动时保持静止有利于快速得到较好的初始偏置。

## 9. 扩展状态接口

以下函数已经在 `vqf/vqf.c` 中实现，但当前 `vqf/vqf.h` 没有完整声明。若要从其他 C 文件调用，应将声明补充到 `vqf/vqf.h` 中。

### 9.1 静止状态

```c
bool getRestDetected(void);
```

返回：

- `true`：当前判断为静止；
- `false`：当前判断为运动或尚未达到静止时间。

### 9.2 静止偏差

```c
void getRelativeRestDeviations(vqf_real_t out[2]);
```

返回相对于阈值的静止偏差：

```text
out[0]：陀螺仪偏差 / restThGyr
out[1]：加速度偏差 / restThAcc
```

小于 `1.0` 通常表示低于对应阈值。

### 9.3 磁场状态

```c
bool getMagDistDetected(void);
vqf_real_t getMagRefNorm(void);
vqf_real_t getMagRefDip(void);
void setMagRef(vqf_real_t norm, vqf_real_t dip);
```

- `getMagDistDetected()`：是否检测到磁场干扰；
- `getMagRefNorm()`：磁场参考模长；
- `getMagRefDip()`：磁场参考倾角，单位弧度；
- `setMagRef()`：手动设置磁场参考值。

### 9.4 偏置和参数控制

```c
void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma);
void setMotionBiasEstEnabled(bool enabled);
void setRestBiasEstEnabled(bool enabled);
void setMagDistRejectionEnabled(bool enabled);
void setTauAcc(vqf_real_t tauAcc);
void setTauMag(vqf_real_t tauMag);
void setRestDetectionThresholds(vqf_real_t thGyr,
                                vqf_real_t thAcc);
void resetState(void);
```

单位要求：

- `setBiasEstimate()` 的 `bias`：`rad/s`；
- `setBiasEstimate()` 的 `sigma`：`rad/s`；
- `setTauAcc()`、`setTauMag()`：秒；
- `setRestDetectionThresholds()` 的 `thGyr`：`rad/s`；
- `setRestDetectionThresholds()` 的 `thAcc`：与 `acc` 输入相同的单位。

调用 `initVqf()` 会重新执行默认参数初始化。因此如果需要自定义参数，应在 `initVqf()` 之后设置。

## 10. BMI088 调用示例

BMI088 本身包含陀螺仪和加速度计，但不包含磁力计。没有外部磁力计时只能使用 6D 姿态。

```c
#include <math.h>
#include "vqf.h"

#define DEG_TO_RAD 0.017453292519943295f

static vqf_real_t gyr[3];
static vqf_real_t acc[3];
static vqf_real_t quat[4];

void attitude_init(void)
{
    // 示例：陀螺仪 1000 Hz，加速度计 100 Hz
    initVqf(0.001f, 0.01f, 0.01f);
}

void attitude_update(float gx_dps, float gy_dps, float gz_dps,
                     float ax_mps2, float ay_mps2, float az_mps2)
{
    gyr[0] = gx_dps * DEG_TO_RAD;
    gyr[1] = gy_dps * DEG_TO_RAD;
    gyr[2] = gz_dps * DEG_TO_RAD;

    acc[0] = ax_mps2;
    acc[1] = ay_mps2;
    acc[2] = az_mps2;

    updateGyr(gyr);
    updateAcc(acc);

    getQuat6D(quat);
}
```

如果增加外部磁力计：

```c
vqf_real_t mag[3];

mag[0] = mx_uT;
mag[1] = my_uT;
mag[2] = mz_uT;

updateMag(mag);
getQuat9D(quat);
```

## 11. 传感器校准要求

VQF 内部可以估计陀螺仪运行时零偏，但不能代替完整的传感器硬件校准。

建议在使用前完成：

1. 陀螺仪零偏校准；
2. 加速度计零偏和比例因子校准；
3. 传感器轴方向确认；
4. 磁力计硬铁校准；
5. 磁力计软铁校准；
6. 真实采样周期确认。

不需要为每台设备重新标定 `tauAcc`、`tauMag` 等算法参数，除非项目对响应速度、噪声或动态性能有特殊要求。

## 12. 常见错误

### 把频率当成周期

错误：

```c
initVqf(1000.0f, 100.0f, 50.0f);
```

正确：

```c
initVqf(0.001f, 0.01f, 0.02f);
```

### 直接传入 deg/s

错误：

```c
updateGyr(gyr_dps);
```

正确：

```c
gyr[i] = gyr_dps[i] * DEG_TO_RAD;
updateGyr(gyr);
```

### 没有磁力计却期待 yaw 长期不漂

`getQuat6D()` 没有绝对航向参考，yaw 长期漂移是正常现象。需要外部磁力计、GPS 航向、视觉或其他绝对航向信息。

### 只更新陀螺仪

只调用 `updateGyr()` 时，无法完成加速度校正、静止检测和完整的陀螺零偏估计。应根据实际数据到达调用：

```c
updateGyr(gyr);
updateAcc(acc);
```

### 使用扩展函数但头文件没有声明

如果调用 `getRestDetected()`、`getMagDistDetected()` 等扩展接口，请先在头文件中添加对应声明，并确保声明与实现一致。
