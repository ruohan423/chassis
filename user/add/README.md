# RM2024-PowerModule 麦轮/全向轮功率控制 —— 纯 C 逐行移植

目标：**尽可能保留原实现**。类名、字段名、常量名、宏开关、注释、分支结构全部沿用原 C++ 代码，
只做 C 语言化和去 FreeRTOS 化，后续接入具体工程时再做适配。

## 文件对应

| 本目录 | 原工程 | 说明 |
|---|---|---|
| `PowerController.h/.c` | `Mecanum_and_Omni/PowerController.hpp/.cpp` | 功率环 + 能量环 + RLS + 故障处理，逐段对应 |
| `ChassisController.h/.c` | `Mecanum_and_Omni/ChassisController.hpp/.cpp` | 运动学 + 轮速 PID + 调用功率环，逐段对应 |
| `RLS.h/.c` | `Utils/RLS.hpp` (RLS<2>) | 2 维标量展开，字段名与原模板类一致，不再依赖 `Matrix.hpp` |
| `PID.h/.c` | `PID.hpp`（原仓库未包含） | 按用法最小实现，接入时直接换成你自己的 PID |
| `AbstractFeedbackMotor.h` | `AbstractFeedbackMotor.hpp`（原仓库未包含） | 虚函数 → 函数指针表 |
| `adapted/` | | 之前那版改动较多的简化移植，仅供参考 |

## C++ → C 的机械映射

| 原 C++ | 本移植 |
|---|---|
| `namespace Core::Control::Power` 内函数 `f()` | `Power_f()` |
| `struct Manager` / `PowerObj` / `PowerStatus` | 同名 struct，字段名不变 |
| `Manager::Manager(...)` | `Manager_construct(...)` |
| `Manager::Motors::operator[](i)` | `Motors_at(&motors, i)` |
| `AbstractFeedbackMotor::getXxx()` 虚函数 | `motor->getXxx(motor)` 函数指针 |
| `PID pid; pid(set, fb)` | `PID_run(&pid, set, fb)` |
| `Matrixf<2,1>` / `Matrixf<2,2>` | `float[2]` / `float[2][2]` |
| `xTaskGetTickCount()` | 由调用方传入 `nowTick` |
| `configTICK_RATE_HZ` | `POWER_TICK_RATE_HZ` / `CHASSIS_TICK_RATE_HZ`，默认 1000 |
| `powerDaemon` 任务 | `Power_daemonInit(now)` 一次 + `Power_daemonStep(now)` 每 1 ms |
| `RefereeSystem::subscribeMessage` / `SuperCapacitor::getStatus()` | `PowerFeedbackSources` 里的只读指针 |
| `#if USE_SUPER_CAPACITOR` / `USE_DEBUG` / `USE_MECANUM_CHASSIS` | 保留，同名宏 |

## 与原实现仅有的差异

1. **整数除法**：原 `(now - lastUpdateTick) / configTICK_RATE_HZ` 为整数除法，1 kHz 下恒为 0。
   默认改为浮点秒；定义 `POWER_KEEP_ORIGINAL_INT_DIV 1` 可完全复刻原行为。
2. **RLS 断言**：原 `configASSERT(lambda >= 0 || lambda <= 1)` 恒真，改为 `&&`。
3. **空指针保护**：断连电机分支增加 `motor != NULL` 判断，避免原代码在电机指针为空时解引用。
4. **缺失分号**：原 `#else` 分支 `manager.estimatedCapEnergy = 0.0f` 缺分号（该分支原本不编译），已补。
5. **PID**：原 `PID.hpp` 不在仓库，`PID::Param` 9 个参数含义按用法推断（Kp, Ki, Kd, ?, outputMax, ?, ?, alpha, period）。

其余逻辑、常量、分支顺序、粘滞关闭 RLS、电容离线能量积分、回调接口等均未改动。

## 接入骨架

```c
/* 1. 电机函数指针表 ×4, 你的 PID ×4 */
/* 2. 裁判/电容数据由通讯层写入下面三块内存, 只读指针交给功率模块 */
static RefereePowerHeatMessageData   powerMsg;
static RefereeRobotStatusMessageData robotMsg;
static CapacitorStatus               capStat;
static uint8_t refereeOnline(void) { return ...; }

Motors motors = { &mRf, &mLf, &mLb, &mRb };
Manager mana;
Manager_construct(&mana, &motors, INFANTRY, Enable, 0.22f, 1.2f, 2.78f, 0.9999f);
PowerFeedbackSources src = { &powerMsg, &robotMsg, &capStat, refereeOnline };
Power_init(&mana, &src);

/* 功率任务 */
void powerTask(void) { delay_ms(1000); Power_daemonInit(tick()); for (;;) { Power_daemonStep(tick()); delay_ms(1); } }

/* 底盘任务 */
ChassisController chassis; /* ChassisConfigs_construct + ChassisController_construct */
void chassisTask(void) { for (;;) { ChassisController_settargetVelocity(&chassis, vx, vy, vw);
                                    ChassisController_update(&chassis, tick()); delay_ms(1); } }
```
