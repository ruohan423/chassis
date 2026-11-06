# 把 RM2024 功率控制移植到我的工程

## 背景
`/home/mot/work/power_ctrl_c/` 是 RM2024-PowerModule 麦轮/全向轮功率控制的纯 C 逐行移植，已脱离 FreeRTOS 和 C++。
先读它的 `README.md`，再读 `PowerController.h`、`ChassisController.h`、`AbstractFeedbackMotor.h`，理解接口后再动我的工程。

## 我的工程信息（请先核对，缺的自己在工程里找）
- 工程路径：`<填路径>`
- 底盘类型：<麦轮 / 全向轮>，轮径 <m>，前后轮距 <m>，左右轮距 <m>（全向填轮到中心距离）
- 底盘电机：<如 M3508+C620>，KA <Nm/A>，减速比 <>，电流限幅 <A>，指令限幅 <如 16384>
- 兵种：<INFANTRY / HERO / SENTRY>
- 是否有超级电容通讯：<有 / 无>
- RTOS 与 tick：<FreeRTOS, configTICK_RATE_HZ=1000 / 其他>
- 现有底盘任务所在文件：<填路径>

## 任务
1. **摸清我的工程**：找到并记下
   - 电机驱动：读 rpm、读电流（或力矩）、读在线/断连计数、设输出 的函数和结构体
   - PID 实现：运行函数、复位函数、输出限幅字段
   - 裁判系统数据：`chassis_power_limit`、`robot_level`、`buffer_energy`、`chassis_power` 对应的字段和在线判断
   - 超级电容数据（若有）：在线标志、DCDC 使能、错误码、剩余能量、底盘功率
   - 底盘任务里 "逆解 → PID → 下发电机" 的那段代码
2. **拷入文件**：`PowerController.h/.c`、`RLS.h/.c`、`AbstractFeedbackMotor.h`；需要整套底盘控制时再拷 `ChassisController.h/.c`。
   `PID.h/.c` 不要拷，改用我工程的 PID：在 `PowerController.c` 和 `ChassisController.c` 里把 `PID_run / PID_reset / ->outputMax` 换成我的等价调用，`PID` 类型换成我的 PID 结构体。
3. **电机适配**：为 4 个底盘电机各写一张 `AbstractFeedbackMotor` 函数指针表。单位必须对齐：
   `getTorqueFeedback` 返回 Nm = 电流(A) × KA × 减速比；`getRPMFeedback` 返回 rpm；`getKA` 返回 Nm/A（不含减速比）；`getReductionRatio`、`getCurrentLimit`(A)、`getOutputLimit`(指令单位) 按我的电机填。
   轮序固定 0 右前、1 左前、2 左后、3 右后，X 向右、Y 向前、逆时针为正；若我的编号或坐标系不同，只在逆解处换行，不要改功率环。
4. **反馈适配**：填 `PowerFeedbackSources`。若我的裁判/电容结构体字段类型与 `RefereePowerHeatMessageData`、`RefereeRobotStatusMessageData`、`CapacitorStatus` 不一致，写一层转换，每周期拷到这三个结构体里，不要改 `PowerController.c` 的读取逻辑。
   没有电容就定义 `USE_SUPER_CAPACITOR 0`。
5. **任务接线**：
   - 新建或复用一个 1 kHz 任务：启动后延时 1 s → `Power_daemonInit(tick)` → 循环 `Power_daemonStep(tick)` 每 1 ms。把 `POWER_TICK_RATE_HZ` 定义成我的 tick 频率。
   - 底盘任务：在轮速 PID 之后、下发电机之前插入 `Power_getControlledOutput()`，参考 `ChassisController_update()` 里填 `PowerObj` 的四行。若直接用 `ChassisController_*`，把原来的逆解和 PID 调用替换掉。
   - 初始化顺序：`Manager_construct` → `Power_init` → 创建任务。初值用 k1=0.22、k2=1.2、k3=2.78、lambda=0.9999。
6. **编译通过**，然后给我一份清单：改了哪些文件、每处改动对应原实现的哪一行、哪些接口没能一一对应及其原因。

## 约束
- 不改算法逻辑，不改常量值，不改函数名和字段名；只做接口适配。
- 遇到工程里找不到的信息（如 KA、减速比、裁判字段名）先列出来问我，不要猜。
- 不要删除或"优化"原实现里看起来多余的分支（如断连电机 1 s 内继续计功率、RLS 粘滞关闭），那是有意为之。
- 适配完成前不要动 `adapted/` 目录，那是另一版参考实现。
