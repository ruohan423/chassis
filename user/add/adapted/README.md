# power_ctrl_c

RM2024-PowerModule（ENTERPRIZE 战队）麦轮/全向轮功率控制的纯 C 移植。
无 FreeRTOS、无 C++、无矩阵库依赖，只需 `<math.h>`。

| 文件 | 对应原工程 | 内容 |
|---|---|---|
| `rls2.h/.c` | `Utils/RLS.hpp` (RLS<2>) + `Matrix.hpp` | 2 参数递归最小二乘，2×2 标量展开 |
| `power_ctrl.h/.c` | `Mecanum_and_Omni/PowerController.hpp/.cpp` | 电机功率模型、功率环 `pc_limit`、能量环 + RLS + 故障处理 `pc_daemon_update` |
| `chassis_example.c` | `Mecanum_and_Omni/ChassisController.cpp` | 运动学逆解 → 轮速 PID → 功率环 → 电机输出 的接线示例 |

## 接入步骤

1. 把 `rls2.*`、`power_ctrl.*` 加入工程。
2. 参照 `chassis_example.c`，实现 7 个 `extern` 接口（电机 rpm/电流/在线、PID、裁判+电容反馈）。
3. 算 `k0 = KA(Nm/A) × 减速比 × 电流限幅(A) / 指令限幅`，这是把 PID 输出单位换成 Nm 的唯一桥梁。
4. 标定 k3：失能底盘电机，读电容或裁判实时功率的平均值。
5. 起两个 1 kHz 周期：`pc_daemon_update()` 后台任务，`pc_limit()` 放在底盘 PID 之后、下发电机之前。
6. 用 `pc_set_user_max_power()` 实现自己的充电/补电策略；它会被自动夹在能量环算出的 `[full_max_power, base_max_power]` 之间。

## 与原工程的差异

- 电容断连时的「剩余能量积分估算」没有移植：原代码里 `(now - lastTick) / configTICK_RATE_HZ` 是整数除法，1 kHz 下恒为 0，该功能实际从未生效。
- RLS 使能不再是粘滞关闭：原代码在电容断连时把 `rlsEnabled` 永久置 0，这里改为每周期按反馈有效性门控。
- 能量环 PD 用最简实现（Kp=50，Kd=0.2，D 项一阶滤波 α=0.2，输出 ±300 W）。
