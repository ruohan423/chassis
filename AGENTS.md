# Embedded C Project Coding Guidelines

## Task Code Style

任务代码应参考工程中现有的代表性任务实现，优先保证主流程直观、调用目的明确，不为了形式上的封装增加不必要的函数层级。当前工程的 `user/ins_task.c` 只是参考示例；如果新工程不存在该文件，应选择结构清晰、维护状态较好的任务作为风格参照，不要求文件名或目录一致。

### Task Function

任务入口函数应直接展示以下内容：

1. 初始化模块状态。
2. 初始化外设或接收模块。
3. 等待事件或处理周期任务。
4. 执行超时和故障处理。
5. 进入下一次循环。

任务函数应让读者直接看出任务做什么、何时处理数据、何时清理状态。复杂的 DMA、串口和协议解析实现应放到对应功能模块目录，不要全部堆积在任务入口文件中。目录和文件名应根据目标工程现有结构确定，例如当前工程使用 `user/remote/`，新工程不需要复用该名称。

## CMSIS-RTOS2

工程使用 CMSIS-RTOS2 封装。应用代码不要直接使用 FreeRTOS 原生 API。

使用：

- `#include "cmsis_os2.h"`
- `osKernelGetTickCount()`
- `osDelayUntil()`
- `osSemaphoreAcquire()`
- `osSemaphoreRelease()`

不要在任务业务代码中直接使用：

- `FreeRTOS.h`
- `task.h`
- `xTask*`
- `vTask*`
- `pdMS_TO_TICKS()`

对外任务入口名称和硬件回调名称保持目标工程已有约定不变。下面的名称仅是当前工程示例，新工程应使用实际存在的接口名称：

- `Example_Task`
- `HAL_UARTEx_RxEventCallback`
- `HAL_UART_ErrorCallback`

## DMA/UART Receive Pattern

对于使用 UART 接收协议数据的模块，优先使用以下结构。这里的“遥控器”、UART 编号、帧长度和模块名称都是当前工程示例，新工程应替换为实际设备和接口：

- UART RX DMA 使用循环模式 `DMA_CIRCULAR`。
- 使用 UART 空闲中断 `IDLE` 处理不定长数据。
- DMA 缓冲区使用合理余量，不能只按单帧长度分配。
- HAL 回调只负责记录必要的 DMA 环绕状态和释放信号量。
- 任务中直接调用 `osSemaphoreAcquire()`，使等待接收事件的逻辑可见。
- 信号量只表示“DMA 中有新的数据需要处理”，不能作为帧内容或数据长度来源。
- 任务被唤醒后根据 DMA 当前写位置读取所有新增字节。
- HAL 回调传入的 `Size` 不作为循环 DMA 的唯一数据边界。
- 信号量等待时间与遥控链路超时时间必须分开定义，不能把 100ms 链路超时直接隐藏在接收处理函数内部。
- 没有固定任务周期要求时，任务应阻塞等待信号量，而不是忙等或人为增加固定延时。

推荐结构：

```c
for (;;)
{
    if (osSemaphoreAcquire(rx_semaphore, RX_SEMAPHORE_WAIT_MS) == osOK)
    {
        uart_receive_process();
    }

    if (uart_receive_is_timeout(RX_LINK_TIMEOUT_MS))
    {
        control_input_clear();
        uart_receive_clear_state();
    }
}
```

信号量等待时间只用于让任务周期性检查链路超时；链路超时常量只表示最后一次有效帧距当前的时间。两者语义不能混用。

## Function Decomposition

减少函数数量，避免过度拆分调用链。

只有以下情况才新增内部函数：

- 函数具有明确且独立的职责。
- 函数包含完整的错误恢复流程。
- 函数被多个位置复用。
- 函数能够明显提高调用者的可读性。
- 函数用于隔离复杂的底层实现。

以下情况不要单独创建函数：

- 只有一行赋值。
- 只有一次简单返回。
- 只有一条简单计算。
- 只被调用一次且不能提升语义。
- 只是机械转发另一个函数。

例如不要创建：

```c
static void reset_parser(void)
{
    frame_length = 0U;
}
```

直接在使用位置写：

```c
frame_length = 0U;
```

DMA 写位置计算、数据读取、帧拼接和帧校验可以合并到一个职责明确的接收处理函数中，让调用者直接表达“处理收到的数据”，避免以下过深调用链：

```text
process_dma_data -> get_dma_write_count -> get_dma_write_position -> parse_byte -> publish_frame
```

## Source File Layout

- 任务文件先放任务入口函数，使初始化和主循环作为本文件的主要流程优先可见。
- 任务入口需要调用本文件内部辅助函数时，在入口前放简短的 `static` 前置声明；辅助函数实现放在任务入口之后。
- 对外任务入口和硬件回调按工程既有接口要求保留位置和名称。

## Naming

内部函数和变量使用小写下划线命名，优先使用完整语义名称：

- `process_received_data`
- `restart_uart`
- `frame_length`
- `read_count`
- `last_frame_tick`
- `has_received_frame`
- `uart_error_pending`
- `dma_wrap_count`

不要在同一模块中混用 `RC_*`、`REMOTE_*`、`rc_*`、`Remote_*` 等多套命名体系。对外接口名称可以根据既有工程约定保留。

命名应简洁且能表达职责，避免重复上下文或堆叠过多限定词。优先使用工程已有模块前缀；在模块边界清楚时，可使用稳定、易懂的缩写，例如 `gyro_cal_push`、`cal_session_t`、`TEMP_CTRL_DT`。单位或物理量仍需在名称、类型或就近注释中明确。

## Formatting

## 注释语言

- 工程自有代码中的新增或修改注释必须使用中文。
- 注释中的函数名、宏名、类型名、寄存器名、协议字段名和标准单位可以保留英文或代码原文。
- 第三方库、许可证文本以及由 CubeMX 重新生成的上游注释不要求翻译；不要为了翻译它们产生大范围无关修改。

- 函数左大括号单独换行。
- `if`、`for`、`while` 的大括号单独换行。
- 一行代码不要无必要拆成多行。
- 简单函数调用、返回、赋值和条件判断保持单行；只有表达式确实过长或影响可读性时才换行。
- 不要为了格式统一拆开短表达式。
- 不要把只有一条简单语句的函数人为拆成多行或额外封装。
- 注释只解释协议限制、硬件行为或不明显的异常处理原因。
- 不添加描述显而易见代码的注释。

## Required Reliability

简化代码时不得删除以下保护。具体保护项应根据协议和硬件能力调整，但不能为了减少代码而删除必要的边界检查和故障恢复：

- DMA 环形缓冲区。
- DMA 溢出检测。
- 完整帧校验。
- 无效帧后的滑动窗口重同步。
- UART 错误恢复。
- UART DMA 启动失败后的重试。
- 输入链路超时后清零或切换到安全控制状态。
- 多帧或半帧在一次任务处理中的正确处理。

当输入协议没有可靠同步字时，收到无效帧后应保留滑动窗口重同步逻辑。以下代码中的计数器和帧长度宏均为占位符，应替换为目标工程已有定义：

```c
frame_invalid_count++;
memmove(frame, &frame[1], FRAME_LENGTH - 1U);
frame_length = FRAME_LENGTH - 1U;
```

不要保留无法由任务调度准确判断的帧间隔统计。任务唤醒时间不等于实际帧到达时间，帧间隔统计容易产生误报。

## Sensor and Periodic Task Notes

传感器任务和其他固定周期任务也应保持代表性任务实现的风格。当前工程的 `ins_task.c` 只是示例；新工程应参考本项目中已有的清晰任务，不要求存在同名文件：

- 初始化滤波器或传感器。
- 在循环中直接展示采样、更新算法和输出结果。
- 使用固定周期时使用 `osDelayUntil()`，避免周期漂移。
- 临时计算数组或局部变量放在实际使用范围内。
- 不输出没有正确物理意义的派生量。
- 如果某个字段的计算逻辑不正确或没有可靠定义，应删除字段及其赋值，而不是保留错误结果。

例如，某个线性加速度字段如果没有完成重力分量去除和坐标系定义，不应继续保留为世界坐标加速度的简单复制。

## CubeMX Generated Code

不要直接修改会被 STM32CubeMX 重新生成覆盖的配置或生成代码。

涉及以下内容时，优先判断是否属于 CubeMX 生成范围：

- `.ioc` 配置文件。
- `Core/Src/` 和 `Core/Inc/` 中由 CubeMX 生成的外设初始化代码。
- FreeRTOS、UART、DMA、GPIO、时钟和中断初始化代码。
- `/* USER CODE BEGIN */` / `/* USER CODE END */` 之外的生成代码。
- CMake 或工程文件中由 CubeMX 重新生成维护的内容。

如果需求需要修改 CubeMX 配置，不要直接编辑这些生成文件。应先明确提示用户在 CubeMX 中进行配置，例如：

- 需要启用或修改哪个外设。
- 需要配置哪个 UART、DMA、NVIC 或 FreeRTOS 对象。
- DMA 需要设置为 `Circular` 还是 `Normal` 模式。
- UART 是否需要启用 `IDLE` 接收中断。
- 需要创建的信号量、线程或其他 RTOS 对象及其初始值、最大值和属性。
- 需要重新生成代码，以及重新生成后需要在 `USER CODE` 区域补充哪些逻辑。

只有以下情况可以直接修改生成文件：

- 修改位于明确的 `USER CODE BEGIN` / `USER CODE END` 区域内。
- 用户明确要求直接修改生成文件，并接受下一次 CubeMX 重新生成可能覆盖修改的风险。
- 修改不属于 CubeMX 配置和生成职责，例如独立用户模块的实现文件。

当无法确定文件是否会被 CubeMX 覆盖时，不要直接修改。应先检查 `.ioc`、生成文件注释和 CMake 配置，并向用户说明需要在 CubeMX 中修改的项目和步骤。

### Change Reporting

如果发现当前需求需要 CubeMX 配置变更，应在回复中明确说明：

1. 当前代码不能安全直接修改的原因。
2. 需要用户在 CubeMX 图形界面中修改的具体配置。
3. 修改后需要重新生成哪些文件或代码。
4. 重新生成后，代码实现应放在哪些 `USER CODE` 区域或独立用户文件中。

不要静默修改 `.ioc` 或生成代码后将其当作普通业务代码提交。

## Verification

完成任务代码修改后至少执行：

```text
cmake --build build/Debug --parallel 4
git diff --check
```

同时检查：

- 新增源文件是否加入 `CMakeLists.txt`。
- 是否残留直接使用 FreeRTOS API 的代码。
- 是否存在无用函数、无用宏或无用状态变量。
- 任务入口是否能直接表达业务流程。
- 信号量等待和链路超时是否语义分离。
- DMA、UART 错误和帧重同步保护是否仍然存在。
