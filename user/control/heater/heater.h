#ifndef SENTRY_CHASSIS_HEATER_H
#define SENTRY_CHASSIS_HEATER_H

#include <stdbool.h>

/* IMU 加热器闭环：TIM10_CH1 PWM 加热，目标温度与 PID 参数见 bias_config.h。
 * 使用软目标控制，输出限于 0..HEATER_OUT_MAX，达到目标后允许积分维持保温输出。
 * 初始化和更新由启动校准线程、INS 线程先后独占调用，不允许并发访问。
 * 温度无效、达到 TEMP_LIMIT_C 或采样周期无效时关闭加热并锁存故障，返回 false；
 * 调用方应终止依赖加热的流程，故障仅由 heater_init 清除；heater_stop 只将输出归零。 */
bool heater_init(void);
bool heater_update(float temperature_c, float sample_period_s);
void heater_stop(void);

#endif
