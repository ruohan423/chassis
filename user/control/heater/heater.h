#ifndef SENTRY_CHASSIS_HEATER_H
#define SENTRY_CHASSIS_HEATER_H

#include <stdbool.h>

/* IMU 加热器闭环：TIM10_CH1 PWM 加热，目标温度与 PID 参数见 bias_config.h。
 * heater_update 在温度达到 TEMP_LIMIT_C 时关闭加热并返回 false，调用方应终止依赖加热的流程。 */
bool heater_init(void);
bool heater_update(float temperature_c, float sample_period_s);
void heater_stop(void);

#endif
