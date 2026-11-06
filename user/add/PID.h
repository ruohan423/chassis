/* PID.h - 原工程 PID.hpp 不在仓库内, 这里按其在 PowerController/ChassisController 中的用法做最小实现.
 * 原调用: PID::Param(Kp, Ki, Kd, Kf?, outputMax, integralMax?, deadzone?, alpha, period)
 *         pid(set, fb)            -> 返回输出
 *         pid.getParam().outputMax
 *         pid.reset()
 * 接入你的工程时, 把 PID_run / PID_reset / outputMax 换成你自己的 PID 即可.
 */
#ifndef PID_H
#define PID_H
#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integralMax;
    float outputMax;
    float alpha;        /* D 项一阶低通系数 */
    float dt;           /* 控制周期 s */
    /* 状态 */
    float integral, lastError, dFilt;
    uint8_t first;
} PID;

void  PID_init(PID *p, float kp, float ki, float kd, float integralMax, float outputMax, float alpha, float dt);
float PID_run(PID *p, float set, float fb);
void  PID_reset(PID *p);

#endif
