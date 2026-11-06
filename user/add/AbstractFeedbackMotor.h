/* AbstractFeedbackMotor.h - 原工程的电机抽象基类, 这里用函数指针表代替虚函数.
 * 接入工程: 为每个电机填一张表, ctx 指向你的电机对象.
 */
#ifndef ABSTRACT_FEEDBACK_MOTOR_H
#define ABSTRACT_FEEDBACK_MOTOR_H
#include <stdint.h>

typedef struct AbstractFeedbackMotor {
    void *ctx;
    float    (*getTorqueFeedback)(const struct AbstractFeedbackMotor *m);   /* Nm, 含减速箱 */
    float    (*getRPMFeedback)(const struct AbstractFeedbackMotor *m);      /* rpm */
    float    (*getPositionFeedback)(const struct AbstractFeedbackMotor *m); /* rad (舵轮用, 麦轮可为 NULL) */
    uint32_t (*getDisconnectCounter)(const struct AbstractFeedbackMotor *m);/* 连续未收到反馈的次数 */
    float    (*getCurrentLimit)(const struct AbstractFeedbackMotor *m);     /* A */
    float    (*getOutputLimit)(const struct AbstractFeedbackMotor *m);      /* 指令单位上限, 如 16384 */
    float    (*getKA)(const struct AbstractFeedbackMotor *m);               /* Nm/A, 不含减速比 */
    float    (*getReductionRatio)(const struct AbstractFeedbackMotor *m);
    void     (*setOutput)(struct AbstractFeedbackMotor *m, float output);   /* 指令单位 */
} AbstractFeedbackMotor;

#endif
