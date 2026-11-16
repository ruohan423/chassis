/*
 * rls.h - 2 维递推最小二乘，
 * 2x2 矩阵运算标量展开，不依赖矩阵库。
 */
#ifndef SENTRY_CHASSIS_RLS_H
#define SENTRY_CHASSIS_RLS_H

#include <stdint.h>

#define RLS_DIM 2

typedef struct
{
    uint32_t dimension;                  /* RLS 参数维数 */
    float lambda;                        /* 遗忘因子 */
    float delta;                         /* 协方差矩阵初始尺度 */

    uint32_t lastUpdate;                 /* 上次更新时间 */
    uint32_t updateCnt;                  /* 累计更新次数 */

    float transMatrix[RLS_DIM][RLS_DIM]; /* 协方差矩阵 */
    float gainVector[RLS_DIM];           /* 参数更新增益 */
    float paramsVector[RLS_DIM];         /* 当前参数 */
    float defaultParamsVector[RLS_DIM];  /* 复位时恢复的默认参数 */
    float output;                        /* 最近一次估计输出，当前工程未使用 */
} RLS;

/* r 为调用方持有的可写状态，必须在所有 RLS 调用期间持续有效，模块不会另存其指针。 */
void RLS_construct(RLS *r, float delta_, float lambda_);

/* initParam 为调用方持有的 RLS_DIM 个只读初值，仅在调用期间借用。 */
void RLS_constructWithParam(RLS *r, float delta_, float lambda_, const float *initParam);
void RLS_reset(RLS *r);
/* sampleVector 为 RLS_DIM 个只读样本，仅在调用期间借用；返回指针归 r 所有，在 r 生命周期内有效，后续更新会改变内容。 */
const float *RLS_update(RLS *r, const float *sampleVector, float actualOutput, uint32_t nowTick);
/* updatedParams 为 RLS_DIM 个只读参数，仅在调用期间借用并复制到 r。 */
void RLS_setParamVector(RLS *r, const float *updatedParams);
/* 返回 r 内部只读参数数组，不转移所有权；r 被修改后内容可能变化。 */
const float *RLS_getParamsVector(const RLS *r);
float RLS_getOutput(const RLS *r);

#endif
