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
    uint32_t dimension;                  /* Dimension of the RLS space */
    float lambda;                        /* The forget index */
    float delta;                         /* Intialized value of the transferred matrix */

    uint32_t lastUpdate;                 /* Last update tick */
    uint32_t updateCnt;                  /* Total update Count */

    float transMatrix[RLS_DIM][RLS_DIM]; /* Transfer matrix instance */
    float gainVector[RLS_DIM];           /* Gain vector for params update */
    float paramsVector[RLS_DIM];         /* Params vector */
    float defaultParamsVector[RLS_DIM];
    float output;                        /* Estimated / filtered output (原工程未使用) */
} RLS;

void RLS_construct(RLS *r, float delta_, float lambda_);

void RLS_constructWithParam(RLS *r, float delta_, float lambda_, const float initParam[RLS_DIM]);
void RLS_reset(RLS *r);
/* 返回 paramsVector. nowTick 替代 xTaskGetTickCount() */
const float *RLS_update(RLS *r, const float sampleVector[RLS_DIM], float actualOutput, uint32_t nowTick);
void RLS_setParamVector(RLS *r, const float updatedParams[RLS_DIM]);
const float *RLS_getParamsVector(const RLS *r);
float RLS_getOutput(const RLS *r);

#endif
