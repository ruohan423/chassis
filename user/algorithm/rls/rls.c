/* rls.c
 *此移植标量展开.
 */
#include "rls.h"

#include <assert.h>

static void RLS_validate(const RLS *r)
{
    assert(r->lambda >= 0.0f && r->lambda <= 1.0f);
    assert(r->delta > 0.0f);
}


void RLS_reset(RLS *r)
{
    int i, j;
    for (i = 0; i < RLS_DIM; i++) {
        for (j = 0; j < RLS_DIM; j++) r->transMatrix[i][j] = (i == j) ? r->delta : 0.0f;  /* eye() * delta */
        r->gainVector[i]   = 0.0f;
        r->paramsVector[i] = 0.0f;
    }
}

void RLS_construct(RLS *r, float delta_, float lambda_)
{
    int i;
    r->dimension  = RLS_DIM;
    r->lambda     = lambda_;
    r->delta      = delta_;
    r->lastUpdate = 0;
    r->updateCnt  = 0;
    r->output     = 0.0f;
    for (i = 0; i < RLS_DIM; i++) r->defaultParamsVector[i] = 0.0f;
    RLS_reset(r);
    RLS_validate(r);
}

void RLS_constructWithParam(RLS *r, float delta_, float lambda_, const float initParam[RLS_DIM])
{
    int i;
    RLS_construct(r, delta_, lambda_);
    for (i = 0; i < RLS_DIM; i++) r->defaultParamsVector[i] = initParam[i];
}

/* gainVector   = (P*phi) / (1 + phi'*P*phi / lambda) / lambda
 * paramsVector += gainVector * (actualOutput - phi'*paramsVector)
 * transMatrix  = (P - gainVector*phi'*P) / lambda
 */
const float *RLS_update(RLS *r, const float sampleVector[RLS_DIM], float actualOutput, uint32_t nowTick)
{
    const float *phi = sampleVector;
    float (*P)[RLS_DIM] = r->transMatrix;
    float Pphi[RLS_DIM], phiP[RLS_DIM];
    float phiPphi = 0.0f, pred = 0.0f, denom;
    int i, j;

    for (i = 0; i < RLS_DIM; i++) {
        Pphi[i] = 0.0f;
        for (j = 0; j < RLS_DIM; j++) Pphi[i] += P[i][j] * phi[j];
        phiPphi += phi[i] * Pphi[i];
        pred    += phi[i] * r->paramsVector[i];
    }
    denom = (1.0f + phiPphi / r->lambda) * r->lambda;
    for (i = 0; i < RLS_DIM; i++) r->gainVector[i] = Pphi[i] / denom;

    for (i = 0; i < RLS_DIM; i++) r->paramsVector[i] += r->gainVector[i] * (actualOutput - pred);

    for (j = 0; j < RLS_DIM; j++) {
        phiP[j] = 0.0f;
        for (i = 0; i < RLS_DIM; i++) phiP[j] += phi[i] * P[i][j];
    }
    for (i = 0; i < RLS_DIM; i++)
        for (j = 0; j < RLS_DIM; j++)
            P[i][j] = (P[i][j] - r->gainVector[i] * phiP[j]) / r->lambda;

    r->updateCnt++;
    r->lastUpdate = nowTick;
    return r->paramsVector;
}

void RLS_setParamVector(RLS *r, const float updatedParams[RLS_DIM])
{
    int i;
    for (i = 0; i < RLS_DIM; i++) {
        r->paramsVector[i]        = updatedParams[i];
        r->defaultParamsVector[i] = updatedParams[i];
    }
}

const float *RLS_getParamsVector(const RLS *r) { return r->paramsVector; }
float RLS_getOutput(const RLS *r) { return r->output; }
