#include "rls2.h"

void rls2_init(rls2_t *r, float delta, float lambda, float theta0, float theta1)
{
    r->P[0][0] = delta; r->P[0][1] = 0.0f;
    r->P[1][0] = 0.0f;  r->P[1][1] = delta;
    r->theta[0] = theta0;
    r->theta[1] = theta1;
    r->lambda = lambda;
}

/* 与原工程 RLS::update 完全等价:
 *   K = P*phi / (lambda + phi'*P*phi)
 *   theta += K*(y - phi'*theta)
 *   P = (P - K*phi'*P) / lambda
 */
void rls2_update(rls2_t *r, const float phi[2], float y)
{
    float Pphi0 = r->P[0][0] * phi[0] + r->P[0][1] * phi[1];
    float Pphi1 = r->P[1][0] * phi[0] + r->P[1][1] * phi[1];
    float denom = r->lambda + phi[0] * Pphi0 + phi[1] * Pphi1;
    if (denom <= 0.0f) return;
    float K0 = Pphi0 / denom;
    float K1 = Pphi1 / denom;

    float err = y - (phi[0] * r->theta[0] + phi[1] * r->theta[1]);
    r->theta[0] += K0 * err;
    r->theta[1] += K1 * err;

    float phiP0 = phi[0] * r->P[0][0] + phi[1] * r->P[1][0];
    float phiP1 = phi[0] * r->P[0][1] + phi[1] * r->P[1][1];
    r->P[0][0] = (r->P[0][0] - K0 * phiP0) / r->lambda;
    r->P[0][1] = (r->P[0][1] - K0 * phiP1) / r->lambda;
    r->P[1][0] = (r->P[1][0] - K1 * phiP0) / r->lambda;
    r->P[1][1] = (r->P[1][1] - K1 * phiP1) / r->lambda;
}
