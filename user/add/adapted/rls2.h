/* rls2.h - 2 参数递归最小二乘，对应原工程 Utils/RLS.hpp 的 RLS<2>
 * 回归模型: y = theta0*phi0 + theta1*phi1
 * 功率控制中: phi0 = Σ|ω_i|, phi1 = Σ τ_i², y = P_measured - Σ τ_i ω_i - k3
 */
#ifndef RLS2_H
#define RLS2_H

typedef struct {
    float P[2][2];   /* 协方差(传递)矩阵 */
    float theta[2];  /* 估计参数 [k1, k2] */
    float lambda;    /* 遗忘因子, 原工程 0.99999 */
} rls2_t;

void rls2_init(rls2_t *r, float delta, float lambda, float theta0, float theta1);
void rls2_update(rls2_t *r, const float phi[2], float y);

#endif
