// 二维二次约束规划求解，移植自 RMCS-main 的（菱形约束版本）。
#ifndef SENTRY_CHASSIS_QCP_SOLVER_H
#define SENTRY_CHASSIS_QCP_SOLVER_H

typedef struct
{
    float x;
    float y;
} qcp_point_t;

/* 在可行域内最大化 objective.x * x + objective.y * y，可行域为以下约束的交集：
 *   边界：x <= x_max，y <= y_max；
 *   菱形：|x| / rhombus_right + |y| / rhombus_top <= 1，两个半轴必须为正；
 *   二次：q[0] x² + q[1] xy + q[2] y² + q[3] x + q[4] y + q[5] <= 0。
 * 二次约束无可行点时返回零点。 */
qcp_point_t qcp_solve(qcp_point_t objective, float x_max, float y_max, float rhombus_right, float rhombus_top,
                      const float quadratic[6]);

#endif
