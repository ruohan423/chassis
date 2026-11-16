// 二维二次约束规划求解，移植自 RMCS-main 的（菱形约束版本）。
#ifndef SENTRY_CHASSIS_QCP_SOLVER_H
#define SENTRY_CHASSIS_QCP_SOLVER_H

#include <stdint.h>

typedef struct
{
    float x;
    float y;
} qcp_point_t;

#define QCP_MAX_VERTICES 6U
#define QCP_RHOMBUS_MAX_VERTICES 5U

typedef struct
{
    qcp_point_t vertices[QCP_MAX_VERTICES];
    uint32_t count;
} qcp_polygon_t;

typedef struct
{
    float q00;
    float q01;
    float q11;
    float p0;
    float p1;
    float r;
} qcp_quadratic_t;

/* 求解器工作区由调用方持有，避免在实时任务栈上创建顶点和约束临时数据。 */
typedef struct
{
    qcp_polygon_t polygon;
    qcp_quadratic_t quadratic;
    qcp_point_t rhombus_vertices[QCP_RHOMBUS_MAX_VERTICES];
} qcp_workspace_t;

/* 在可行域内最大化 objective.x * x + objective.y * y，可行域为以下约束的交集：
 *   边界：x <= x_max，y <= y_max；
 *   菱形：|x| / rhombus_right + |y| / rhombus_top <= 1，两个半轴必须为正；
 *   二次：q[0] x² + q[1] xy + q[2] y² + q[3] x + q[4] y + q[5] <= 0。
 * 二次约束无可行点时返回零点。quadratic 指向调用期间有效的 6 个只读系数，workspace 由调用方持有且仅供本次调用使用。 */
qcp_point_t qcp_solve(qcp_point_t objective, float x_max, float y_max, float rhombus_right, float rhombus_top,
                      const float *quadratic, qcp_workspace_t *workspace);

#endif
