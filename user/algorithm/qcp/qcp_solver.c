#include "qcp_solver.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* 菱形被 y_max 截断后最多 5 个顶点，再被 x_max 裁剪最多增加 1 个。 */
#define QCP_MAX_VERTICES 8U
#define QCP_EPSILON      1e-9f

typedef struct
{
    qcp_point_t vertices[QCP_MAX_VERTICES];
    uint32_t count;
} qcp_polygon_t;

/* 二次约束的矩阵形式：0.5 xᵀQx + pᵀx + r <= 0，Q 对称。 */
typedef struct
{
    float q00;
    float q01;
    float q11;
    float p0;
    float p1;
    float r;
} qcp_quadratic_t;

static void build_polygon(qcp_polygon_t *polygon, float rhombus_right, float rhombus_top, float x_max, float y_max);
static bool point_inside_polygon(qcp_point_t p, const qcp_polygon_t *polygon);
static qcp_point_t best_point_at_intersections(qcp_point_t objective, const qcp_polygon_t *polygon,
                                               const qcp_quadratic_t *quadratic);

qcp_point_t qcp_solve(qcp_point_t objective, float x_max, float y_max, float rhombus_right, float rhombus_top,
                      const float quadratic[6])
{
    qcp_polygon_t polygon;
    build_polygon(&polygon, rhombus_right, rhombus_top, x_max, y_max);

    qcp_quadratic_t q = {2.0f * quadratic[0], quadratic[1], 2.0f * quadratic[2], quadratic[3], quadratic[4], quadratic[5]};
    if (q.q00 < 0.0f)
    {
        q = (qcp_quadratic_t){-q.q00, -q.q01, -q.q11, -q.p0, -q.p1, -q.r};
    }

    /* 二次约束边界上的目标极值点；落在多边形内即为最优解。 */
    float determinant = q.q00 * q.q11 - q.q01 * q.q01;
    if (determinant < 0.0f)
    {
        return (qcp_point_t){0.0f, 0.0f};
    }
    float q00 = q.q00;
    float q11 = q.q11;
    if (determinant < QCP_EPSILON)
    {
        q00 += QCP_EPSILON;
        q11 += QCP_EPSILON;
        determinant = q00 * q11 - q.q01 * q.q01;
    }
    const float inverse00 = q11 / determinant;
    const float inverse01 = -q.q01 / determinant;
    const float inverse11 = q00 / determinant;
    const float scale_squared = (q.p0 * (inverse00 * q.p0 + inverse01 * q.p1) + q.p1 * (inverse01 * q.p0 + inverse11 * q.p1) - 2.0f * q.r)
        / (objective.x * (inverse00 * objective.x + inverse01 * objective.y)
           + objective.y * (inverse01 * objective.x + inverse11 * objective.y));
    /* 负值表示二次约束无可行点，与参考实现中 NaN 落入交点搜索的结果一致。 */
    if (scale_squared >= 0.0f)
    {
        const float scale = sqrtf(scale_squared);
        const float vx = scale * objective.x - q.p0;
        const float vy = scale * objective.y - q.p1;
        const qcp_point_t best = {inverse00 * vx + inverse01 * vy, inverse01 * vx + inverse11 * vy};
        if (point_inside_polygon(best, &polygon))
        {
            return best;
        }
    }

    return best_point_at_intersections(objective, &polygon, &q);
}

/* 生成逆时针、无重复顶点的线性可行域，要求两个菱形半轴为正。 */
static void build_polygon(qcp_polygon_t *polygon, float rhombus_right, float rhombus_top, float x_max, float y_max)
{
    qcp_polygon_t rhombus = {.count = 0U};

    if (y_max >= rhombus_top)
    {
        rhombus.vertices[rhombus.count++] = (qcp_point_t){rhombus_right, 0.0f};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){0.0f, rhombus_top};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){-rhombus_right, 0.0f};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max > 0.0f)
    {
        const float intersecting_x = (rhombus_right / rhombus_top) * (rhombus_top - y_max);
        rhombus.vertices[rhombus.count++] = (qcp_point_t){rhombus_right, 0.0f};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){intersecting_x, y_max};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){-intersecting_x, y_max};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){-rhombus_right, 0.0f};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max > -rhombus_top)
    {
        const float intersecting_x = -(rhombus_right / rhombus_top) * (-rhombus_top - y_max);
        rhombus.vertices[rhombus.count++] = (qcp_point_t){intersecting_x, y_max};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){-intersecting_x, y_max};
        rhombus.vertices[rhombus.count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max == -rhombus_top)
    {
        rhombus.vertices[rhombus.count++] = (qcp_point_t){0.0f, -rhombus_top};
    }

    /* Sutherland-Hodgman 裁剪：保留 x <= x_max 的部分。 */
    polygon->count = 0U;
    for (uint32_t i = rhombus.count - 1U, j = 0U; j < rhombus.count; i = j++)
    {
        const qcp_point_t current = rhombus.vertices[j];
        const qcp_point_t previous = rhombus.vertices[i];
        const float current_side = current.x - x_max;
        const float previous_side = previous.x - x_max;
        const qcp_point_t crossing = {
            x_max, current.y + (previous.y - current.y) * (x_max - current.x) / (previous.x - current.x)};

        if (current_side == 0.0f)
        {
            polygon->vertices[polygon->count++] = current;
        }
        else if (current_side < 0.0f)
        {
            if (previous_side > 0.0f)
            {
                polygon->vertices[polygon->count++] = crossing;
            }
            polygon->vertices[polygon->count++] = current;
        }
        else if (previous_side < 0.0f && rhombus.count != 2U)
        {
            polygon->vertices[polygon->count++] = crossing;
        }
    }
}

static bool between(float p, float a, float b)
{
    return (p >= a && p <= b) || (p <= a && p >= b);
}

static bool point_inside_polygon(qcp_point_t p, const qcp_polygon_t *polygon)
{
    bool inside = false;

    for (uint32_t i = polygon->count - 1U, j = 0U; j < polygon->count; i = j++)
    {
        const qcp_point_t a = polygon->vertices[i];
        const qcp_point_t b = polygon->vertices[j];

        if ((p.x == a.x && p.y == a.y) || (p.x == b.x && p.y == b.y))
        {
            return true;
        }
        if (a.y == b.y && p.y == a.y && between(p.x, a.x, b.x))
        {
            return true;
        }

        if (between(p.y, a.y, b.y))
        {
            /* 射线恰好穿过顶点时按略低于顶点处理，避免重复计数。 */
            if ((p.y == a.y && b.y >= a.y) || (p.y == b.y && a.y >= b.y))
            {
                continue;
            }

            const float cross = (a.x - p.x) * (b.y - p.y) - (b.x - p.x) * (a.y - p.y);
            if (cross == 0.0f)
            {
                return true;
            }
            if ((a.y < b.y) == (cross > 0.0f))
            {
                inside = !inside;
            }
        }
    }

    return inside;
}

/* 最优点不在多边形内时，在满足二次约束的顶点及各边与二次边界的交点中取目标最大者。 */
static qcp_point_t best_point_at_intersections(qcp_point_t objective, const qcp_polygon_t *polygon,
                                               const qcp_quadratic_t *quadratic)
{
    bool vertex_feasible[QCP_MAX_VERTICES];
    float max_value = -INFINITY;
    qcp_point_t best = {0.0f, 0.0f};

    for (uint32_t index = 0U; index < polygon->count; index++)
    {
        const qcp_point_t v = polygon->vertices[index];
        vertex_feasible[index] = 0.5f * (quadratic->q00 * v.x * v.x + 2.0f * quadratic->q01 * v.x * v.y + quadratic->q11 * v.y * v.y)
            + quadratic->p0 * v.x + quadratic->p1 * v.y + quadratic->r <= 0.0f;
    }

    for (uint32_t i = polygon->count - 1U, j = 0U; j < polygon->count; i = j++)
    {
        const qcp_point_t x1 = polygon->vertices[i];
        const float dx = polygon->vertices[j].x - x1.x;
        const float dy = polygon->vertices[j].y - x1.y;
        /* 候选：下标 0 为顶点，1、2 为边上的两个交点参数。 */
        float parameters[3] = {vertex_feasible[i] ? 0.0f : NAN, NAN, NAN};

        if (!vertex_feasible[i] || !vertex_feasible[j])
        {
            const float qdx = quadratic->q00 * dx + quadratic->q01 * dy;
            const float qdy = quadratic->q01 * dx + quadratic->q11 * dy;
            const float a = 0.5f * (dx * qdx + dy * qdy);
            const float b = x1.x * qdx + x1.y * qdy + quadratic->p0 * dx + quadratic->p1 * dy;
            const float c = 0.5f * (quadratic->q00 * x1.x * x1.x + 2.0f * quadratic->q01 * x1.x * x1.y + quadratic->q11 * x1.y * x1.y)
                + quadratic->p0 * x1.x + quadratic->p1 * x1.y + quadratic->r;
            const float delta = b * b - 4.0f * a * c;
            if (delta >= 0.0f)
            {
                parameters[1] = (-b + sqrtf(delta)) / (2.0f * a);
                parameters[2] = (-b - sqrtf(delta)) / (2.0f * a);
            }
        }

        for (uint32_t index = 0U; index < 3U; index++)
        {
            const float t = parameters[index];
            if (!(index == 0U ? t == 0.0f : (0.0f < t && t < 1.0f)))
            {
                continue;
            }
            const qcp_point_t point = {x1.x + t * dx, x1.y + t * dy};
            const float value = objective.x * point.x + objective.y * point.y;
            if (value > max_value || (value == max_value && point.x >= best.x && point.y >= best.y))
            {
                max_value = value;
                best = point;
            }
        }
    }

    return best;
}
