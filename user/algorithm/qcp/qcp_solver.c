#include "qcp_solver.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define QCP_EPSILON      1e-9f

static void build_polygon(qcp_workspace_t *workspace, float rhombus_right, float rhombus_top, float x_max, float y_max);
static bool point_inside_polygon(qcp_point_t p, const qcp_polygon_t *polygon);
static qcp_point_t best_point_at_intersections(qcp_point_t objective, const qcp_polygon_t *polygon,
                                               const qcp_quadratic_t *quadratic);

qcp_point_t qcp_solve(qcp_point_t objective, float x_max, float y_max, float rhombus_right, float rhombus_top,
                      const float *quadratic, qcp_workspace_t *workspace)
{
    build_polygon(workspace, rhombus_right, rhombus_top, x_max, y_max);

    workspace->quadratic = (qcp_quadratic_t){2.0f * quadratic[0], quadratic[1], 2.0f * quadratic[2],
                                             quadratic[3], quadratic[4], quadratic[5]};
    qcp_quadratic_t *constraint = &workspace->quadratic;
    if (constraint->q00 < 0.0f)
    {
        *constraint = (qcp_quadratic_t){-constraint->q00, -constraint->q01, -constraint->q11,
                                       -constraint->p0, -constraint->p1, -constraint->r};
    }

    /* 二次约束边界上的目标极值点；落在多边形内即为最优解。 */
    float determinant = constraint->q00 * constraint->q11 - constraint->q01 * constraint->q01;
    if (determinant < 0.0f)
    {
        return (qcp_point_t){0.0f, 0.0f};
    }
    if (determinant < QCP_EPSILON)
    {
        constraint->q00 += QCP_EPSILON;
        constraint->q11 += QCP_EPSILON;
        determinant = constraint->q00 * constraint->q11 - constraint->q01 * constraint->q01;
    }
    const float inverse00 = constraint->q11 / determinant;
    const float inverse01 = -constraint->q01 / determinant;
    const float inverse11 = constraint->q00 / determinant;
    const float scale_squared = (constraint->p0 * (inverse00 * constraint->p0 + inverse01 * constraint->p1)
                                 + constraint->p1 * (inverse01 * constraint->p0 + inverse11 * constraint->p1)
                                 - 2.0f * constraint->r)
        / (objective.x * (inverse00 * objective.x + inverse01 * objective.y)
           + objective.y * (inverse01 * objective.x + inverse11 * objective.y));
    /* 负值表示二次约束无可行点，与参考实现中 NaN 落入交点搜索的结果一致。 */
    if (scale_squared >= 0.0f)
    {
        const float scale = sqrtf(scale_squared);
        const float vx = scale * objective.x - constraint->p0;
        const float vy = scale * objective.y - constraint->p1;
        const qcp_point_t best = {inverse00 * vx + inverse01 * vy, inverse01 * vx + inverse11 * vy};
        if (point_inside_polygon(best, &workspace->polygon))
        {
            return best;
        }
    }

    return best_point_at_intersections(objective, &workspace->polygon, constraint);
}

/* 生成逆时针、无重复顶点的线性可行域，要求两个菱形半轴为正。 */
static void build_polygon(qcp_workspace_t *workspace, float rhombus_right, float rhombus_top, float x_max, float y_max)
{
    qcp_polygon_t *polygon = &workspace->polygon;
    qcp_point_t *rhombus_vertices = workspace->rhombus_vertices;
    uint32_t rhombus_vertex_count = 0U;

    if (y_max >= rhombus_top)
    {
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){rhombus_right, 0.0f};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){0.0f, rhombus_top};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){-rhombus_right, 0.0f};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max > 0.0f)
    {
        const float intersecting_x = (rhombus_right / rhombus_top) * (rhombus_top - y_max);
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){rhombus_right, 0.0f};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){intersecting_x, y_max};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){-intersecting_x, y_max};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){-rhombus_right, 0.0f};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max > -rhombus_top)
    {
        const float intersecting_x = -(rhombus_right / rhombus_top) * (-rhombus_top - y_max);
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){intersecting_x, y_max};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){-intersecting_x, y_max};
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){0.0f, -rhombus_top};
    }
    else if (y_max == -rhombus_top)
    {
        rhombus_vertices[rhombus_vertex_count++] = (qcp_point_t){0.0f, -rhombus_top};
    }

    /* Sutherland-Hodgman 裁剪：保留 x <= x_max 的部分。 */
    polygon->count = 0U;
    for (uint32_t i = rhombus_vertex_count - 1U, j = 0U; j < rhombus_vertex_count; i = j++)
    {
        const qcp_point_t *current = &rhombus_vertices[j];
        const qcp_point_t *previous = &rhombus_vertices[i];
        const float current_side = current->x - x_max;
        const float previous_side = previous->x - x_max;

        if (current_side == 0.0f)
        {
            polygon->vertices[polygon->count++] = *current;
        }
        else if (current_side < 0.0f)
        {
            if (previous_side > 0.0f)
            {
                polygon->vertices[polygon->count++] = (qcp_point_t){
                    x_max, current->y + (previous->y - current->y) * (x_max - current->x) / (previous->x - current->x)};
            }
            polygon->vertices[polygon->count++] = *current;
        }
        else if (previous_side < 0.0f && rhombus_vertex_count != 2U)
        {
            polygon->vertices[polygon->count++] = (qcp_point_t){
                x_max, current->y + (previous->y - current->y) * (x_max - current->x) / (previous->x - current->x)};
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
        const qcp_point_t *a = &polygon->vertices[i];
        const qcp_point_t *b = &polygon->vertices[j];

        if ((p.x == a->x && p.y == a->y) || (p.x == b->x && p.y == b->y))
        {
            return true;
        }
        if (a->y == b->y && p.y == a->y && between(p.x, a->x, b->x))
        {
            return true;
        }

        if (between(p.y, a->y, b->y))
        {
            /* 射线恰好穿过顶点时按略低于顶点处理，避免重复计数。 */
            if ((p.y == a->y && b->y >= a->y) || (p.y == b->y && a->y >= b->y))
            {
                continue;
            }

            const float cross = (a->x - p.x) * (b->y - p.y) - (b->x - p.x) * (a->y - p.y);
            if (cross == 0.0f)
            {
                return true;
            }
            if ((a->y < b->y) == (cross > 0.0f))
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
    float max_value = -INFINITY;
    qcp_point_t best = {0.0f, 0.0f};

    for (uint32_t i = polygon->count - 1U, j = 0U; j < polygon->count; i = j++)
    {
        const qcp_point_t *x1 = &polygon->vertices[i];
        const qcp_point_t *x2 = &polygon->vertices[j];
        const float dx = x2->x - x1->x;
        const float dy = x2->y - x1->y;
        const bool x1_feasible = 0.5f * (quadratic->q00 * x1->x * x1->x
                                         + 2.0f * quadratic->q01 * x1->x * x1->y
                                         + quadratic->q11 * x1->y * x1->y)
            + quadratic->p0 * x1->x + quadratic->p1 * x1->y + quadratic->r <= 0.0f;
        const bool x2_feasible = 0.5f * (quadratic->q00 * x2->x * x2->x
                                         + 2.0f * quadratic->q01 * x2->x * x2->y
                                         + quadratic->q11 * x2->y * x2->y)
            + quadratic->p0 * x2->x + quadratic->p1 * x2->y + quadratic->r <= 0.0f;

        if (x1_feasible)
        {
            const float value = objective.x * x1->x + objective.y * x1->y;
            if (value > max_value || (value == max_value && x1->x >= best.x && x1->y >= best.y))
            {
                max_value = value;
                best = *x1;
            }
        }

        if (!x1_feasible || !x2_feasible)
        {
            const float qdx = quadratic->q00 * dx + quadratic->q01 * dy;
            const float qdy = quadratic->q01 * dx + quadratic->q11 * dy;
            const float a = 0.5f * (dx * qdx + dy * qdy);
            const float b = x1->x * qdx + x1->y * qdy + quadratic->p0 * dx + quadratic->p1 * dy;
            const float c = 0.5f * (quadratic->q00 * x1->x * x1->x + 2.0f * quadratic->q01 * x1->x * x1->y + quadratic->q11 * x1->y * x1->y)
                + quadratic->p0 * x1->x + quadratic->p1 * x1->y + quadratic->r;
            const float delta = b * b - 4.0f * a * c;
            if (delta >= 0.0f)
            {
                const float root = sqrtf(delta);
                float parameter = (-b + root) / (2.0f * a);
                if (0.0f < parameter && parameter < 1.0f)
                {
                    const qcp_point_t point = {x1->x + parameter * dx, x1->y + parameter * dy};
                    const float value = objective.x * point.x + objective.y * point.y;
                    if (value > max_value || (value == max_value && point.x >= best.x && point.y >= best.y))
                    {
                        max_value = value;
                        best = point;
                    }
                }

                parameter = (-b - root) / (2.0f * a);
                if (0.0f < parameter && parameter < 1.0f)
                {
                    const qcp_point_t point = {x1->x + parameter * dx, x1->y + parameter * dy};
                    const float value = objective.x * point.x + objective.y * point.y;
                    if (value > max_value || (value == max_value && point.x >= best.x && point.y >= best.y))
                    {
                        max_value = value;
                        best = point;
                    }
                }
            }
        }
    }

    return best;
}
