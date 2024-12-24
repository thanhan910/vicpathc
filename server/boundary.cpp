#include "boundary.h"
#include "point.h"
#include "segment.h"

#include <cmath>

bool Boundary::contains(const Point &p) const
{
    return (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max);
}

bool Boundary::intersects_boundary(const Boundary &other) const
{
    return !(other.x_min > x_max || other.x_max < x_min ||
             other.y_min > y_max || other.y_max < y_min);
}

bool Boundary::intersects_segment(const Segment &segment) const
{
    if (contains(segment.p1) || contains(segment.p2))
        return true;

    return (
        segment.intersects(x_min, y_max, x_max, y_max, false) || segment.intersects(x_max, y_max, x_max, y_min, true) || segment.intersects(x_max, y_min, x_min, y_min, false) || segment.intersects(x_min, y_min, x_min, y_max, true));
}

double Boundary::distance_min(const Point &p) const
{
    bool ge_x_min = (p.x >= x_min);
    bool le_x_max = (p.x <= x_max);
    bool ge_y_min = (p.y >= y_min);
    bool le_y_max = (p.y <= y_max);

    if (ge_x_min && le_x_max && ge_y_min && le_y_max)
    {
        return 0.0;
    }
    if (ge_x_min && le_x_max && !ge_y_min)
    {
        return y_min - p.y;
    }
    if (ge_x_min && le_x_max && !le_y_max)
    {
        return p.y - y_max;
    }
    if (ge_y_min && le_y_max && !ge_x_min)
    {
        return x_min - p.x;
    }
    if (ge_y_min && le_y_max && !le_x_max)
    {
        return p.x - x_max;
    }

    if (!ge_x_min && !ge_y_min)
    {
        return sqrt((x_min - p.x) * (x_min - p.x) + (y_min - p.y) * (y_min - p.y));
    }
    if (!ge_x_min && !le_y_max)
    {
        return sqrt((x_min - p.x) * (x_min - p.x) + (p.y - y_max) * (p.y - y_max));
    }
    if (!le_x_max && !ge_y_min)
    {
        return sqrt((p.x - x_max) * (p.x - x_max) + (y_min - p.y) * (y_min - p.y));
    }
    if (!le_x_max && !le_y_max)
    {
        return sqrt((p.x - x_max) * (p.x - x_max) + (p.y - y_max) * (p.y - y_max));
    }
    return -1.0;
}