#include "geo.h"

#include <cmath>

namespace g_geometry {
    g_point g_segment::midpoint() const
    {
        return g_point{(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

    bool g_aligned_segment::containsPoint(const g_point &p) const
    {
        if (is_vertical)
        {
            return p.x == x1 && (p.y >= y1 && p.y <= y2 || p.y >= y2 && p.y <= y1);
        }
        else
        {
            return p.y == y1 && (p.x >= x1 && p.x <= x2 || p.x >= x2 && p.x <= x1);
        }
    }

    bool g_boundary::containsPoint(const g_point &p) const
    {
        return p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max;
    }

    double distance(const double d1, const double d2, const bool is_vertical)
    {
        return std::abs(d1 - d2);
    }

    double distance(const g_point &p1, const g_point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    double distance(const g_point &p, const g_boundary &b)
    {
        bool ge_x_min = (p.x >= b.x_min);
        bool le_x_max = (p.x <= b.x_max);
        bool ge_y_min = (p.y >= b.y_min);
        bool le_y_max = (p.y <= b.y_max);

        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return 0.0;
        }
        if (ge_x_min && le_x_max && !ge_y_min)
        {
            return distance(p.y, b.y_min, true);
        }
        if (ge_x_min && le_x_max && !le_y_max)
        {
            return distance(p.y, b.y_max, true);
        }
        if (ge_y_min && le_y_max && !ge_x_min)
        {
            return distance(p.x, b.x_min, false);
        }
        if (ge_y_min && le_y_max && !le_x_max)
        {
            return distance(p.x, b.x_max, false);
        }
        if (!ge_x_min && !ge_y_min)
        {
            return distance(p, g_point{b.x_min, b.y_min});
        }
        if (!ge_x_min && !le_y_max)
        {
            return distance(p, g_point{b.x_min, b.y_max});
        }
        if (!le_x_max && !ge_y_min)
        {
            return distance(p, g_point{b.x_max, b.y_min});
        }
        if (!le_x_max && !le_y_max)
        {
            return distance(p, g_point{b.x_max, b.y_max});
        }
        return -1.0;
    }
}