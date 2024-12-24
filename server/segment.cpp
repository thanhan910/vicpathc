#include "segment.h"

#include "point.h"
#include <cmath>

Point Segment::midpoint() const
{
    return {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
}

Point Segment::projectionPoint(const Point &p) const
{
    // Project p onto the line segment, clamp to endpoints
    double A = p.x - p1.x;
    double B = p.y - p1.y;
    double C = p2.x - p1.x;
    double D = p2.y - p1.y;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = (len_sq != 0) ? dot / len_sq : -1;

    double xx, yy;
    if (param < 0)
    {
        xx = p1.x;
        yy = p1.y;
    }
    else if (param > 1)
    {
        xx = p2.x;
        yy = p2.y;
    }
    else
    {
        xx = p1.x + param * C;
        yy = p1.y + param * D;
    }
    return {xx, yy};
}

Point Segment::nearestPoint(const Point &p) const
{
    Point projection = projectionPoint(p);
    double x_min = std::min(p1.x, p2.x), x_max = std::max(p1.x, p2.x), y_min = std::min(p1.y, p2.y), y_max = std::max(p1.y, p2.y);
    bool ge_x_min = projection.x >= x_min;
    bool le_x_max = projection.x <= x_min;
    bool ge_y_min = projection.y >= y_min;
    bool le_y_max = projection.y <= y_min;
    // Check if the closest point is on the line segment
    if (ge_x_min && le_x_max && ge_y_min && le_y_max)
    {
        return projection;
    }
    else
    {
        if (p1.x == p2.x)
        {
            if (p1.y < p2.y)
            {
                if (projection.y < p1.y)
                {
                    return p1;
                }
                else
                {
                    return p2;
                }
            }
            else
            {
                if (projection.y < p2.y)
                {
                    return p1;
                }
                else
                {
                    return p2;
                }
            }
        }
        else if (p1.x < p2.x)
        {
            if (projection.x < p1.x)
            {
                return p1;
            }
            else
            {
                return p2;
            }
        }
        else
        {
            if (projection.x < p2.x)
            {
                return p1;
            }
            else
            {
                return p2;
            }
        }
    }
}

// Compute the distance from a point to the line segment
double Segment::perpendicularDistanceToPoint(const Point &p) const
{
    Point projection = projectionPoint(p);
    return p.distanceTo(projection);
}

double Segment::minDistanceToPoint(const Point &p) const
{
    Point closest = nearestPoint(p);
    return p.distanceTo(closest);
}

bool Segment::intersects(const double &x1, const double &y1, const double &x2, const double &y2, const bool &is_vertical) const
{

    // lambda function to check if a horizontal or vertical line contains a point
    auto contains = [](const Point &p, const double &x1, const double &y1, const double &x2, const double &y2, const bool &is_vertical)
    {
        if (is_vertical)
        {
            return (p.x == x1 && p.y >= y1 && p.y <= y2);
        }
        else
        {
            return (p.y == y1 && p.x >= x1 && p.x <= x2);
        }
    };

    if (is_vertical)
    {
        bool crossLine = (p1.x < x1 && p2.x > x1) || (p1.x > x1 && p2.x < x1);
        if (!crossLine)
        {
            if (p1.x == x1)
                return contains(p1, x1, y1, x2, y2, is_vertical);
            if (p2.x == x1)
                return contains(p2, x1, y1, x2, y2, is_vertical);
            else
                return false;
        }
    }
    else
    {
        bool crossLine = (p1.y < y1 && p2.y > y1) || (p1.y > y1 && p2.y < y1);
        if (!crossLine)
        {
            if (p1.y == y1)
                return contains(p1, x1, y1, x2, y2, is_vertical);
            else if (p2.y == y1)
                return contains(p2, x1, y1, x2, y2, is_vertical);
            else
                return false;
        }
    }

    double delta_y = p2.y - p1.y;
    double delta_x = p2.x - p1.x;
    if (is_vertical)
    {
        double line_intersect_y = p1.y + delta_y * (x1 - p1.x) / delta_x;
        return (line_intersect_y >= y1 && line_intersect_y <= y2);
    }
    else
    {
        double line_intersect_x = p1.x + delta_x * (y1 - p1.y) / delta_y;
        return (line_intersect_x >= x1 && line_intersect_x <= x2);
    }
}
