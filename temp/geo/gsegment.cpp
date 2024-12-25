#include "geoobjs.h"

GPoint GSegment::midpoint() const
{
    return GPoint{(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
}

GPoint GSegment::projectionPoint(const GPoint &p) const {
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

GPoint GSegment::nearestPoint(const GPoint &p) const {
    GPoint projection = projectionPoint(p);
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

double GSegment::perpendicularDistanceFromPoint(const GPoint &p) const {
    GPoint projection = projectionPoint(p);
    return p.distanceToPoint(projection);
}

double GSegment::minDistanceFromPoint(const GPoint &p) const {
    GPoint closest = nearestPoint(p);
    return p.distanceToPoint(closest);
}
