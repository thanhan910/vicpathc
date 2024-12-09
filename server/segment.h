#pragma once

#include "point.h"

struct Segment
{
    Point p1, p2;

    int roadufi;

    int pos;

    Point midpoint() const;

    Point projectionPoint(const Point &p) const;

    Point nearestPoint(const Point &p) const;

    double perpendicularDistanceToPoint(const Point &p) const;

    double minDistanceToPoint(const Point &p) const;

    bool intersects(const double &x1, const double &y1, const double &x2, const double &y2, const bool &is_vertical) const;
};

