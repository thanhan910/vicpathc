#pragma once

#include "point.h"
#include "segment.h"

struct Boundary
{
    double x_min, y_min, x_max, y_max;

    bool contains(const Point &p) const;

    bool intersects_boundary(const Boundary &other) const;

    bool intersects_segment(const Segment &segment) const;

    double distance_min(const Point &p) const;
};