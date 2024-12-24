#include "point.h"

#include <cmath>

double Point::distanceTo(const Point &other) const
{
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
}