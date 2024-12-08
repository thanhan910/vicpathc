#pragma once

// Point structure
struct Point
{
    double x, y;

    double distanceTo(const Point &other) const;
};