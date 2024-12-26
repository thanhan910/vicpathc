#include "geoobjs.h"

#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>


namespace bg = boost::geometry;

using BPoint = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;

double GPoint::distanceToPoint(const GPoint &other) const
{
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
}

double GPoint::distanceToPointGeography(const GPoint &other) const
{
    BPoint p1(x, y);
    BPoint p2(other.x, other.y);
    return bg::distance(p1, p2);
}