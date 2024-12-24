#pragma once

#include <vector>
#include <string>

struct GeoPoint
{
    double x, y;

    double distanceToGeoPoint(const GeoPoint &other) const;
};


struct GeoSegment
{
    GeoPoint p1, p2;

    GeoSegment(const GeoPoint &p1, const GeoPoint &p2) : p1{p1}, p2{p2} {};

    GeoPoint midpoint() const;

    GeoPoint projectionPoint(const GeoPoint &p) const;

    GeoPoint nearestPoint(const GeoPoint &p) const;

    double perpendicularDistanceFromPoint(const GeoPoint &p) const;

    double minDistanceFromPoint(const GeoPoint &p) const;
};



struct GeoAlignedSegment
{
    double x1, y1, x2, y2;
    bool is_vertical;

    GeoAlignedSegment(const GeoPoint &p1, const GeoPoint &p2);

    bool containsPoint(const GeoPoint &p) const;

    double distanceToPoint(const GeoPoint &p) const;

    bool intersectsSegment(const GeoSegment &other) const;
};


struct GeoBoundary
{
    double x_min, y_min, x_max, y_max;

    bool containsPoint(const GeoPoint &p) const;

    bool intersectsBoundary(const GeoBoundary &other) const;

    bool intersectsSegment(const GeoSegment &segment) const;

    double pointMinDistance(const GeoPoint &p) const;
};


struct GeoLine
{
    std::vector<GeoPoint> points;

    GeoLine() {};

    GeoLine(const std::vector<GeoPoint> &points) : points{points} {};

    GeoLine(const std::string &wkt);
};


struct RoadSegment : GeoSegment
{
    int roadufi;
    int pos;

    RoadSegment (const GeoPoint &p1, const GeoPoint &p2, int roadufi, int pos) : GeoSegment(p1, p2), roadufi{roadufi}, pos{pos} {};
};

struct RoadLine : GeoLine
{
    int roadufi;

    RoadLine(int roadufi) : roadufi{roadufi} {};

    RoadLine(const std::vector<GeoPoint> &points, int roadufi) : GeoLine(points), roadufi{roadufi} {};
};