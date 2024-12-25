#pragma once

#include <vector>
#include <string>

struct GPoint
{
    double x, y;

    double distanceToPoint(const GPoint &other) const;

    double distanceToPointGeography(const GPoint &other) const;
};


struct GSegment
{
    GPoint p1, p2;

    GSegment(const GPoint &p1, const GPoint &p2) : p1{p1}, p2{p2} {};

    GPoint midpoint() const;

    GPoint projectionPoint(const GPoint &p) const;

    GPoint nearestPoint(const GPoint &p) const;

    double perpendicularDistanceFromPoint(const GPoint &p) const;

    double minDistanceFromPoint(const GPoint &p) const;
};



struct GAlignedSegment
{
    double x1, y1, x2, y2;
    bool is_vertical;

    GAlignedSegment(const GPoint &p1, const GPoint &p2);

    bool containsPoint(const GPoint &p) const;

    double distanceToPoint(const GPoint &p) const;

    bool intersectsSegment(const GSegment &other) const;
};


struct GBoundary
{
    double x_min, y_min, x_max, y_max;

    bool containsPoint(const GPoint &p) const;

    bool intersectsBoundary(const GBoundary &other) const;

    bool intersectsSegment(const GSegment &segment) const;

    double pointMinDistance(const GPoint &p) const;
};


struct GLine
{
    std::vector<GPoint> points;

    GLine() {};

    GLine(const std::vector<GPoint> &points) : points{points} {};

    GLine(const std::string &wkt);
};


struct RoadSegment : GSegment
{
    int roadufi;
    int pos;

    RoadSegment (const GPoint &p1, const GPoint &p2, int roadufi, int pos) : GSegment(p1, p2), roadufi{roadufi}, pos{pos} {};
};

struct RoadLine : GLine
{
    int roadufi;

    RoadLine(int roadufi) : roadufi{roadufi} {};

    RoadLine(const std::vector<GPoint> &points, int roadufi) : GLine(points), roadufi{roadufi} {};
};