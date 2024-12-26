#pragma once

#include <vector>
#include <string>

namespace g_geometry
{

    struct g_point
    {
        double x, y;
    };

    struct g_segment
    {
        g_point p1, p2;
        g_segment(const g_point &p1, const g_point &p2) : p1{p1}, p2{p2} {};
        g_point midpoint() const;
    };

    struct g_aligned_segment
    {
        double x1, y1, x2, y2;
        bool is_vertical;
        g_aligned_segment(const g_point &p1, const g_point &p2) : x1{p1.x}, y1{p1.y}, x2{p2.x}, y2{p2.y}, is_vertical{p1.x == p2.x} {};
        g_aligned_segment(const double x1, const double y1, const double x2, const double y2, const bool is_vertical) : x1{x1}, y1{y1}, x2{x2}, y2{y2}, is_vertical{is_vertical} {};
        bool containsPoint(const g_point &p) const;
    };

    struct g_boundary
    {
        double x_min, y_min, x_max, y_max;
        bool containsPoint(const g_point &p) const;
    };

    struct g_line
    {
        std::vector<g_point> points;
        g_line() {};
        g_line(const std::vector<g_point> &points) : points{points} {};
        g_line(const std::string &wkt);
    };

    struct d_segment : g_segment
    {
        int roadufi;
        int pos;
        d_segment(const g_point &p1, const g_point &p2, int roadufi, int pos) : g_segment(p1, p2), roadufi{roadufi}, pos{pos} {};
    };

    struct d_line : g_line
    {
        int roadufi;
        d_line(int roadufi) : roadufi{roadufi} {};
        d_line(const std::vector<g_point> &points, int roadufi) : g_line(points), roadufi{roadufi} {};
    };

    bool intersects(const g_segment &s1, const g_segment &s2);
    bool intersects(const g_boundary &b1, const g_boundary &b2);
    bool intersects(const g_segment &s, const g_aligned_segment &as);
    bool intersects(const g_segment &s, const g_boundary &b);
    double distance(const double d1, const double d2, const bool is_vertical);
    double distance(const g_point &p1, const g_point &p2);
    double distance(const g_point &p, const g_boundary &b);
    
    g_point projectionPoint(const g_point &p, const g_segment &s);
    g_point closestPoint(const g_point &p, const g_segment &s);
    double projectionDistance(const g_point &p, const g_segment &s);
    double closestDistance(const g_point &p, const g_segment &s);
};