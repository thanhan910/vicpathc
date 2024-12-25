#pragma once

// #include "geoobjects/point.h"
// #include "geoobjects/segment.h"
// #include "geoobjects/boundary.h"

#include <vector>
#include <tuple>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>
namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::geographic<bg::degree>> bg_point;
typedef bg::model::segment<bg_point> bg_segment;
typedef bg::model::box<bg_point> bg_box;

struct DSegment
{
    int roadufi;
    int pos;
    bg_segment s;
};

// Forward declaration of QuadNode to avoid circular dependency
class QuadNode;

struct AnswerNearestSegment
{
    DSegment segment;
    bg_point nearest_point;
    double distance;
    std::vector<QuadNode *> quads;
};

// QuadNode node
class QuadNode
{
public:

    friend class QuadTree;
    friend struct AnswerNearestSegment;
    // Bounding box for each quadrant
    QuadNode(bg_box boundary, int capacity = 4);

    // Insert a segment
    bool insert(DSegment segment);

    AnswerNearestSegment find_nearest_segment(const bg_point &p);
    
private:
    bg_box boundary;
    int capacity;
    bool divided;
    int segment_count;
    std::vector<DSegment> segments;

    // Quadrants
    std::vector<QuadNode *> children;

    // Divide the current quadrant into 4
    void subdivide();
    
    // Find the nearest segment to a point
    std::vector<QuadNode *> nearestSegment(const bg_point &point, double &minDistance, DSegment &nearestSegment, bg_point &nearestPoint) const;
};

