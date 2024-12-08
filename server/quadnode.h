#pragma once

#include "point.h"
#include "segment.h"
#include "boundary.h"

#include <vector>
#include <tuple>


// QuadNode node
class QuadNode
{
public:

    friend class QuadTree;
    // Bounding box for each quadrant
    QuadNode(Boundary boundary, int capacity = 4);

    // Insert a segment
    bool insert(Segment segment);

    std::tuple<Segment, Point, double, std::vector<QuadNode *>> find_nearest_segment(const Point &p);
    
    // Return bounding box
    Boundary getBoundary() const;

    std::vector<Segment> getSegments();
    
    bool isDivided() const;
    
    std::vector<QuadNode *> getChildren() const;

    int getSegmentCount() const;
    
private:
    Boundary boundary;
    int capacity;
    bool divided;
#ifdef USE_MIDPOINT_COUNT
    int midpoint_count;
#endif
    int segment_count;
    std::vector<Segment> segments;

    // Quadrants
    std::vector<QuadNode *> children;

    // Divide the current quadrant into 4
    void subdivide();
    
    // Find the nearest segment to a point
    std::vector<QuadNode *> nearestSegment(const Point &point, double &minDistance, Segment &nearestSegment, Point &nearestPoint) const;
};

