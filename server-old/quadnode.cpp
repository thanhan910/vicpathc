#include "point.h"
#include "segment.h"
#include "boundary.h"

#include "quadnode.h"

#include <vector>
#include <limits>
#include <stack>

// QuadNode node

// Bounding box for each quadrant
QuadNode::QuadNode(Boundary boundary, int capacity)
    : boundary(boundary), capacity(capacity), divided(false), segment_count(0) {}

// Insert a segment
bool QuadNode::insert(Segment segment)
{
    if (!(boundary.intersects_segment(segment)))
        return false;

    if (!divided)
    {
        if ((segments.size() < capacity || (boundary.x_max - boundary.x_min) < 0.000001 || (boundary.y_max - boundary.y_min) < 0.000001))
        {
            segments.push_back(segment);
            segment_count++;
            return true;
        }
        else
        {
            subdivide();
        }
    }

    for (QuadNode *child : children)
    {
        if (child->insert(segment))
        {
            segment_count++;
            return true;
        }
    }
    return false;
}

std::tuple<Segment, Point, double, std::vector<QuadNode *>> QuadNode::find_nearest_segment(const Point &p)
{
    double min_distance = std::numeric_limits<double>::max();
    Segment nearest_segment = {{0, 0}, {0, 0}, -1};
    Point nearest_point = {0, 0};
    auto quads = nearestSegment(p, min_distance, nearest_segment, nearest_point);
    return {nearest_segment, nearest_point, min_distance, quads};
}

// Return bounding box
Boundary QuadNode::getBoundary() const
{
    return boundary;
}

// Return the segments
std::vector<Segment> QuadNode::getSegments()
{
    return segments;
}

bool QuadNode::isDivided() const
{
    return divided;
}

std::vector<QuadNode *> QuadNode::getChildren() const
{
    return children;
}

int QuadNode::getSegmentCount() const
{
    return segment_count;
}

// Divide the current quadrant into 4
void QuadNode::subdivide()
{
    double x_mid = (boundary.x_min + boundary.x_max) / 2;
    double y_mid = (boundary.y_min + boundary.y_max) / 2;

    children.push_back(new QuadNode({boundary.x_min, boundary.y_min, x_mid, y_mid}, capacity));
    children.push_back(new QuadNode({boundary.x_min, y_mid, x_mid, boundary.y_max}, capacity));
    children.push_back(new QuadNode({x_mid, boundary.y_min, boundary.x_max, y_mid}, capacity));
    children.push_back(new QuadNode({x_mid, y_mid, boundary.x_max, boundary.y_max}, capacity));

    for (const Segment &seg : segments)
    {
        for (QuadNode *child : children)
        {
            child->insert(seg);
        }
    }

    // Clear the segments vector
    segments.clear();

    divided = true;
}

// Find the nearest segment to a point
std::vector<QuadNode *> QuadNode::nearestSegment(const Point &point, double &minDistance, Segment &nearestSegment, Point &nearestPoint) const
{
    if (divided)
    {
        std::vector<QuadNode *> container_quads;
        std::vector<QuadNode *> non_container_quads;
        for (QuadNode *child : children)
        {
            if (child->boundary.contains(point))
            {
                container_quads.push_back(child);
            }
            else
            {
                non_container_quads.push_back(child);
            }
        }
        for (QuadNode *child : container_quads)
        {
            if (child->segment_count == 0)
            {
                continue;
            }
            else
            {
                auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
                quad.push_back(const_cast<QuadNode *>(this));
                return quad;
            }
        }
        std::vector<QuadNode *> quads;
        for (QuadNode *child : non_container_quads)
        {
            if (child->segment_count == 0)
            {
                continue;
            }
            auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
            quads = quad;
        }
        quads.push_back(const_cast<QuadNode *>(this));
        return quads;
    }

    else
    {
        std::vector<QuadNode *> quads;
        // Segment nearest;
        for (const Segment &segment : segments)
        {
            Point closest = segment.nearestPoint(point);
            double distance = point.distanceTo(closest);
            if (distance < minDistance)
            {
                nearestPoint = closest;
                minDistance = distance;
                nearestSegment = segment;
            }
        }
        quads.push_back(const_cast<QuadNode *>(this));
        return quads;
    }
}
