#pragma once

#include "quadnode.h"

#include <unordered_map>


class QuadTree
{
private:
    QuadNode *root;

public:
    QuadTree();

    std::vector<Segment> get_segments();
    
    std::tuple<double, double, double, double> get_min_max_coords();

    std::unordered_map<int, std::pair<double, double>> get_stops();

    void gen_quadtree();

    std::tuple<Segment, Point, double, std::vector<QuadNode *>> find_nearest_segment(const Point &p);
    
    void test_quadtree();
};
