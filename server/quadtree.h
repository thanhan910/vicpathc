#pragma once

#include "quadnode.h"

#include <unordered_map>


class QuadTree
{
private:
    QuadNode *root;

public:
    QuadTree();

    std::vector<DSegment> get_segments();
    
    std::tuple<double, double, double, double> get_min_max_coords();

    std::unordered_map<int, std::pair<double, double>> get_stops();

    void gen_quadtree();

    AnswerNearestSegment find_nearest_segment(const bg_point &p);
    
    void test_quadtree();
};
