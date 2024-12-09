#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "db.h"

#include "quadtree.h"
#include "point.h"
#include "segment.h"


namespace bg = boost::geometry;


#define TEST_STOPS
#define TEST_SINGLE_POINT

using BPoint = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;
using PointUFI = int;
using Coordinate = std::pair<double, double>;
using RoadUFI = int;
using RoadDirection = std::string;
using RoadLength = double;
using Neighbor = std::tuple<PointUFI, RoadUFI, RoadLength>;
using NeighborList = std::vector<Neighbor>;
using NeighborMap = std::map<PointUFI, NeighborList>;

using PointMap = std::map<PointUFI, PointUFI>;

using NearestRoadInfo = std::tuple<RoadUFI, RoadDirection, PointUFI, PointUFI, double, double, Segment>;

using Path = std::vector<RoadUFI>;
using FrontierItem = std::tuple<RoadLength, RoadLength, PointUFI, Path>;

using PathRoadInfo = std::tuple<RoadUFI, std::string, RoadDirection, double, std::vector<std::pair<double, double>>>;

class SearchGraph {

private:

    #define START_POINT_UFI 0
    #define GOAL_POINT_UFI 1
    #define DIRECTION_FORWARD "F"
    #define DIRECTION_REVERSE "R"
    #define DIRECTION_BOTH "B"

    std::map<PointUFI, Coordinate> points_coords;
    NeighborMap neighbors_map;

    
    void get_points();

    void get_neighbors();
    
    // Function to calculate geodesic distance between two points (lon1, lat1) and (lon2, lat2)
    double geodesic_distance(double lat1, double lon1, double lat2, double lon2);

    double heuristic(PointUFI current, PointUFI goal);

    std::pair<NeighborMap, PointMap> gen_extra_info(NearestRoadInfo start_road_info, NearestRoadInfo goal_road_info);

    std::pair<Path, RoadLength> astar(PointUFI start, PointUFI goal, NeighborMap special_neighbors, PointMap skip_neighbors);

public:

    QuadTree quadtree;

    SearchGraph();

    NearestRoadInfo find_nearest_road(double lon, double lat);

    std::pair<Path, RoadLength> search_path(double lon1, double lat1, double lon2, double lat2);

    std::vector<PathRoadInfo> get_path_info(const Path &path);

    void build();
};

void test_searchgraph();