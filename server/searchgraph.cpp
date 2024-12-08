#include "searchgraph.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "db.h"

#include "quadtree.h"
#include "point.h"

namespace bg = boost::geometry;

#define TEST_STOPS
#define TEST_SINGLE_POINT

void SearchGraph::get_points()
{
    // Initialize MongoDB instance and client

    mongocxx::collection points_collection = db["points"];

    // Vector to store documents from MongoDB
    std::vector<bsoncxx::document::value> points_coords_mongo_get;

    // Get the estimated document count
    auto collection_count = points_collection.estimated_document_count();

    // Iterate over the collection and append documents to the vector
    for (auto &&doc : points_collection.find({}))
    {
        points_coords_mongo_get.push_back(bsoncxx::document::value(doc));
    }

    // Convert BSON documents to the desired map structure
    for (const auto &doc : points_coords_mongo_get)
    {
        auto view = doc.view();
        PointUFI id = view["_id"].get_int32().value;
        bsoncxx::array::view coordinates_tuple = view["coords"].get_array().value;
        Coordinate coordinates = {coordinates_tuple[0].get_double(), coordinates_tuple[1].get_double()};

        points_coords[id] = coordinates;
    }

    // // Output the neighbors map for verification
    // for (const auto& [id, neighbor_list] : neighbors) {
    //     std::cout << "ID: " << id << "\n";
    //     for (const auto& [neighbor_id, neighbor_name, neighbor_distance] : neighbor_list) {
    //         std::cout << "  Neighbor ID: " << neighbor_id << ", Name: " << neighbor_name << ", Distance: " << neighbor_distance << "\n";
    //     }
    // }

    // return neighbors;
}

void SearchGraph::get_neighbors()
{
    // Initialize MongoDB instance and client

    mongocxx::collection neighbours_collection = db["points_neighbours"];

    // Vector to store documents from MongoDB
    std::vector<bsoncxx::document::value> neighbors_mongo;

    // // Get the estimated document count
    // auto collection_count = neighbours_collection.estimated_document_count();

    // Iterate over the collection and append documents to the vector
    for (auto &&doc : neighbours_collection.find({}))
    {
        neighbors_mongo.push_back(bsoncxx::document::value(doc));
    }

    // Convert BSON documents to the desired map structure
    for (const auto &doc : neighbors_mongo)
    {
        auto view = doc.view();
        int id = view["_id"].get_int32().value;
        std::vector<Neighbor> neighbor_list;

        for (const auto &neighbor : view["neighbours"].get_array().value)
        {
            auto neighbor_view = neighbor.get_array().value;
            PointUFI neighbor_id = neighbor_view[0].get_int32().value;
            RoadUFI road_id = neighbor_view[1].get_int32().value;
            RoadLength neighbor_distance = neighbor_view[2].get_double().value;
            neighbor_list.emplace_back(neighbor_id, road_id, neighbor_distance);
        }

        neighbors_map[id] = neighbor_list;
    }
}

// Function to calculate geodesic distance between two points (lon1, lat1) and (lon2, lat2)
double SearchGraph::geodesic_distance(double lat1, double lon1, double lat2, double lon2)
{
    BPoint p1(lat1, lon1), p2(lat2, lon2);
    return bg::distance(p1, p2) * 1000.0; // Convert to meters
}

double SearchGraph::heuristic(PointUFI current, PointUFI goal)
{
    auto [lon1, lat1] = points_coords[current];
    auto [lon2, lat2] = points_coords[goal];
    return geodesic_distance(lat1, lon1, lat2, lon2);
}

std::pair<NeighborMap, PointMap> SearchGraph::gen_extra_info(NearestRoadInfo start_road_info, NearestRoadInfo goal_road_info)
{

    auto [start_road_ufi, start_road_direction, start_road_px, start_road_py, start_from_ufi, start_to_ufi] = start_road_info;
    auto [goal_road_ufi, goal_road_direction, goal_road_px, goal_road_py, goal_from_ufi, goal_to_ufi] = goal_road_info;

    NeighborMap special_neighbors;

    points_coords[START_POINT_UFI] = {start_road_px, start_road_py};
    points_coords[GOAL_POINT_UFI] = {goal_road_px, goal_road_py};

    double start_from_ufi_distance = geodesic_distance(start_road_py, start_road_px, points_coords[start_from_ufi].second, points_coords[start_from_ufi].first);
    double start_to_ufi_distance = geodesic_distance(start_road_py, start_road_px, points_coords[start_to_ufi].second, points_coords[start_to_ufi].first);
    double goal_from_ufi_distance = geodesic_distance(goal_road_py, goal_road_px, points_coords[goal_from_ufi].second, points_coords[goal_from_ufi].first);
    double goal_to_ufi_distance = geodesic_distance(goal_road_py, goal_road_px, points_coords[goal_to_ufi].second, points_coords[goal_to_ufi].first);

    special_neighbors[start_from_ufi] = {};
    special_neighbors[start_to_ufi] = {};
    special_neighbors[goal_from_ufi] = {};
    special_neighbors[goal_to_ufi] = {};
    special_neighbors[START_POINT_UFI] = {};
    special_neighbors[GOAL_POINT_UFI] = {};

    if (start_road_direction == DIRECTION_FORWARD || start_road_direction == DIRECTION_BOTH)
    {
        special_neighbors[start_from_ufi].push_back({START_POINT_UFI, start_road_ufi, start_from_ufi_distance});
        special_neighbors[START_POINT_UFI].push_back({start_to_ufi, start_road_ufi, start_to_ufi_distance});
    }
    if (start_road_direction == DIRECTION_REVERSE || start_road_direction == DIRECTION_BOTH)
    {
        special_neighbors[start_to_ufi].push_back({START_POINT_UFI, start_road_ufi, start_to_ufi_distance});
        special_neighbors[START_POINT_UFI].push_back({start_from_ufi, start_road_ufi, start_from_ufi_distance});
    }
    if (goal_road_direction == DIRECTION_FORWARD || goal_road_direction == DIRECTION_BOTH)
    {
        special_neighbors[goal_from_ufi].push_back({GOAL_POINT_UFI, goal_road_ufi, goal_from_ufi_distance});
        special_neighbors[GOAL_POINT_UFI].push_back({goal_to_ufi, goal_road_ufi, goal_to_ufi_distance});
    }
    if (goal_road_direction == DIRECTION_REVERSE || goal_road_direction == DIRECTION_BOTH)
    {
        special_neighbors[goal_to_ufi].push_back({GOAL_POINT_UFI, goal_road_ufi, goal_to_ufi_distance});
        special_neighbors[GOAL_POINT_UFI].push_back({goal_from_ufi, goal_road_ufi, goal_from_ufi_distance});
    }

    PointMap skip_neighbors = {
        {start_from_ufi, start_to_ufi},
        {start_to_ufi, start_from_ufi},
        {goal_from_ufi, goal_to_ufi},
        {goal_to_ufi, goal_from_ufi},
    };

    return {special_neighbors, skip_neighbors};
}

std::pair<Path, RoadLength> SearchGraph::astar(PointUFI start, PointUFI goal, NeighborMap special_neighbors, PointMap skip_neighbors)
{
    // std::vector<FrontierItem> frontier;
    std::priority_queue<FrontierItem, std::vector<FrontierItem>, std::greater<FrontierItem>> frontier;
    std::set<PointUFI> visited;
    Path path;

    // frontier.emplace_back(START_POINT_UFI, 0, start, path);
    frontier.emplace(0, 0, start, path);

    while (!frontier.empty())
    {
        // std::sort(frontier.begin(), frontier.end());
        auto [_, cost, current, current_path] = frontier.top();
        frontier.pop();
        // frontier.erase(frontier.begin());

        if (current == goal)
        {
            return {current_path, cost};
        }

        if (current == 1)
        {
            return {path, cost};
        }

        if (visited.find(current) != visited.end())
        {
            continue;
        }

        visited.insert(current);

        NeighborList neighbor_points = {};
        if (neighbors_map.find(current) != neighbors_map.end())
        {
            for (const auto &p : neighbors_map[current])
            {
                neighbor_points.push_back(p);
            }
        }
        if (special_neighbors.find(current) != neighbors_map.end())
        {
            for (const auto &p : special_neighbors[current])
            {
                neighbor_points.push_back(p);
            }
        }

        for (const auto &[neighbor_point, road_ufi, road_length] : neighbor_points)
        {
            if (skip_neighbors.find(current) != skip_neighbors.end() && skip_neighbors[current] == neighbor_point)
            {
                continue;
            }

            if (visited.find(neighbor_point) == visited.end())
            {
                double heuristic_cost = heuristic(neighbor_point, goal);
                std::vector<int> new_path = current_path;
                new_path.push_back(road_ufi);
                // frontier.emplace(cost + road_length + heuristic_cost, cost + road_length, neighbor_point, new_path);
                frontier.push({cost + road_length + heuristic_cost, cost + road_length, neighbor_point, new_path});
            }
        }
    }

    return {path, 0.0};
}

SearchGraph::SearchGraph() {}

std::vector<NearestRoadInfo> SearchGraph::find_nearest_road(double lon, double lat)
{

    Point p = {lon, lat};

    auto [nearestSegment, closestPoint, minDistance, quads] = quadtree.find_nearest_segment(p);

    RoadUFI roadufi = nearestSegment.roadufi;

    pqxx::work txn(conn);

    std::string query = "SELECT ufi, direction_code, from_ufi, to_ufi FROM vmtrans.tr_road_all WHERE ufi = " + std::to_string(roadufi) + " LIMIT 1;";

    pqxx::result result = txn.exec(query);
    std::vector<NearestRoadInfo> roads;

    for (auto row : result)
    {
        roads.emplace_back(
            row[0].as<RoadUFI>(),
            row[1].as<RoadDirection>(),
            closestPoint.x,
            closestPoint.y,
            row[2].as<PointUFI>(),
            row[3].as<PointUFI>());
    }

    return roads;
}

std::pair<Path, RoadLength> SearchGraph::search_path(double lon1, double lat1, double lon2, double lat2)
{

    NearestRoadInfo start_road_info = find_nearest_road(lon1, lat1).at(0);
    auto [start_road_ufi, start_road_direction, start_road_px, start_road_py, start_from_ufi, start_to_ufi] = start_road_info;

    std::cout << start_road_ufi << " " << start_from_ufi << " " << start_to_ufi << std::endl;

    NearestRoadInfo goal_road_info = find_nearest_road(lon2, lat2).at(0);
    auto [goal_road_ufi, goal_road_direction, goal_road_px, goal_road_py, goal_from_ufi, goal_to_ufi] = goal_road_info;

    std::cout << goal_road_ufi << " " << goal_from_ufi << " " << goal_to_ufi << std::endl;

    if (start_road_ufi == goal_road_ufi)
    {
        return {{start_road_ufi}, 0.0};
    }

    auto [special_neighbors, skip_neighbors] = gen_extra_info(start_road_info, goal_road_info);

    return astar(START_POINT_UFI, GOAL_POINT_UFI, special_neighbors, skip_neighbors); // Assuming start is 0 and goal is 1
}

void SearchGraph::build()
{
    // Populate the points_coords and neighbors collections from MongoDB (similar to Python code)
    // Example usage:
    std::chrono::steady_clock::time_point begin, end;
    begin = std::chrono::steady_clock::now();
    get_neighbors();
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    // Calculate size on RAM
    std::cout << "Size of neighbors_map: " << sizeof(neighbors_map) << " bytes" << std::endl;
    begin = std::chrono::steady_clock::now();
    get_points();
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    // Calculate size on RAM
    std::cout << "Size of points_coords: " << sizeof(points_coords) << " bytes" << std::endl;

    quadtree.gen_quadtree();
}

void test_searchgraph()
{

    SearchGraph searchgraph = SearchGraph();

    searchgraph.build();

    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();
    double lon1 = 144.9631, lat1 = -37.8136; // Melbourne
    double lon2 = 145.0458, lat2 = -37.8768; // Nearby suburb
    auto [path, cost] = searchgraph.search_path(lon1, lat1, lon2, lat2);
    std::cout << "Path: ";
    for (const auto &p : path)
        std::cout << p << " ";
    std::cout << "\nTotal Cost: " << cost << std::endl;
    end = std::chrono::steady_clock::now();
    std::cout << "Find shortest path = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // Sample 10 points and find the nearest roads of each
    std::vector<double> sample_lons = {
        144.9631, 145.0458, 144.9731, 144.9831, 144.9931, 145.0031, 145.0131, 145.0231, 145.0331, 145.0431};
    std::vector<double> sample_lats = {
        -37.8136, -37.8768, -37.8236, -37.8336, -37.8436, -37.8536, -37.8636, -37.8736, -37.8836, -37.8936};
    for (int i = 0; i < 10; i++)
    {
        double lon = sample_lons[i], lat = sample_lats[i];
        auto roads = searchgraph.find_nearest_road(lon, lat);
        std::cout << "Point " << i << " (" << lon << ", " << lat << "): ";
        for (const auto &[road_ufi, direction, px, py, from_ufi, to_ufi] : roads)
        {
            std::cout << road_ufi << " ";
        }
        std::cout << std::endl;
    }
    // g++ main.cpp -o test $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib
};