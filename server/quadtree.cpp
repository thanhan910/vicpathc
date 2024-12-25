#include "quadtree.h"
#include "quadnode.h"

#include "db.h"

#include <cmath>
#include <queue>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>

#define TEST_STOPS
#define TEST_SINGLE_POINT

using FrontierElement = std::pair<QuadNode *, double>;

struct CompareFrontierElement
{
    bool operator()(const FrontierElement &q1, const FrontierElement &q2)
    {
        return q1.second > q2.second;
    }
};

QuadTree::QuadTree() : root(nullptr) {}

std::vector<DSegment> QuadTree::get_segments()
{
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.segments";

    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();
    pqxx::result result = txn.exec(query);
    end = std::chrono::steady_clock::now();
    std::cout << "Get from PostgreSQL = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    std::vector<DSegment> segments;

    for (auto row : result)
    {
        int roadufi = row[0].as<int>();
        int pos = row[1].as<int>();
        double x1 = row[2].as<double>();
        double y1 = row[3].as<double>();
        double x2 = row[4].as<double>();
        double y2 = row[5].as<double>();
        // Point p1 = {x1, y1};
        // Point p2 = {x2, y2};
        DSegment segment = {roadufi, pos, {{x1, y1}, {x2, y2}}};
        segments.push_back(segment);
    }

    end = std::chrono::steady_clock::now();
    std::cout << "Gen segments = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return segments;
}

std::tuple<double, double, double, double> QuadTree::get_min_max_coords()
{
    pqxx::work txn(conn);

    std::string query = "SELECT * FROM vmtrans.boundary";

    pqxx::result result = txn.exec(query);

    double x_min = result[0][0].as<double>();
    double y_min = result[0][1].as<double>();
    double x_max = result[0][2].as<double>();
    double y_max = result[0][3].as<double>();

    x_min = std::floor(x_min) - 1.0;
    y_min = std::floor(y_min) - 1.0;
    x_max = std::ceil(x_max) + 1.0;
    y_max = std::ceil(y_max) + 1.0;

    std::cout << "x_min: " << x_min << " y_min: " << y_min << " x_max: " << x_max << " y_max: " << y_max << std::endl;

    return std::make_tuple(x_min, y_min, x_max, y_max);
}

std::unordered_map<int, std::pair<double, double>> QuadTree::get_stops()
{
    pqxx::work txn(conn);

    std::string query = "SELECT stop_id, stop_lat, stop_lon FROM vmtrans.stops";

    pqxx::result result = txn.exec(query);

    std::unordered_map<int, std::pair<double, double>> stops;

    for (auto row : result)
    {
        int id = row[0].as<int>();
        double lat = row[1].as<double>();
        double lon = row[2].as<double>();
        // stops.emplace_back(id, std::make_pair(lat, lon));
        stops[id] = std::make_pair(lon, lat);
    }

    return stops;
}

void QuadTree::gen_quadtree()
{
    // std::vector<Segment> segments = get_segments();

    std::chrono::steady_clock::time_point begin, end;


    begin = std::chrono::steady_clock::now();
    auto [x_min, y_min, x_max, y_max] = get_min_max_coords();
    end = std::chrono::steady_clock::now();
    std::cout << "Get min max coords: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();
    bg_box boundary = {{x_min, y_min}, {x_max, y_max}};
    root = new QuadNode(boundary);
    end = std::chrono::steady_clock::now();
    std::cout << "Create quadtree from boundary: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    // Read segments from a csv file
    begin = std::chrono::steady_clock::now();

    // std::ifstream file("../local/segments.csv");
    std::ifstream file("/data/segments.csv");

    std::string line;

    // Skip the first line
    std::getline(file, line);
    int roadufi, pos;
    double x1, y1, x2, y2;
    char delimiter;
    int i = 0;
    while (file >> roadufi >> delimiter >> pos >> delimiter >> x1 >> delimiter >> y1 >> delimiter >> x2 >> delimiter >> y2)
    {
        i++;
        if (i % 108000 == 0)
        {
            std::cout << "Batch: " << i / 108000 << std::endl;
        }
        DSegment segment = {roadufi, pos, {{x1, y1}, {x2, y2}}};
        root->insert(segment);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Insert segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}

double distance_from_point_to_box(const bg_point &p, const bg_box &b)
{
    double p_x = p.get<0>();
    double p_y = p.get<1>();
    double x_min = b.min_corner().get<0>();
    double y_min = b.min_corner().get<1>();
    double x_max = b.max_corner().get<0>();
    double y_max = b.max_corner().get<1>();

    bool ge_x_min = (p_x >= x_min);
    bool le_x_max = (p_x <= x_max);
    bool ge_y_min = (p_y >= y_min);
    bool le_y_max = (p_y <= y_max);

    if (ge_x_min && le_x_max && ge_y_min && le_y_max)
    {
        return 0.0;
    }
    if (ge_x_min && le_x_max && !ge_y_min)
    {
        return y_min - p_y;
    }
    if (ge_x_min && le_x_max && !le_y_max)
    {
        return p_y - y_max;
    }
    if (ge_y_min && le_y_max && !ge_x_min)
    {
        return x_min - p_x;
    }
    if (ge_y_min && le_y_max && !le_x_max)
    {
        return p_x - x_max;
    }

    if (!ge_x_min && !ge_y_min)
    {
        return sqrt((x_min - p_x) * (x_min - p_x) + (y_min - p_y) * (y_min - p_y));
    }
    if (!ge_x_min && !le_y_max)
    {
        return sqrt((x_min - p_x) * (x_min - p_x) + (p_y - y_max) * (p_y - y_max));
    }
    if (!le_x_max && !ge_y_min)
    {
        return sqrt((p_x - x_max) * (p_x - x_max) + (y_min - p_y) * (y_min - p_y));
    }
    if (!le_x_max && !le_y_max)
    {
        return sqrt((p_x - x_max) * (p_x - x_max) + (p_y - y_max) * (p_y - y_max));
    }
    return -1.0;
}

AnswerNearestSegment QuadTree::find_nearest_segment(const bg_point &p)
{
    double min_distance = std::numeric_limits<double>::max();
    // Segment nearest_segment = {{0, 0}, {0, 0}, -1, -1};
    DSegment nearest_segment = {-1, -1, {{0, 0}, {0, 0}}};
    bg_point min_point = {0, 0};

    std::priority_queue<FrontierElement, std::vector<FrontierElement>, CompareFrontierElement> frontier;

    frontier.push({root, 0.0});

    std::vector<QuadNode *> quads;
    while (!frontier.empty())
    {
        FrontierElement q = frontier.top();
        frontier.pop();
        QuadNode *quad = q.first;
        double distance = q.second;
        quads.push_back(quad);

        if (distance > min_distance)
        {
            break;
        }
        if (quad->segment_count == 0)
        {
            continue;
        }
        if (quad->divided)
        {
            for (QuadNode *child : quad->children)
            {
                if (child->segment_count <= 0)
                {
                    continue;
                }
                double child_boundary_distance = distance_from_point_to_box(p, child->boundary);
                frontier.push({child, child_boundary_distance});
            }
        }
        else
        {
            for (const DSegment &segment : quad->segments)
            {
                // Point closest = segment.nearestPoint(p);
                bg_segment closest_segment;
                bg::closest_points(segment.s, p, closest_segment);
                double distance = bg::distance(p, segment.s);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_segment = segment;
                    min_point = closest_segment.first;
                }
            }
        }
    }

    return {nearest_segment, min_point, min_distance, quads};
}

void QuadTree::test_quadtree()
{
#ifdef TEST_STOPS

    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> stops = get_stops();
    end = std::chrono::steady_clock::now();
    std::cout << "Get stops: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    // std::ofstream file("../local/stops_nearest_segment.csv");
    std::ofstream file("/data/stops_nearest_segment.csv");

    file << "stop_id,stop_lat,stop_lon,roadufi,segment_x1,segment_y1,segment_x2,segment_y2,distance_degree,distance_meter" << std::endl;

    int i = 0;
    for (const auto &[id, stop] : stops)
    {
        i++;
        bg_point p = {stop.first, stop.second};
        // std::cout << "Stop: " << i << " " << id << " (" << stop.first << ", " << stop.second << ")" << std::endl;
        // auto [nearestSegment, minDistance, quads] = quadtree.find_nearest_segment(p);
        auto [nearestSegment, closestPoint, minDistance, quads] = find_nearest_segment(p);
        double distanceMeter = minDistance * 111139;
        file << id << "," << 
        std::setprecision(17) << stop.first << "," << 
        std::setprecision(17) << stop.second << "," << 
        std::setprecision(17) << stop.second << "," << nearestSegment.roadufi << "," << 
        std::setprecision(17) << nearestSegment.s.first.get<0>() << "," <<
        std::setprecision(17) << nearestSegment.s.first.get<1>() << "," <<
        std::setprecision(17) << nearestSegment.s.second.get<0>() << "," <<
        std::setprecision(17) << nearestSegment.s.second.get<1>() << "," <<
        std::setprecision(17) << minDistance << "," << 
        std::setprecision(17) << distanceMeter << std::endl;
        if (nearestSegment.roadufi < 0)
        {
            std::cout << "Nearest segment to (" << stop.first << ", " << stop.second << ") " << "is from (" << 
            std::setprecision(17) << nearestSegment.s.first.get<0>() << ", " << 
            std::setprecision(17) << nearestSegment.s.first.get<1>() << ") " << "to (" << 
            std::setprecision(17) << nearestSegment.s.second.get<0>() << ", " << 
            std::setprecision(17) << nearestSegment.s.second.get<1>() << ") " << "with distance: " << 
            std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;
        }
    }

    file.close();
    end = std::chrono::steady_clock::now();
    // std::cout << "Total stops: " << stops.size() << std::endl;
    std::cout << "Find stops' nearest segments: time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif

#ifdef TEST_SINGLE_POINT

    bg_point p = {144.866, -37.7512};
    // Point p = {145.183, -37.9948};
    // Point p = {145.183, -37.9948};

    begin = std::chrono::steady_clock::now();

    auto [nearestSegment, closestPoint, minDistance, quads] = root->find_nearest_segment(p);
    end = std::chrono::steady_clock::now();
    std::cout << "Nearest segment to (" << 
    std::setprecision(17) << p.get<0>() << ", " << 
    std::setprecision(17) << p.get<1>() << ") " << "is from (" << 
    std::setprecision(17) << nearestSegment.s.first.get<0>() << ", " << 
    std::setprecision(17) << nearestSegment.s.first.get<1>() << ") " << "to (" << 
    std::setprecision(17) << nearestSegment.s.second.get<0>() << ", " << 
    std::setprecision(17) << nearestSegment.s.second.get<1>() << ") " << "with distance: " <<
    std::setprecision(17) << minDistance << " " << "roadufi: " << nearestSegment.roadufi << std::endl;

    // Traverse in reverse
    for (auto it = quads.rbegin(); it != quads.rend(); ++it)
    {
        QuadNode *quad = *it;
        std::cout << "Quad: " 
        << std::setprecision(17) 
        << quad->boundary.min_corner().get<0>() 
        << " " 
        << std::setprecision(17) 
        << quad->boundary.min_corner().get<1>()
        << " " 
        << std::setprecision(17) 
        << quad->boundary.max_corner().get<0>()
        << " " 
        << std::setprecision(17) 
        << quad->boundary.max_corner().get<1>()
        << " " 
        << quad->segments.size()
        << " " 
        << quad->divided
        << " " << quad->segment_count << std::endl;

        if (quad->segments.size() > 0)
        {
            for (auto seg : quad->segments)
            {
                std::cout << "Segment: " 
                << seg.s.first.get<0>()
                << " " 
                << seg.s.first.get<1>()
                << " " 
                << seg.s.second.get<0>()
                << " " 
                << seg.s.second.get<1>()
                << std::endl;
            }
        }
    }

    std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();

    auto nearestAns = find_nearest_segment(p);
    // auto nearestSegment = nearestAns.segment;
    // auto minDistance = nearestAns.distance;
    // auto quads = nearestAns.quads;
    end = std::chrono::steady_clock::now();
    std::cout << "Nearest segment to (" << 
    std::setprecision(17) << p.get<0>() << ", " << 
    std::setprecision(17) << p.get<1>() << ") " << "is from (" << 
    std::setprecision(17) << nearestAns.segment.s.first.get<0>() << ", " << 
    std::setprecision(17) << nearestAns.segment.s.first.get<1>() << ") " << "to (" << 
    std::setprecision(17) << nearestAns.segment.s.second.get<0>() << ", " << 
    std::setprecision(17) << nearestAns.segment.s.second.get<1>() << ") " << "with distance: " <<
    std::setprecision(17) << nearestAns.distance << " " << "roadufi: " << nearestAns.segment.roadufi << std::endl;

    // Traverse in reverse
    for (auto it = nearestAns.quads.begin(); it != quads.end(); ++it)
    {
        QuadNode *quad = *it;
        std::cout << "Quad: " 
        << std::setprecision(17) 
        << quad->boundary.min_corner().get<0>() 
        << " " 
        << std::setprecision(17) 
        << quad->boundary.min_corner().get<1>()
        << " " 
        << std::setprecision(17) 
        << quad->boundary.max_corner().get<0>()
        << " " 
        << std::setprecision(17) 
        << quad->boundary.max_corner().get<1>()
        << " " 
        << quad->segments.size()
        << " " 
        << quad->divided
        << " " << quad->segment_count << std::endl;

        if (quad->segments.size() > 0)
        {
            for (auto seg : quad->segments)
            {
                std::cout << "Segment: " 
                << seg.s.first.get<0>()
                << " " 
                << seg.s.first.get<1>()
                << " " 
                << seg.s.second.get<0>()
                << " " 
                << seg.s.second.get<1>()
                << std::endl;
            }
        }
    }

    std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

#endif
}
