#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <iostream>
// #include <vector>
// #include <cmath>
#include <limits>
#include <stack>
// #include <queue>
// #include <iomanip>
// #include <chrono>

#include <string>
#include <vector>
// #include <sstream>
// #include <fstream>

#include <thread>

// #include "db.h"
#include "point.h"
#include "searchgraph.h"

#include "server.h"

#define TEST_STOPS
#define TEST_SINGLE_POINT

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;



Server::Server(const std::string &address, unsigned short port)
    : endpoint_(net::ip::make_address(address), port), acceptor_(ioc_, endpoint_), searchgraph_()
{
    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();
    searchgraph_.build();
    end = std::chrono::steady_clock::now();
    std::cout << "Total time building search graph = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
}

// Run the server
void Server::run()
{
    std::cout << "Server is running on http://" << endpoint_.address() << ":" << endpoint_.port() << std::endl;

    // Main server loop
    for (;;)
    {
        tcp::socket socket = acceptor_.accept();
        std::thread(&Server::do_session, this, std::move(socket)).detach();
    }
}

std::map<std::string, double> Server::parse_query(const std::string_view &query)
{
    std::map<std::string, double> params;
    std::string query_str = std::string(query);
    std::istringstream iss(query_str);
    std::string token;

    while (std::getline(iss, token, '&'))
    {
        auto pos = token.find('=');
        if (pos != std::string::npos)
        {
            std::string key = token.substr(0, pos);
            std::string value = token.substr(pos + 1);
            try
            {
                params[key] = std::stod(value);
            }
            catch (...)
            {
                return params;
            }
        }
    }
    return params;
}

// Function to parse query parameters from the URL
std::optional<std::pair<double, double>> Server::parse_query_xy(const std::string_view &query)
{
    double x = 0.0, y = 0.0;
    std::string query_str = std::string(query);
    std::istringstream iss(query_str);
    std::string token;

    while (std::getline(iss, token, '&'))
    {
        auto pos = token.find('=');
        if (pos != std::string::npos)
        {
            std::string key = token.substr(0, pos);
            std::string value = token.substr(pos + 1);
            try
            {
                if (key == "x")
                    x = std::stod(value);
                if (key == "y")
                    y = std::stod(value);
            }
            catch (...)
            {
                return std::nullopt; // Handle invalid input
            }
        }
    }
    return {{x, y}};
}

// HTTP request handler
void Server::handle_request(const http::request<http::string_body> &req, http::response<http::string_body> &res)
{
    if (req.method() == http::verb::get && req.target() == "/")
    {
        res.result(http::status::ok);
        res.set(http::field::content_type, "text/plain");
        res.body() = "QuadNode summary stub";
    }
    else if (req.method() == http::verb::get && req.target().starts_with("/nearestsegment"))
    {
        // Extract query string
        std::string_view target = req.target();
        auto pos = target.find('?');
        if (pos == std::string::npos)
        {
            res.result(http::status::bad_request);
            res.body() = "Missing query parameters.";
            res.prepare_payload();
            return;
        }

        std::string_view query = target.substr(pos + 1);
        auto params = parse_query(query);
        double x = params["x"], y = params["y"];

        Point p = {x, y};
        auto [nearestSegment, closestPoint, minDistance, quads] = searchgraph_.quadtree.find_nearest_segment(p);
        res.result(http::status::ok);
        res.set(http::field::content_type, "application/json");
        res.body() = "{\"roadufi\": " + std::to_string(nearestSegment.roadufi) 
        + ", \"pos\": " + std::to_string(nearestSegment.pos)
        + ", \"distance\": " + std::to_string(minDistance) 
        + ", \"x1\": " + std::to_string(nearestSegment.p1.x) 
        + ", \"y1\": " + std::to_string(nearestSegment.p1.y) 
        + ", \"x2\": " + std::to_string(nearestSegment.p2.x) 
        + ", \"y2\": " + std::to_string(nearestSegment.p2.y) 
        + ", \"px\": " + std::to_string(closestPoint.x) 
        + ", \"py\": " + std::to_string(closestPoint.y) 
        + "}";
    }
    else if (req.method() == http::verb::get && req.target().starts_with("/searchpath"))
    {
        // Extract query string
        std::string_view target = req.target();
        auto pos = target.find('?');
        if (pos == std::string::npos)
        {
            res.result(http::status::bad_request);
            res.body() = "Missing query parameters.";
            res.prepare_payload();
            return;
        }

        std::string_view query = target.substr(pos + 1);
        auto params = parse_query(query);

        double x1 = params["x1"], y1 = params["y1"], x2 = params["x2"], y2 = params["y2"];

        auto [path, cost] = searchgraph_.search_path(x1, y1, x2, y2);
        auto path_info = searchgraph_.get_path_info(path);
        res.result(http::status::ok);
        res.set(http::field::content_type, "application/json");
        std::string path_str = "[";
        for (int i = 0; i < path_info.size(); i++)
        {
            if (i > 0) path_str += ", ";
            auto road_info = path_info[i];
            int roadufi = std::get<0>(road_info);
            // RoadUFI, std::string, RoadDirection, double, std::string
            std::string ezi_road_name_label = std::get<1>(road_info);
            RoadDirection direction_code = std::get<2>(road_info);
            double road_length_meters = std::get<3>(road_info);
            std::vector<std::pair<double, double>> geom = std::get<4>(road_info);
            path_str += "{";
            // path_str +=  "\"roadufi\": " + std::to_string(roadufi) + ","; + "\"roadname\": " + ezi_road_name_label + "," + "\"direction\": " + direction_code + "," + "\"direction\": " + road_length_meters + "," + "\"geom\": " + geom + "}";
            path_str +=  "\"ufi\": " + std::to_string(roadufi) + ",";
            path_str += "\"ezi_road_name_label\": \"" + ezi_road_name_label + "\",";
            path_str += "\"direction_code\": \"" + direction_code + "\",";
            path_str += "\"road_length_meters\": " + std::to_string(road_length_meters) + ",";
            path_str += "\"geom\": [";
            for (int j = 0; j < geom.size(); j++)
            {
                if (j > 0) path_str += ", ";
                // Get double with 17 precision
                std::ostringstream oss;
                oss << std::setprecision(17) << geom[j].first; // or std::fixed
                std::string x_str = oss.str();
                std::ostringstream oss2;
                oss2 << std::setprecision(17) << geom[j].second; // or std::fixed
                std::string y_str = oss2.str();
                path_str += "[" + x_str + ", " + y_str + "]";
            }
            path_str += "]";
            path_str += "}";
        }
        path_str += "]";
        res.body() = "{\"path\": " + path_str + ", \"cost\": " + std::to_string(cost) + "}";
    }
    else
    {
        res.result(http::status::not_found);
        res.set(http::field::content_type, "text/plain");
        res.body() = "404 Not Found";
    }
    res.prepare_payload();
}

// Session handler
void Server::do_session(tcp::socket socket)
{
    try
    {
        beast::flat_buffer buffer;

        // Read the HTTP request
        http::request<http::string_body> req;
        http::read(socket, buffer, req);

        // Prepare HTTP response
        http::response<http::string_body> res;

        // Handle the request
        handle_request(req, res);

        // Write the HTTP response back to the client
        http::write(socket, res);
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
