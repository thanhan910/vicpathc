#pragma once

#include <pqxx/pqxx>

#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/cursor.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>
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

#include "db.h"
#include "point.h"
#include "searchgraph.h"



#define TEST_STOPS
#define TEST_SINGLE_POINT

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;




// Server Class
class Server
{
public:
    Server(const std::string& address, unsigned short port);

    // Run the server
    void run();

private:
    net::io_context ioc_;
    tcp::endpoint endpoint_;
    tcp::acceptor acceptor_;
    SearchGraph searchgraph_;

    std::map<std::string, double> parse_query(const std::string_view& query);

    // Function to parse query parameters from the URL
    std::optional<std::pair<double, double>> parse_query_xy(const std::string_view& query);
    
    // HTTP request handler
    void handle_request(const http::request<http::string_body>& req, http::response<http::string_body>& res);

    // Session handler
    void do_session(tcp::socket socket);
};
