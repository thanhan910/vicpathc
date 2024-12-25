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

extern mongocxx::instance instance;
extern mongocxx::client client;
extern mongocxx::database db;
extern pqxx::connection conn;
