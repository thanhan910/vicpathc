#include "db.h"

mongocxx::instance instance{};
mongocxx::client client{mongocxx::uri{"mongodb://host.docker.internal:27017"}};
mongocxx::database db = client["vic_db"];
pqxx::connection conn("dbname=vic_db user=postgres password=postgres host=host.docker.internal port=5432");