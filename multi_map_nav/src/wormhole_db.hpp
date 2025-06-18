#ifndef WORMHOLE_DB_HPP
#define WORMHOLE_DB_HPP

#include <string>
#include <utility>
#include <sqlite3.h>

class WormholeDB {
public:
    WormholeDB(const std::string& db_path);
    ~WormholeDB();

    std::pair<double, double> getWormholePosition(const std::string& from_map, const std::string& to_map);

private:
    sqlite3* db_;
};

#endif

