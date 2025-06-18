// wormhole_db.hpp
#pragma once

#include <string>

struct Wormhole {
    double from_x, from_y;
    double to_x, to_y;
};

class WormholeDB {
public:
    WormholeDB(const std::string& db_path);
    Wormhole getWormholePosition(const std::string& from_map, const std::string& to_map);

private:
    std::string db_path_;
};

