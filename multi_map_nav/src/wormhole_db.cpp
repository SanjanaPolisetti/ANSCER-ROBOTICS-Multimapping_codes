// wormhole_db.cpp
#include "multi_map_nav/wormhole_db.hpp"
#include <sqlite3.h>
#include <stdexcept>

WormholeDB::WormholeDB(const std::string& db_path) : db_path_(db_path) {}

Wormhole WormholeDB::getWormholePosition(const std::string& from_map, const std::string& to_map) {
    sqlite3* db;
    sqlite3_stmt* stmt;
    Wormhole result;

    if (sqlite3_open(db_path_.c_str(), &db) != SQLITE_OK)
        throw std::runtime_error("Cannot open wormhole database");

    std::string query = "SELECT from_x, from_y, to_x, to_y FROM wormholes WHERE from_map=? AND to_map=? LIMIT 1;";
    if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare query");

    sqlite3_bind_text(stmt, 1, from_map.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, to_map.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result.from_x = sqlite3_column_double(stmt, 0);
        result.from_y = sqlite3_column_double(stmt, 1);
        result.to_x = sqlite3_column_double(stmt, 2);
        result.to_y = sqlite3_column_double(stmt, 3);
    } else {
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        throw std::runtime_error("No wormhole found between maps");
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return result;
}

