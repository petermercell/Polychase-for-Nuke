// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>

#include "database.h"

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <source_location>
#include <utility>

#include "utils.h"

inline void Sqlite3ExecChecked(
    sqlite3* database, const char* sql,
    int (*callback)(void*, int, char**, char**),
    const std::source_location& loc = std::source_location::current()) {
    char* err_msg = nullptr;

    const int result_code =
        sqlite3_exec(database, sql, callback, nullptr, &err_msg);
    if (result_code != SQLITE_OK) {
        const std::string exception_message =
            fmt::format("SQLite error [{}:{}]: {}", loc.file_name(), loc.line(),
                        err_msg != nullptr ? err_msg : "Unknown error");

        sqlite3_free(err_msg);
        throw std::runtime_error(exception_message);
    }
}

inline int Sqlite3Check(
    int result_code, bool throw_exception = true,
    const std::source_location& loc = std::source_location::current()) {
    switch (result_code) {
        case SQLITE_OK:
        case SQLITE_ROW:
        case SQLITE_DONE:
            return result_code;
    }
    std::string error_message =
        fmt::format("SQLite error [{}:{}]: {}", loc.file_name(), loc.line(),
                    sqlite3_errstr(result_code));

    if (throw_exception) {
        throw std::runtime_error(error_message);
    } else {
        SPDLOG_ERROR("{}", error_message);
        return result_code;
    }
}

inline int Sqlite3CheckNoThrow(int result_code,
                               const std::source_location& loc =
                                   std::source_location::current()) noexcept {
    return Sqlite3Check(result_code, false, loc);
}

Database::Database(const std::string& path) { Open(path); }

Database::Database(Database&& other) {
    database_ = std::exchange(other.database_, nullptr);
    sql_stmt_read_keypoints_ =
        std::exchange(other.sql_stmt_read_keypoints_, nullptr);
    sql_stmt_write_keypoints_ =
        std::exchange(other.sql_stmt_write_keypoints_, nullptr);
    sql_stmt_read_image_pair_flows_ =
        std::exchange(other.sql_stmt_read_image_pair_flows_, nullptr);
    sql_stmt_write_image_pair_flows_ =
        std::exchange(other.sql_stmt_write_image_pair_flows_, nullptr);
    sql_stmt_find_flows_from_image_ =
        std::exchange(other.sql_stmt_find_flows_from_image_, nullptr);
    sql_stmt_find_flows_to_image_ =
        std::exchange(other.sql_stmt_find_flows_to_image_, nullptr);
    sql_stmt_keypoints_exist_ =
        std::exchange(other.sql_stmt_keypoints_exist_, nullptr);
    sql_stmt_pair_flow_exist_ =
        std::exchange(other.sql_stmt_pair_flow_exist_, nullptr);
    sql_stmt_min_image_id = std::exchange(other.sql_stmt_min_image_id, nullptr);
    sql_stmt_max_image_id = std::exchange(other.sql_stmt_max_image_id, nullptr);
}

void Database::Open(const std::string& path) {
    Close();

    // SQLITE_OPEN_NOMUTEX specifies that the connection should not have a
    // mutex (so that we don't serialize the connection's operations).
    // Modifications to the database will still be serialized, but multiple
    // connections can read concurrently.
    Sqlite3Check(sqlite3_open_v2(
        path.c_str(), &database_,
        SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
        nullptr));

    // Don't wait for the operating system to write the changes to disk
    Sqlite3ExecChecked(database_, "PRAGMA synchronous=OFF", nullptr);

    // Use faster journaling mode
    Sqlite3ExecChecked(database_, "PRAGMA journal_mode=WAL", nullptr);

    // Store temporary tables and indices in memory
    Sqlite3ExecChecked(database_, "PRAGMA temp_store=MEMORY", nullptr);

    // Disabled by default
    Sqlite3ExecChecked(database_, "PRAGMA foreign_keys=ON", nullptr);

    // Enable auto vacuum to reduce DB file size
    Sqlite3ExecChecked(database_, "PRAGMA auto_vacuum=1", nullptr);

    CreateTables();
    PrepareSQLStatements();
}

void Database::Close() noexcept {
    if (database_ != nullptr) {
        FinalizeSQLStatements();
        Sqlite3CheckNoThrow(sqlite3_close_v2(database_));
        database_ = nullptr;
    }
}

void Database::CreateTables() const {
    CreateKeypointsTable();
    CreateOpticalFlowTable();
}

void Database::CreateKeypointsTable() const {
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS keypoints(
            image_id   INTEGER  PRIMARY KEY  NOT NULL,
            rows       INTEGER               NOT NULL,
            keypoints  BLOB                  NOT NULL
        );
    )";

    Sqlite3ExecChecked(database_, sql, nullptr);
}

void Database::CreateOpticalFlowTable() const {
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS optical_flow(
            image_id_from           INTEGER  NOT NULL,
            image_id_to             INTEGER  NOT NULL,
            rows                    INTEGER  NOT NULL,
            src_keypoints_indices   BLOB     NOT NULL,
            tgt_keypoints           BLOB     NOT NULL,
            flow_errors             BLOB     NOT NULL,
            PRIMARY KEY(image_id_from, image_id_to),
            FOREIGN KEY(image_id_from) REFERENCES keypoints(image_id) ON DELETE CASCADE
        );
    )";

    Sqlite3ExecChecked(database_, sql, nullptr);
}

template <typename T>
void ReadMatrixBlob(sqlite3_stmt* sql_stmt, int32_t num_rows, int32_t col,
                    std::vector<T>& vec) {
    vec.clear();
    vec.resize(num_rows);

    const size_t num_bytes =
        static_cast<size_t>(sqlite3_column_bytes(sql_stmt, col));
    CHECK_EQ(vec.size() * sizeof(T), num_bytes);

    memcpy(reinterpret_cast<char*>(vec.data()),
           sqlite3_column_blob(sql_stmt, col), num_bytes);
}

template <typename T>
void WriteMatrixBlob(sqlite3_stmt* sql_stmt, const std::vector<T>& matrix,
                     int32_t col) {
    const size_t num_bytes = matrix.size() * sizeof(T);
    Sqlite3Check(sqlite3_bind_blob(sql_stmt, col,
                                   reinterpret_cast<const char*>(matrix.data()),
                                   static_cast<int>(num_bytes), SQLITE_STATIC));
}

Keypoints Database::ReadKeypoints(int32_t image_id) const {
    Keypoints keypoints;
    ReadKeypoints(image_id, keypoints);
    return keypoints;
}

void Database::ReadKeypoints(int32_t image_id, Keypoints& keypoints) const {
    sqlite3_stmt* sql_stmt = sql_stmt_read_keypoints_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id));
    const int rc = Sqlite3Check(sqlite3_step(sql_stmt));
    if (rc != SQLITE_ROW) {
        Sqlite3Check(sqlite3_reset(sql_stmt));
        return;
    }
    const int rows = sqlite3_column_int(sql_stmt, 0);
    CHECK_GE(rows, 0);

    ReadMatrixBlob(sql_stmt, rows, 1, keypoints);

    Sqlite3Check(sqlite3_reset(sql_stmt));
}

void Database::WriteKeypoints(int32_t image_id, const Keypoints& keypoints) {
    sqlite3_stmt* sql_stmt = sql_stmt_write_keypoints_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id));
    Sqlite3Check(sqlite3_bind_int(sql_stmt, 2, keypoints.size()));
    WriteMatrixBlob(sql_stmt, keypoints, 3);

    Sqlite3Check(sqlite3_step(sql_stmt));
    Sqlite3Check(sqlite3_reset(sql_stmt));
}

void Database::WriteImagePairFlow(int32_t image_id_from, int32_t image_id_to,
                                  const KeypointsIndices& src_kps_indices,
                                  const Keypoints& tgt_kps,
                                  const FlowErrors& flow_errors) {
    sqlite3_stmt* sql_stmt = sql_stmt_write_image_pair_flows_;

    const size_t rows = src_kps_indices.size();

    CHECK_EQ(tgt_kps.size(), rows);
    CHECK_EQ(flow_errors.size(), rows);

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id_from));
    Sqlite3Check(sqlite3_bind_int(sql_stmt, 2, image_id_to));
    Sqlite3Check(sqlite3_bind_int(sql_stmt, 3, rows));
    WriteMatrixBlob(sql_stmt, src_kps_indices, 4);
    WriteMatrixBlob(sql_stmt, tgt_kps, 5);
    WriteMatrixBlob(sql_stmt, flow_errors, 6);

    Sqlite3Check(sqlite3_step(sql_stmt));
    Sqlite3Check(sqlite3_reset(sql_stmt));
}

void Database::WriteImagePairFlow(const ImagePairFlow& image_pair_flow) {
    WriteImagePairFlow(image_pair_flow.image_id_from,
                       image_pair_flow.image_id_to,
                       image_pair_flow.src_kps_indices, image_pair_flow.tgt_kps,
                       image_pair_flow.flow_errors);
}

void Database::ReadImagePairFlow(int32_t image_id_from, int32_t image_id_to,
                                 ImagePairFlow& image_pair_flow) const {
    sqlite3_stmt* sql_stmt = sql_stmt_read_image_pair_flows_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id_from));
    Sqlite3Check(sqlite3_bind_int(sql_stmt, 2, image_id_to));

    const int rc = Sqlite3Check(sqlite3_step(sql_stmt));
    if (rc != SQLITE_ROW) {
        Sqlite3Check(sqlite3_reset(sql_stmt));
        return;
    }
    const size_t rows = static_cast<size_t>(sqlite3_column_int(sql_stmt, 0));

    ReadMatrixBlob(sql_stmt, rows, 1, image_pair_flow.src_kps_indices);
    ReadMatrixBlob(sql_stmt, rows, 2, image_pair_flow.tgt_kps);
    ReadMatrixBlob(sql_stmt, rows, 3, image_pair_flow.flow_errors);
    image_pair_flow.image_id_from = image_id_from;
    image_pair_flow.image_id_to = image_id_to;

    Sqlite3Check(sqlite3_reset(sql_stmt));
}

ImagePairFlow Database::ReadImagePairFlow(int32_t image_id_from,
                                          int32_t image_id_to) const {
    ImagePairFlow image_pair_flow;
    ReadImagePairFlow(image_id_from, image_id_to, image_pair_flow);

    return image_pair_flow;
}

std::vector<int32_t> Database::FindOpticalFlowsFromImage(
    int32_t image_id_from) const {
    std::vector<int32_t> result;
    FindOpticalFlowsFromImage(image_id_from, result);
    return result;
}

void Database::FindOpticalFlowsFromImage(int32_t image_id_from,
                                         std::vector<int32_t>& result) const {
    sqlite3_stmt* sql_stmt = sql_stmt_find_flows_from_image_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id_from));

    result.clear();
    while (Sqlite3Check(sqlite3_step(sql_stmt)) == SQLITE_ROW) {
        const int32_t image_id_to = sqlite3_column_int(sql_stmt, 0);
        result.push_back(image_id_to);
    }

    Sqlite3Check(sqlite3_reset(sql_stmt));
}

std::vector<int32_t> Database::FindOpticalFlowsToImage(
    int32_t image_id_to) const {
    std::vector<int32_t> result;
    FindOpticalFlowsToImage(image_id_to, result);
    return result;
}

void Database::FindOpticalFlowsToImage(int32_t image_id_to,
                                       std::vector<int32_t>& result) const {
    sqlite3_stmt* sql_stmt = sql_stmt_find_flows_to_image_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id_to));

    result.clear();
    while (Sqlite3Check(sqlite3_step(sql_stmt)) == SQLITE_ROW) {
        const int32_t image_id_from = sqlite3_column_int(sql_stmt, 0);
        result.push_back(image_id_from);
    }

    Sqlite3Check(sqlite3_reset(sql_stmt));
}

bool Database::KeypointsExist(int32_t image_id) const {
    sqlite3_stmt* sql_stmt = sql_stmt_keypoints_exist_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id));
    const bool exists = Sqlite3Check(sqlite3_step(sql_stmt)) == SQLITE_ROW;

    Sqlite3Check(sqlite3_reset(sql_stmt));
    return exists;
}

bool Database::ImagePairFlowExists(int32_t image_id_from,
                                   int32_t image_id_to) const {
    sqlite3_stmt* sql_stmt = sql_stmt_pair_flow_exist_;

    Sqlite3Check(sqlite3_bind_int(sql_stmt, 1, image_id_from));
    Sqlite3Check(sqlite3_bind_int(sql_stmt, 2, image_id_to));
    const bool exists = Sqlite3Check(sqlite3_step(sql_stmt)) == SQLITE_ROW;

    Sqlite3Check(sqlite3_reset(sql_stmt));
    return exists;
}

int32_t Database::GetMinImageIdWithKeypoints() const {
    sqlite3_stmt* sql_stmt = sql_stmt_min_image_id;

    const int rc = Sqlite3Check(sqlite3_step(sql_stmt));
    if (rc != SQLITE_ROW) {
        Sqlite3Check(sqlite3_reset(sql_stmt));
        return kInvalidId;
    }

    const int32_t image_id = sqlite3_column_int(sql_stmt, 0);
    Sqlite3Check(sqlite3_reset(sql_stmt));
    return image_id;
}

int32_t Database::GetMaxImageIdWithKeypoints() const {
    sqlite3_stmt* sql_stmt = sql_stmt_max_image_id;

    const int rc = Sqlite3Check(sqlite3_step(sql_stmt));
    if (rc != SQLITE_ROW) {
        Sqlite3Check(sqlite3_reset(sql_stmt));
        return kInvalidId;
    }

    const int32_t image_id = sqlite3_column_int(sql_stmt, 0);
    Sqlite3Check(sqlite3_reset(sql_stmt));
    return image_id;
}

void Database::PrepareSQLStatements() {
    const char* sql = nullptr;

    sql = "SELECT rows, keypoints FROM keypoints WHERE image_id = ?;";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_read_keypoints_, 0));

    sql = "INSERT INTO keypoints(image_id, rows, keypoints) VALUES(?, ?, ?);";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_write_keypoints_, 0));

    sql =
        "SELECT rows, src_keypoints_indices, tgt_keypoints, flow_errors FROM "
        "optical_flow WHERE image_id_from = ? AND "
        "image_id_to = ?;";
    Sqlite3Check(sqlite3_prepare_v2(database_, sql, -1,
                                    &sql_stmt_read_image_pair_flows_, 0));

    sql =
        "INSERT INTO optical_flow(image_id_from, image_id_to, rows, "
        "src_keypoints_indices, tgt_keypoints, "
        "flow_errors) VALUES(?, ?, ?, ?, ?, ?);";
    Sqlite3Check(sqlite3_prepare_v2(database_, sql, -1,
                                    &sql_stmt_write_image_pair_flows_, 0));

    sql = "SELECT image_id_to FROM optical_flow WHERE image_id_from = ?";
    Sqlite3Check(sqlite3_prepare_v2(database_, sql, -1,
                                    &sql_stmt_find_flows_from_image_, 0));

    sql = "SELECT image_id_from FROM optical_flow WHERE image_id_to = ?";
    Sqlite3Check(sqlite3_prepare_v2(database_, sql, -1,
                                    &sql_stmt_find_flows_to_image_, 0));

    sql = "SELECT 1 FROM keypoints WHERE image_id = ?;";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_keypoints_exist_, 0));

    sql =
        "SELECT 1 FROM optical_flow WHERE image_id_from = ? AND image_id_to = "
        "?;";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_pair_flow_exist_, 0));

    sql = "SELECT MIN(image_id) FROM keypoints;";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_min_image_id, 0));

    sql = "SELECT MAX(image_id) FROM keypoints;";
    Sqlite3Check(
        sqlite3_prepare_v2(database_, sql, -1, &sql_stmt_max_image_id, 0));
}

void Database::FinalizeSQLStatements() noexcept {
    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_read_keypoints_));
    sql_stmt_read_keypoints_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_write_keypoints_));
    sql_stmt_write_keypoints_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_read_image_pair_flows_));
    sql_stmt_read_image_pair_flows_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_write_image_pair_flows_));
    sql_stmt_write_image_pair_flows_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_find_flows_from_image_));
    sql_stmt_find_flows_from_image_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_find_flows_to_image_));
    sql_stmt_find_flows_to_image_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_keypoints_exist_));
    sql_stmt_keypoints_exist_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_pair_flow_exist_));
    sql_stmt_pair_flow_exist_ = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_min_image_id));
    sql_stmt_min_image_id = nullptr;

    Sqlite3CheckNoThrow(sqlite3_finalize(sql_stmt_max_image_id));
    sql_stmt_max_image_id = nullptr;
}

Database::~Database() noexcept { Close(); }
