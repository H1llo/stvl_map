#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "finenav_core/plugin_map.hpp"
#include "stvl_map/stvl_map_params.hpp"
#include "stvl_map/grid_map.hpp"
#include "stvl_map/voxel.hpp"

namespace stvl_map {

// STVL-specific sparse index policy. GridMap only defines GridIndex and maps
// logical indices to its circular backing storage; it does not own this hash.
struct ActiveIndexHash {
    std::size_t operator()(const GridIndex& index) const noexcept {
        const auto mix = [](std::size_t value) {
            value ^= value >> 30U;
            value *= static_cast<std::size_t>(0xbf58476d1ce4e5b9ULL);
            value ^= value >> 27U;
            value *= static_cast<std::size_t>(0x94d049bb133111ebULL);
            return value ^ (value >> 31U);
        };
        const auto ux = static_cast<std::size_t>(static_cast<std::uint32_t>(index.x));
        const auto uy = static_cast<std::size_t>(static_cast<std::uint32_t>(index.y));
        const auto uz = static_cast<std::size_t>(static_cast<std::uint32_t>(index.z));
        return mix(ux) ^ (mix(uy) << 1U) ^ (mix(uz) << 2U);
    }
};

class StvlMap {
public:
    using ConfigType = Params;
    using DataType = Voxel;
    using Clock = Voxel::Clock;

    StvlMap() = default;

    void configure(const ConfigType& config);
    void selfManage(finenav::core::MapRuntime& runtime);

    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    bool isInside(const finenav::common::Position3D& position) const {
        return grid_.isInside(position);
    }
    finenav::common::Region3D getWindowBounds() const {
        return grid_.getWindowBounds();
    }
    finenav::common::Position3D getWindowCenter() const {
        return grid_.getWindowCenter();
    }
    void shiftWindowTo(const finenav::common::Position3D& position);

    void insertPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                          const finenav::core::RobotState& state,
                          Clock::time_point now = Clock::now());
    void pruneExpiredCells(Clock::time_point now = Clock::now());

    bool isOccupied(const finenav::common::Position3D& position) const;
    std::size_t occupiedCellCount() const noexcept { return active_cells_.size(); }

private:
    void insertPoint(const finenav::common::Position3D& position,
                     Clock::time_point now);

    GridMap<Voxel> grid_;
    std::unordered_set<GridIndex, ActiveIndexHash> active_cells_;

    std::string cloud_topic_{"/points"};
    std::string input_frame_{"base_link"};
    std::string map_frame_{"map"};
    double map_length_{20.0};
    double resolution_{0.2};
    double decay_time_sec_{5.0};
    finenav::core::ObservationBufferPolicy observation_policy_{};
    rclcpp::Logger logger_{rclcpp::get_logger("StvlMap")};
    bool configured_{false};
};

}  // namespace stvl_map
