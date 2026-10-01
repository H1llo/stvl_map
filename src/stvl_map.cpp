#include "stvl_map/stvl_map.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace stvl_map {

void StvlMap::configure(const ConfigType& config) {
    const bool shape_changed = !configured_ || map_length_ != config.map_length ||
                               resolution_ != config.resolution;

    cloud_topic_ = config.cloud_topic;
    input_frame_ = config.input_frame;
    map_frame_ = config.map_frame;
    map_length_ = config.map_length;
    resolution_ = config.resolution;
    decay_time_sec_ = config.decay_time_sec;
    observation_policy_.max_buffer_size =
        config.max_buffer_size > 0
            ? static_cast<std::size_t>(config.max_buffer_size)
            : 0U;
    observation_policy_.observation_keep_time_sec = config.observation_keep_time_sec;
    observation_policy_.source_timeout_sec = config.source_timeout_sec;
    observation_policy_.expected_update_rate_hz = config.expected_update_rate_hz;

    if (shape_changed) {
        grid_.configure(GridMapConfig{
            {map_length_, map_length_, map_length_}, resolution_, {0.0, 0.0, 0.0}});
        active_cells_.clear();
    }
    configured_ = true;

    RCLCPP_INFO(logger_,
                "StvlMap configured: topic='%s', frames='%s'/'%s', length=%.2f m, "
                "resolution=%.3f m, decay=%.2f s",
                cloud_topic_.c_str(), input_frame_.c_str(), map_frame_.c_str(),
                map_length_, resolution_, decay_time_sec_);
}

void StvlMap::selfManage(finenav::core::MapRuntime& runtime) {
    runtime.addObservationSource<sensor_msgs::msg::PointCloud2>(
        "point_cloud", cloud_topic_, rclcpp::SensorDataQoS(), observation_policy_,
        [this](const sensor_msgs::msg::PointCloud2& cloud,
               const rclcpp::Time&, const finenav::core::RobotState& state) {
            insertPointCloud(cloud, state);
        });

    runtime.registerPreUpdateHook([this](const finenav::core::RobotState&) {
        pruneExpiredCells();
    });
}

void StvlMap::insertPointCloud(const sensor_msgs::msg::PointCloud2& cloud,
                               const finenav::core::RobotState& state,
                               Clock::time_point now) {
    if (!configured_) {
        RCLCPP_WARN(logger_, "StvlMap received a point cloud before configure(); dropping it.");
        return;
    }

    const bool in_map_frame = cloud.header.frame_id == map_frame_;
    const bool in_input_frame = cloud.header.frame_id == input_frame_;
    if (!in_map_frame && !in_input_frame) {
        RCLCPP_WARN(
            logger_,
            "StvlMap: dropping cloud in unexpected frame '%s' (expected '%s' or '%s').",
            cloud.header.frame_id.c_str(), input_frame_.c_str(), map_frame_.c_str());
        return;
    }

    Eigen::Quaterniond orientation(
        state.pose.orientation.w, state.pose.orientation.x,
        state.pose.orientation.y, state.pose.orientation.z);
    if (orientation.norm() < std::numeric_limits<double>::epsilon()) {
        orientation = Eigen::Quaterniond::Identity();
    } else {
        orientation.normalize();
    }
    const Eigen::Vector3d translation(
        state.pose.position.x, state.pose.position.y, state.pose.position.z);

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
                !std::isfinite(*iter_z)) {
                continue;
            }
            Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
            if (in_input_frame) {
                point = orientation * point + translation;
            }
            insertPoint({point.x(), point.y(), point.z()}, now);
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN(logger_, "StvlMap: dropping malformed PointCloud2: %s", error.what());
    }
}

void StvlMap::insertPoint(const finenav::common::Position3D& position,
                          Clock::time_point now) {
    if (!grid_.isInside(position)) {
        return;
    }
    const GridIndex index = grid_.getIndex(position);
    auto& voxel = grid_.at(index);
    voxel.occupied = true;
    voxel.z_height = static_cast<float>(position.z);
    voxel.last_update = now;
    active_cells_.insert(index);
}

void StvlMap::pruneExpiredCells(Clock::time_point now) {
    if (!configured_ || decay_time_sec_ <= 0.0) {
        return;
    }

    for (auto it = active_cells_.begin(); it != active_cells_.end();) {
        const GridIndex index = *it;
        if (!grid_.isInside(index)) {
            it = active_cells_.erase(it);
            continue;
        }

        const Voxel& voxel = grid_.at(index);
        const double age = std::chrono::duration<double>(now - voxel.last_update).count();
        if (!voxel.occupied || age > decay_time_sec_) {
            grid_.at(index) = Voxel{};
            it = active_cells_.erase(it);
        } else {
            ++it;
        }
    }
}

void StvlMap::shiftWindowTo(const finenav::common::Position3D& position) {
    const auto old_center = grid_.getWindowCenter();
    const auto resolution = grid_.getResolution();
    const GridIndex shift{
        static_cast<int>(std::lround((position.x - old_center.x) / resolution)),
        static_cast<int>(std::lround((position.y - old_center.y) / resolution)),
        static_cast<int>(std::lround((position.z - old_center.z) / resolution)),
    };
    if (shift == GridIndex{}) {
        return;
    }

    grid_.shiftWindowTo(position);

    std::unordered_set<GridIndex, ActiveIndexHash> shifted_cells;
    shifted_cells.reserve(active_cells_.size());
    for (const auto& old_index : active_cells_) {
        const GridIndex new_index = old_index - shift;
        if (grid_.isInside(new_index) && grid_.at(new_index).occupied) {
            shifted_cells.insert(new_index);
        }
    }
    active_cells_ = std::move(shifted_cells);
}

bool StvlMap::isOccupied(const finenav::common::Position3D& position) const {
    if (!grid_.isInside(position)) {
        return false;
    }
    return grid_.at(grid_.getIndex(position)).occupied;
}

}  // namespace stvl_map
