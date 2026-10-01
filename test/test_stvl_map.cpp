#include <chrono>
#include <cmath>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "stvl_map/stvl_map.hpp"

namespace {

sensor_msgs::msg::PointCloud2 onePointCloud(const std::string& frame,
                                            float x, float y, float z) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
    *ix = x;
    *iy = y;
    *iz = z;
    return msg;
}

stvl_map::StvlMap configuredMap() {
    stvl_map::StvlMap map;
    stvl_map::Params cfg;
    cfg.map_length = 10.0;
    cfg.resolution = 1.0;
    cfg.input_frame = "base_link";
    cfg.map_frame = "map";
    cfg.decay_time_sec = 1.0;
    map.configure(cfg);
    return map;
}

finenav::core::RobotState identityState() {
    finenav::core::RobotState state;
    state.pose.orientation.w = 1.0;
    return state;
}

TEST(StvlMap, InsertsCloudAlreadyInMapFrame) {
    auto map = configuredMap();
    const auto now = stvl_map::StvlMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 2.0F, 3.0F), identityState(), now);

    EXPECT_TRUE(map.isOccupied({1.0, 2.0, 3.0}));
    EXPECT_EQ(map.occupiedCellCount(), 1U);
}

TEST(StvlMap, TransformsCloudFromInputFrameUsingRobotStatePose) {
    auto map = configuredMap();
    auto state = identityState();
    state.pose.position.x = 2.0;
    state.pose.position.y = 3.0;
    state.pose.orientation.w = std::sqrt(0.5);
    state.pose.orientation.z = std::sqrt(0.5);
    const auto now = stvl_map::StvlMap::Clock::time_point{};

    map.insertPointCloud(onePointCloud("base_link", 1.0F, 0.0F, 0.0F), state, now);

    EXPECT_TRUE(map.isOccupied({2.0, 4.0, 0.0}));
}

TEST(StvlMap, RejectsUnexpectedCloudFrame) {
    auto map = configuredMap();
    map.insertPointCloud(onePointCloud("camera_link", 1.0F, 2.0F, 3.0F), identityState(),
                         stvl_map::StvlMap::Clock::time_point{});

    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

TEST(StvlMap, PrunesCellsPastConfiguredLifetime) {
    auto map = configuredMap();
    const auto t0 = stvl_map::StvlMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 2.0F, 3.0F), identityState(), t0);

    map.pruneExpiredCells(t0 + std::chrono::milliseconds(1100));

    EXPECT_FALSE(map.isOccupied({1.0, 2.0, 3.0}));
    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

TEST(StvlMap, UpdatesActiveIndicesWhenWindowRolls) {
    auto map = configuredMap();
    const auto t0 = stvl_map::StvlMap::Clock::time_point{};
    map.insertPointCloud(onePointCloud("map", 1.0F, 0.0F, 0.0F), identityState(), t0);

    map.shiftWindowTo({3.0, 0.0, 0.0});
    EXPECT_TRUE(map.isOccupied({1.0, 0.0, 0.0}));
    EXPECT_EQ(map.occupiedCellCount(), 1U);

    map.pruneExpiredCells(t0 + std::chrono::milliseconds(1100));
    EXPECT_EQ(map.occupiedCellCount(), 0U);
}

}  // namespace
