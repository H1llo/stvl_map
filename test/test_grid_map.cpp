#include <gtest/gtest.h>

#include "stvl_map/grid_map.hpp"

namespace {

TEST(GridMap, ConvertsWorldPositionsToRoundedLogicalIndices) {
    stvl_map::GridMap<int> grid;
    stvl_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    grid.configure(config);

    const auto index = grid.getIndex(finenav::common::Position3D{0.49, -0.49, 1.51});
    EXPECT_EQ(index.x, 0);
    EXPECT_EQ(index.y, 0);
    EXPECT_EQ(index.z, 2);
    EXPECT_TRUE(grid.isInside(finenav::common::Position3D{2.0, -2.0, 2.0}));
    EXPECT_FALSE(grid.isInside(finenav::common::Position3D{3.0, 0.0, 0.0}));
}

TEST(GridMap, ShiftRetainsOverlapAndClearsCellsLeavingWindow) {
    stvl_map::GridMap<int> grid;
    stvl_map::GridMapConfig config;
    config.length = {5.0, 5.0, 5.0};
    config.resolution = 1.0;
    config.origin = {0.0, 0.0, 0.0};
    grid.configure(config);
    grid.atPosition(finenav::common::Position3D{1.0, 0.0, 0.0}) = 7;
    grid.atPosition(finenav::common::Position3D{0.0, 0.0, 0.0}) = 4;

    grid.shiftWindowTo(finenav::common::Position3D{3.0, 0.0, 0.0});

    const auto center = grid.getWindowCenter();
    EXPECT_DOUBLE_EQ(center.x, 3.0);
    EXPECT_DOUBLE_EQ(center.y, 0.0);
    EXPECT_DOUBLE_EQ(center.z, 0.0);
    EXPECT_TRUE(grid.isInside(finenav::common::Position3D{1.0, 0.0, 0.0}));
    EXPECT_EQ(grid.atPosition(finenav::common::Position3D{1.0, 0.0, 0.0}), 7);
    EXPECT_FALSE(grid.isInside(finenav::common::Position3D{0.0, 0.0, 0.0}));
}

}  // namespace
