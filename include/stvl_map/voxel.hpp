#pragma once

#include <chrono>

namespace stvl_map {

struct Voxel {
    using Clock = std::chrono::steady_clock;

    bool occupied{false};
    float z_height{0.0F};
    Clock::time_point last_update{};
};

}  // namespace stvl_map
