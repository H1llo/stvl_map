#pragma once

#include <cmath>
#include <cstddef>
#include <algorithm>
#include <stdexcept>
#include <vector>

#include "finenav_common/geometry_types.hpp"

namespace stvl_map {

struct GridIndex {
    int x{0};
    int y{0};
    int z{0};

    friend constexpr bool operator==(const GridIndex&, const GridIndex&) = default;
    friend constexpr GridIndex operator+(GridIndex a, GridIndex b) noexcept {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }
    friend constexpr GridIndex operator-(GridIndex a, GridIndex b) noexcept {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    friend constexpr GridIndex operator-(GridIndex a) noexcept {
        return {-a.x, -a.y, -a.z};
    }
};

struct GridMapConfig {
    finenav::common::Position3D length{6.0, 6.0, 6.0};
    double resolution{0.1};
    finenav::common::Position3D origin{0.0, 0.0, 0.0};
};

/**
 * @brief Fixed-resolution rolling grid backed by a circular vector.
 *
 * Logical indices are centred around zero. `origin` is the world position of
 * logical index zero, and `start_index` maps logical cells to the circular
 * backing storage. Moving the window keeps overlapping cells in place and
 * resets cells that leave the old/new overlap.
 */
template <typename T>
class GridMap {
public:
    using DataType = T;
    using Position = finenav::common::Position3D;
    using Region = finenav::common::Region3D;

    GridMap() = default;

    void configure(const GridMapConfig& config) {
        if (config.resolution <= 0.0 || config.length.x <= 0.0 ||
            config.length.y <= 0.0 || config.length.z <= 0.0) {
            throw std::invalid_argument("GridMap length and resolution must be positive");
        }

        resolution_ = config.resolution;
        origin_ = config.origin;
        size_ = {
            oddCellCount(config.length.x, resolution_),
            oddCellCount(config.length.y, resolution_),
            oddCellCount(config.length.z, resolution_),
        };
        half_size_ = {size_.x / 2, size_.y / 2, size_.z / 2};
        data_.assign(static_cast<std::size_t>(size_.x) *
                         static_cast<std::size_t>(size_.y) *
                         static_cast<std::size_t>(size_.z), T{});
        start_index_ = {0, 0, 0};
        configured_ = true;
    }

    T& at(const GridIndex& index) {
        return data_.at(bufferIndex(index));
    }

    const T& at(const GridIndex& index) const {
        return data_.at(bufferIndex(index));
    }

    T& atPosition(const Position& position) {
        return at(getIndex(position));
    }

    const T& atPosition(const Position& position) const {
        return at(getIndex(position));
    }

    GridIndex getIndex(const Position& position) const {
        return {
            static_cast<int>(std::lround((position.x - origin_.x) / resolution_)),
            static_cast<int>(std::lround((position.y - origin_.y) / resolution_)),
            static_cast<int>(std::lround((position.z - origin_.z) / resolution_)),
        };
    }

    Position getPosition(const GridIndex& index) const {
        return {
            origin_.x + static_cast<double>(index.x) * resolution_,
            origin_.y + static_cast<double>(index.y) * resolution_,
            origin_.z + static_cast<double>(index.z) * resolution_,
        };
    }

    bool isInside(const GridIndex& index) const noexcept {
        return configured_ && std::abs(index.x) <= half_size_.x &&
               std::abs(index.y) <= half_size_.y && std::abs(index.z) <= half_size_.z;
    }

    bool isInside(const Position& position) const noexcept {
        return isInside(getIndex(position));
    }

    Region getWindowBounds() const {
        return {getPosition(-half_size_), getPosition(half_size_)};
    }

    Position getWindowCenter() const noexcept { return origin_; }

    GridIndex getMinIndex() const noexcept { return -half_size_; }
    GridIndex getMaxIndex() const noexcept { return half_size_; }
    GridIndex getSize() const noexcept { return size_; }
    double getResolution() const noexcept { return resolution_; }

    /**
     * @brief Shift the world origin by whole voxels.
     * @param position requested new world origin
     * @param removed optional logical indices cleared while leaving the window
     * @return true when at least one voxel shift was applied
     */
    bool shiftWindowTo(const Position& position,
                       std::vector<GridIndex>* removed = nullptr) {
        if (!configured_) {
            return false;
        }
        if (removed != nullptr) {
            removed->clear();
        }

        const GridIndex shift{
            static_cast<int>(std::lround((position.x - origin_.x) / resolution_)),
            static_cast<int>(std::lround((position.y - origin_.y) / resolution_)),
            static_cast<int>(std::lround((position.z - origin_.z) / resolution_)),
        };
        if (shift == GridIndex{}) {
            return false;
        }

        // For an old logical index i, the same world cell has new index i-shift.
        // Clearing the non-overlap before advancing start_index preserves every
        // retained cell while making newly entered logical cells default-valued.
        for (int x = -half_size_.x; x <= half_size_.x; ++x) {
            for (int y = -half_size_.y; y <= half_size_.y; ++y) {
                for (int z = -half_size_.z; z <= half_size_.z; ++z) {
                    const GridIndex old_index{x, y, z};
                    const GridIndex new_index = old_index - shift;
                    if (!isInside(new_index)) {
                        if (removed != nullptr) {
                            removed->push_back(old_index);
                        }
                        at(old_index) = T{};
                    }
                }
            }
        }

        origin_.x += static_cast<double>(shift.x) * resolution_;
        origin_.y += static_cast<double>(shift.y) * resolution_;
        origin_.z += static_cast<double>(shift.z) * resolution_;
        start_index_ = wrap(start_index_ + shift, size_);
        return true;
    }

private:
    static int oddCellCount(double length, double resolution) {
        int count = static_cast<int>(std::ceil(length / resolution));
        count = std::max(count, 1);
        return (count % 2 == 0) ? count + 1 : count;
    }

    static int wrap(int value, int size) noexcept {
        const int remainder = value % size;
        return remainder < 0 ? remainder + size : remainder;
    }

    static GridIndex wrap(const GridIndex& index, const GridIndex& size) noexcept {
        return {wrap(index.x, size.x), wrap(index.y, size.y), wrap(index.z, size.z)};
    }

    std::size_t bufferIndex(const GridIndex& index) const {
        if (!isInside(index)) {
            throw std::out_of_range("GridMap index is outside the rolling window");
        }
        const GridIndex unwrapped = index + half_size_ + start_index_;
        const GridIndex wrapped = wrap(unwrapped, size_);
        return static_cast<std::size_t>(wrapped.z) +
               static_cast<std::size_t>(wrapped.y) * static_cast<std::size_t>(size_.z) +
               static_cast<std::size_t>(wrapped.x) * static_cast<std::size_t>(size_.y) *
                   static_cast<std::size_t>(size_.z);
    }

    std::vector<T> data_;
    Position origin_{0.0, 0.0, 0.0};
    GridIndex size_{0, 0, 0};
    GridIndex half_size_{0, 0, 0};
    GridIndex start_index_{0, 0, 0};
    double resolution_{1.0};
    bool configured_{false};
};

}  // namespace stvl_map
