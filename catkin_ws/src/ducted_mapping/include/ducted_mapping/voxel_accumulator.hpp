// Adapted from AADCL/AG-TEST 9a309a93fd900abccb8775ebdb727c65946f5c0d, Apache-2.0.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ducted_mapping {

struct Point3d {
    double x;
    double y;
    double z;
};

class VoxelAccumulator {
public:
    VoxelAccumulator(double voxel_size, std::size_t max_voxels)
        : voxel_size_(voxel_size), max_voxels_(max_voxels) {
        if (!(voxel_size_ > 0.0) || max_voxels_ == 0) {
            throw std::invalid_argument("voxel size and capacity must be positive");
        }
    }

    bool add(double x, double y, double z) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
        const Key key{index(x), index(y), index(z)};
        auto found = voxels_.find(key);
        if (found != voxels_.end()) {
            found->second.x += x;
            found->second.y += y;
            found->second.z += z;
            ++found->second.count;
            return true;
        }
        if (voxels_.size() >= max_voxels_) {
            capacity_exceeded_ = true;
            return false;
        }
        voxels_.emplace(key, Sum{x, y, z, 1});
        return true;
    }

    std::size_t size() const { return voxels_.size(); }
    bool capacityExceeded() const { return capacity_exceeded_; }

    void clear() {
        voxels_.clear();
        capacity_exceeded_ = false;
    }

    std::vector<Point3d> centroids() const {
        std::vector<std::pair<Key, Point3d>> keyed;
        keyed.reserve(voxels_.size());
        for (const auto& item : voxels_) {
            const Sum& sum = item.second;
            const double count = static_cast<double>(sum.count);
            keyed.push_back({item.first, {sum.x / count, sum.y / count, sum.z / count}});
        }
        std::sort(keyed.begin(), keyed.end(), [](const auto& a, const auto& b) {
            if (a.first.x != b.first.x) return a.first.x < b.first.x;
            if (a.first.y != b.first.y) return a.first.y < b.first.y;
            return a.first.z < b.first.z;
        });
        std::vector<Point3d> result;
        result.reserve(keyed.size());
        for (const auto& item : keyed) result.push_back(item.second);
        return result;
    }

private:
    struct Key {
        std::int64_t x;
        std::int64_t y;
        std::int64_t z;
        bool operator==(const Key& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key& key) const {
            std::size_t seed = std::hash<std::int64_t>{}(key.x);
            seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct Sum {
        double x;
        double y;
        double z;
        std::size_t count;
    };

    std::int64_t index(double value) const {
        return static_cast<std::int64_t>(std::floor(value / voxel_size_));
    }

    double voxel_size_;
    std::size_t max_voxels_;
    bool capacity_exceeded_ = false;
    std::unordered_map<Key, Sum, KeyHash> voxels_;
};

}  // namespace ducted_mapping
