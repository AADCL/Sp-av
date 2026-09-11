// Adapted from AADCL/AG-TEST 9a309a93fd900abccb8775ebdb727c65946f5c0d, Apache-2.0.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ducted_mapping/voxel_accumulator.hpp"

namespace ducted_mapping {

struct StaticMapFilterConfig {
    double temporal_voxel_size = 0.20;
    double map_voxel_size = 0.10;
    double hit_probability = 0.70;
    double miss_probability = 0.40;
    double occupied_probability = 0.72;
    double clearing_probability = 0.35;
    std::uint32_t min_hit_scans = 8;
    double min_observation_span = 2.0;
    std::size_t ray_stride = 4;
    double max_clearing_range = 20.0;
    double ray_endpoint_margin = 0.30;
    double candidate_timeout = 5.0;
    double cleanup_period = 1.0;
    std::size_t max_temporal_voxels = 2000000;
    std::size_t max_map_voxels = 5000000;
};

class StaticMapFilter {
public:
    explicit StaticMapFilter(const StaticMapFilterConfig& config)
        : config_(validated(config)),
          hit_log_odds_(probabilityToLogOdds(config_.hit_probability)),
          miss_log_odds_(probabilityToLogOdds(config_.miss_probability)),
          occupied_log_odds_(probabilityToLogOdds(config_.occupied_probability)),
          clearing_log_odds_(probabilityToLogOdds(config_.clearing_probability)) {}

    void updateScan(const std::vector<Point3d>& endpoints,
                    const Point3d& sensor_position,
                    double stamp_seconds) {
        if (!finite(sensor_position) || !std::isfinite(stamp_seconds) || stamp_seconds <= last_stamp_) return;
        last_stamp_ = stamp_seconds;
        ++scan_sequence_;

        std::unordered_set<Key, KeyHash> occupied_this_scan;
        occupied_this_scan.reserve(endpoints.size());
        for (const Point3d& point : endpoints) {
            if (finite(point)) occupied_this_scan.insert(key(point, config_.temporal_voxel_size));
        }

        for (const Point3d& point : endpoints) {
            if (!finite(point)) continue;
            const Key occupancy_key = key(point, config_.temporal_voxel_size);
            OccupancyState* occupancy = updateHit(occupancy_key, stamp_seconds);
            if (occupancy == nullptr) continue;

            const Key map_key = key(point, config_.map_voxel_size);
            auto found = map_voxels_.find(map_key);
            if (found == map_voxels_.end()) {
                if (map_voxels_.size() >= config_.max_map_voxels) {
                    capacity_exceeded_ = true;
                    continue;
                }
                found = map_voxels_.emplace(map_key, MapState()).first;
            }
            MapState& state = found->second;
            if (state.count == 0 || state.occupancy_generation != occupancy->generation) {
                state = MapState();
                state.occupancy_key = occupancy_key;
                state.occupancy_generation = occupancy->generation;
            }
            state.x += point.x;
            state.y += point.y;
            state.z += point.z;
            ++state.count;
        }

        updateFreeSpace(sensor_position, endpoints, occupied_this_scan, stamp_seconds);
        cleanup(stamp_seconds);
    }

    std::vector<Point3d> points() const {
        std::vector<std::pair<Key, Point3d>> keyed;
        keyed.reserve(map_voxels_.size());
        for (const auto& item : map_voxels_) {
            const MapState& map_state = item.second;
            if (map_state.count == 0) continue;
            const auto occupancy = temporal_voxels_.find(map_state.occupancy_key);
            if (occupancy == temporal_voxels_.end() ||
                occupancy->second.generation != map_state.occupancy_generation ||
                !isStatic(occupancy->second)) {
                continue;
            }
            const double count = static_cast<double>(map_state.count);
            keyed.push_back({item.first,
                             {map_state.x / count, map_state.y / count, map_state.z / count}});
        }
        std::sort(keyed.begin(), keyed.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.first.x != rhs.first.x) return lhs.first.x < rhs.first.x;
            if (lhs.first.y != rhs.first.y) return lhs.first.y < rhs.first.y;
            return lhs.first.z < rhs.first.z;
        });
        std::vector<Point3d> result;
        result.reserve(keyed.size());
        for (const auto& item : keyed) result.push_back(item.second);
        return result;
    }

    std::size_t size() const {
        std::size_t confirmed = 0;
        for (const auto& item : map_voxels_) {
            const MapState& map_state = item.second;
            const auto occupancy = temporal_voxels_.find(map_state.occupancy_key);
            if (map_state.count > 0 && occupancy != temporal_voxels_.end() &&
                occupancy->second.generation == map_state.occupancy_generation &&
                isStatic(occupancy->second)) {
                ++confirmed;
            }
        }
        return confirmed;
    }

    std::size_t temporalSize() const { return temporal_voxels_.size(); }
    bool capacityExceeded() const { return capacity_exceeded_; }

    bool isStaticPoint(const Point3d& point) const {
        if (!finite(point)) return false;
        const auto found = temporal_voxels_.find(key(point, config_.temporal_voxel_size));
        return found != temporal_voxels_.end() && isStatic(found->second);
    }

    void clear() {
        temporal_voxels_.clear();
        map_voxels_.clear();
        scan_sequence_ = 0;
        last_stamp_ = -std::numeric_limits<double>::infinity();
        next_generation_ = 1;
        last_cleanup_ = -std::numeric_limits<double>::infinity();
        capacity_exceeded_ = false;
    }

private:
    double last_stamp_ = -std::numeric_limits<double>::infinity();
    struct Key {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;
        bool operator==(const Key& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key& value) const {
            std::size_t seed = std::hash<std::int64_t>{}(value.x);
            seed ^= std::hash<std::int64_t>{}(value.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<std::int64_t>{}(value.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct OccupancyState {
        double log_odds = 0.0;
        std::uint32_t hit_scans = 0;
        std::uint64_t last_hit_scan = 0;
        std::uint64_t last_miss_scan = 0;
        double first_hit = 0.0;
        double last_hit = 0.0;
        double last_seen = 0.0;
        std::uint64_t generation = 0;
        bool has_hit = false;
    };

    struct MapState {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        std::size_t count = 0;
        Key occupancy_key;
        std::uint64_t occupancy_generation = 0;
    };

    static bool finite(const Point3d& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)
            && std::abs(point.x) < 10000 && std::abs(point.y) < 10000 && std::abs(point.z) < 10000;
    }

    static double clampProbability(double value) {
        return std::max(0.001, std::min(0.999, value));
    }

    static double probabilityToLogOdds(double probability) {
        const double value = clampProbability(probability);
        return std::log(value / (1.0 - value));
    }

    static StaticMapFilterConfig validated(StaticMapFilterConfig config) {
        for (double value : {config.temporal_voxel_size, config.map_voxel_size,
             config.hit_probability, config.miss_probability, config.occupied_probability,
             config.clearing_probability, config.min_observation_span,
             config.max_clearing_range, config.ray_endpoint_margin,
             config.candidate_timeout, config.cleanup_period}) {
            if (!std::isfinite(value)) throw std::invalid_argument("nonfinite filter parameter");
        }
        if (config.temporal_voxel_size < .001 || config.map_voxel_size < .001)
            throw std::invalid_argument("voxel resolution below supported 1mm minimum");
        if (!(config.temporal_voxel_size > 0.0) || !(config.map_voxel_size > 0.0) ||
            config.max_temporal_voxels == 0 || config.max_map_voxels == 0) {
            throw std::invalid_argument("voxel sizes and capacities must be positive");
        }
        config.hit_probability = clampProbability(config.hit_probability);
        config.miss_probability = clampProbability(config.miss_probability);
        config.occupied_probability = clampProbability(config.occupied_probability);
        config.clearing_probability = clampProbability(config.clearing_probability);
        if (config.hit_probability <= 0.5 || config.miss_probability >= 0.5 ||
            config.clearing_probability >= config.occupied_probability) {
            throw std::invalid_argument("invalid Bayesian probabilities");
        }
        config.min_hit_scans = std::max<std::uint32_t>(1, config.min_hit_scans);
        config.min_observation_span = std::max(0.0, config.min_observation_span);
        config.ray_stride = std::max<std::size_t>(1, config.ray_stride);
        config.max_clearing_range = std::max(config.temporal_voxel_size,
                                             config.max_clearing_range);
        config.ray_endpoint_margin = std::max(config.temporal_voxel_size,
                                              config.ray_endpoint_margin);
        config.candidate_timeout = std::max(0.0, config.candidate_timeout);
        config.cleanup_period = std::max(0.0, config.cleanup_period);
        return config;
    }

    Key key(const Point3d& point, double voxel_size) const {
        return {static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
                static_cast<std::int64_t>(std::floor(point.y / voxel_size)),
                static_cast<std::int64_t>(std::floor(point.z / voxel_size))};
    }

    bool isStatic(const OccupancyState& state) const {
        return state.has_hit && state.hit_scans >= config_.min_hit_scans &&
               state.log_odds >= occupied_log_odds_ &&
               state.last_hit - state.first_hit >= config_.min_observation_span;
    }

    OccupancyState* updateHit(const Key& voxel_key, double stamp) {
        auto found = temporal_voxels_.find(voxel_key);
        if (found == temporal_voxels_.end()) {
            if (temporal_voxels_.size() >= config_.max_temporal_voxels) {
                capacity_exceeded_ = true;
                return nullptr;
            }
            found = temporal_voxels_.emplace(voxel_key, OccupancyState()).first;
            found->second.generation = next_generation_++;
        }
        OccupancyState& state = found->second;
        state.last_seen = stamp;
        if (state.last_hit_scan == scan_sequence_) return &state;
        state.last_hit_scan = scan_sequence_;
        state.log_odds = std::min(max_log_odds_, state.log_odds + hit_log_odds_);
        ++state.hit_scans;
        if (!state.has_hit) {
            state.first_hit = stamp;
            state.has_hit = true;
        }
        state.last_hit = stamp;
        return &state;
    }

    void updateMiss(const Key& voxel_key, double stamp) {
        auto found = temporal_voxels_.find(voxel_key);
        if (found == temporal_voxels_.end()) return;
        OccupancyState& state = found->second;
        if (state.last_miss_scan == scan_sequence_) return;
        state.last_miss_scan = scan_sequence_;
        state.last_seen = stamp;
        state.log_odds = std::max(min_log_odds_, state.log_odds + miss_log_odds_);
        if (state.log_odds <= clearing_log_odds_) temporal_voxels_.erase(found);
    }

    void updateFreeSpace(const Point3d& sensor_position,
                         const std::vector<Point3d>& endpoints,
                         const std::unordered_set<Key, KeyHash>& occupied_this_scan,
                         double stamp) {
        for (std::size_t index = 0; index < endpoints.size(); index += config_.ray_stride) {
            const Point3d& endpoint = endpoints[index];
            if (!finite(endpoint)) continue;
            const double dx = endpoint.x - sensor_position.x;
            const double dy = endpoint.y - sensor_position.y;
            const double dz = endpoint.z - sensor_position.z;
            const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (range <= config_.ray_endpoint_margin) continue;
            const double clear_until = std::min(config_.max_clearing_range,
                                                range - config_.ray_endpoint_margin);
            if (clear_until <= config_.temporal_voxel_size) continue;
            const std::size_t steps = static_cast<std::size_t>(
                std::floor(clear_until / config_.temporal_voxel_size));
            Key previous;
            bool have_previous = false;
            for (std::size_t step = 1; step <= steps; ++step) {
                const double distance = static_cast<double>(step) * config_.temporal_voxel_size;
                Point3d sample{sensor_position.x + dx * distance / range,
                               sensor_position.y + dy * distance / range,
                               sensor_position.z + dz * distance / range};
                const Key sample_key = key(sample, config_.temporal_voxel_size);
                if (have_previous && sample_key == previous) continue;
                previous = sample_key;
                have_previous = true;
                if (occupied_this_scan.find(sample_key) == occupied_this_scan.end()) {
                    updateMiss(sample_key, stamp);
                }
            }
        }
    }

    void cleanup(double now) {
        if (now - last_cleanup_ < config_.cleanup_period &&
            temporal_voxels_.size() <= config_.max_temporal_voxels) {
            return;
        }
        last_cleanup_ = now;
        for (auto it = temporal_voxels_.begin(); it != temporal_voxels_.end();) {
            if (!isStatic(it->second) && now - it->second.last_seen > config_.candidate_timeout) {
                it = temporal_voxels_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = map_voxels_.begin(); it != map_voxels_.end();) {
            const auto occupancy = temporal_voxels_.find(it->second.occupancy_key);
            if (occupancy == temporal_voxels_.end() ||
                occupancy->second.generation != it->second.occupancy_generation) {
                it = map_voxels_.erase(it);
            } else {
                ++it;
            }
        }
    }

    StaticMapFilterConfig config_;
    double hit_log_odds_;
    double miss_log_odds_;
    double occupied_log_odds_;
    double clearing_log_odds_;
    std::uint64_t scan_sequence_ = 0;
    std::uint64_t next_generation_ = 1;
    double last_cleanup_ = -std::numeric_limits<double>::infinity();
    bool capacity_exceeded_ = false;
    const double min_log_odds_ = -20.0;
    const double max_log_odds_ = 20.0;
    std::unordered_map<Key, OccupancyState, KeyHash> temporal_voxels_;
    std::unordered_map<Key, MapState, KeyHash> map_voxels_;
};

}  // namespace ducted_mapping
