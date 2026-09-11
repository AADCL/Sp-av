#include <vector>

#include "ducted_mapping/static_map_filter.hpp"

using ducted_mapping::Point3d;
using ducted_mapping::StaticMapFilter;
using ducted_mapping::StaticMapFilterConfig;

int main() {
    StaticMapFilterConfig config;
    config.temporal_voxel_size = 0.20;
    config.map_voxel_size = 0.05;
    config.min_hit_scans = 8;
    config.min_observation_span = 2.0;
    config.ray_stride = 1;
    config.max_clearing_range = 5.0;
    config.ray_endpoint_margin = 0.20;
    config.candidate_timeout = 1.0;
    config.cleanup_period = 0.0;
    config.max_temporal_voxels = 128;
    config.max_map_voxels = 256;

    StaticMapFilter filter(config);
    const Point3d sensor{0.0, 0.0, 0.0};
    const std::vector<Point3d> wall{{1.01, 0.01, 0.01}};

    for (int scan = 0; scan < 7; ++scan) {
        filter.updateScan(wall, sensor, 1.0 + 0.3 * scan);
    }
    if (!filter.points().empty()) return 1;

    filter.updateScan(wall, sensor, 3.1);
    if (filter.points().size() != 1) return 2;

    const std::vector<Point3d> farther_surface{{2.01, 0.01, 0.01}};
    for (int scan = 0; scan < 32; ++scan) {
        filter.updateScan(farther_surface, sensor, 3.2 + 0.1 * scan);
    }
    const auto after_clearing = filter.points();
    for (const auto& point : after_clearing) {
        if (point.x <= 1.5) return 3;
    }

    filter.clear();
    if (!filter.points().empty()) return 4;
    if (filter.capacityExceeded()) return 5;

    // Repeated points in one scan count as one temporal observation.
    StaticMapFilterConfig duplicate_config = config;
    duplicate_config.min_hit_scans = 2;
    duplicate_config.min_observation_span = 0.0;
    StaticMapFilter duplicate_filter(duplicate_config);
    duplicate_filter.updateScan({wall.front(), wall.front(), wall.front()}, sensor, 1.0);
    if (!duplicate_filter.points().empty()) return 6;
    duplicate_filter.updateScan(wall, sensor, 1.1);
    if (duplicate_filter.points().size() != 1) return 7;

    // A candidate that never becomes static is removed after its timeout.
    StaticMapFilter timeout_filter(config);
    timeout_filter.updateScan(wall, sensor, 1.0);
    if (timeout_filter.temporalSize() != 1) return 8;
    timeout_filter.updateScan({}, sensor, 2.1);
    if (timeout_filter.temporalSize() != 0) return 9;

    StaticMapFilterConfig capacity_config = config;
    capacity_config.max_temporal_voxels = 1;
    capacity_config.max_map_voxels = 1;
    StaticMapFilter capacity_filter(capacity_config);
    capacity_filter.updateScan({{1.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}, sensor, 1.0);
    if (!capacity_filter.capacityExceeded()) return 10;
    return 0;
}
