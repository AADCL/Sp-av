#include <ducted_mapping/static_map_filter.hpp>
#include <limits>
using namespace ducted_mapping;
int main() {
  StaticMapFilterConfig c; c.min_hit_scans=2; c.min_observation_span=0;
  StaticMapFilter f(c);
  f.updateScan({{1,0,0}}, {0,0,0}, 1);
  f.updateScan({{1,0,0}}, {0,0,0}, 1);
  if (!f.points().empty()) return 1; // duplicate stamp is not a second observation
  f.updateScan({{1,0,0}}, {0,0,0}, .5);
  if (!f.points().empty()) return 2;
  f.updateScan({{1,0,0}}, {0,0,0}, 2);
  if (f.points().size()!=1) return 3;
  c.temporal_voxel_size=std::numeric_limits<double>::quiet_NaN();
  try { StaticMapFilter bad(c); return 4; } catch (const std::invalid_argument&) {}
  return 0;
}
