#pragma once
#include <array>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#include <ducted_mapping/voxel_accumulator.hpp>

namespace ducted_mapping {
// Observed cells only. Absence means UNKNOWN, never free. Raw endpoints are
// retained here even when the static map filter classifies them as transient.
class ObservedGrid {
public:
  using Key = std::array<int, 3>;
  struct Hash { size_t operator()(const Key& k) const {
    size_t h=0; for (int v:k) h ^= std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2); return h;
  }};
  explicit ObservedGrid(double resolution=.2, size_t capacity=2000000)
    : resolution_(resolution), capacity_(capacity) {
    if (!std::isfinite(resolution) || resolution<.01 || capacity==0)
      throw std::invalid_argument("invalid observed grid limits");
  }
  Key key(const Point3d& p) const {
    if (!std::isfinite(p.x+p.y+p.z) || std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)})>10000)
      throw std::invalid_argument("grid point out of bounds");
    return {{int(std::floor(p.x/resolution_)),int(std::floor(p.y/resolution_)),int(std::floor(p.z/resolution_))}};
  }
  Point3d center(const Key& k) const { return {(k[0]+.5)*resolution_,(k[1]+.5)*resolution_,(k[2]+.5)*resolution_}; }
  void scan(const std::vector<Point3d>& endpoints, const Point3d& sensor, double max_range=20) {
    std::unordered_set<Key,Hash> hits, misses;
    for (const auto& p:endpoints) hits.insert(key(p));
    // Amanatides-Woo traversal, exact voxel crossings; capped rays never mark
    // their artificial endpoints occupied. Near-return cells are not cleared.
    for (size_t n=0;n<endpoints.size();n+=4) {
      const auto& p=endpoints[n]; double d[3]={p.x-sensor.x,p.y-sensor.y,p.z-sensor.z};
      double length=std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
      if (length<=.3) continue;
      double limit=std::min(max_range,length-.3)/length;
      Key k=key(sensor); double s[3]={sensor.x,sensor.y,sensor.z}, next[3], delta[3]; int step[3];
      for(int a=0;a<3;++a) {
        step[a]=d[a]>0?1:-1;
        delta[a]=d[a]==0?INFINITY:resolution_/std::abs(d[a]);
        next[a]=d[a]==0?INFINITY:((k[a]+(step[a]>0?1:0))*resolution_-s[a])/d[a];
      }
      for (size_t count=0;count<10000;++count) {
        if (!hits.count(k)) misses.insert(k);
        double t=std::min({next[0],next[1],next[2]}); if(t>=limit) break;
        for(int a=0;a<3;++a) if(next[a]<=t+1e-12) { k[a]+=step[a]; next[a]+=delta[a]; }
      }
    }
    for(const auto& k:misses) update(k,-.4054651081);
    for(const auto& k:hits) update(k,.8472978604);
  }
  const std::unordered_map<Key,float,Hash>& cells() const { return cells_; }
  double resolution() const { return resolution_; }
private:
  void update(const Key& k,double increment) {
    if (!cells_.count(k) && cells_.size()>=capacity_) throw std::runtime_error("observed grid capacity exceeded");
    float& v=cells_[k]; v=std::max(-3.5,std::min(3.5,v+increment));
  }
  double resolution_; size_t capacity_; std::unordered_map<Key,float,Hash> cells_;
};
}
