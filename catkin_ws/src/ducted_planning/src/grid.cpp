#include <ducted_planning/grid.hpp>
#include <chrono>
#include <queue>
#include <algorithm>
#include <stdexcept>

namespace ducted_planning {
Grid::Grid(const Eigen::Vector3d& o,const Eigen::Vector3d& s,double r):origin(o),size(s),resolution(r) {
  if(!o.allFinite()||!s.allFinite()||!std::isfinite(r)||r<.05||s.minCoeff()<=0||s.maxCoeff()>200)
    throw std::invalid_argument("invalid planning grid bounds");
  dims=(s/r).array().ceil().cast<int>(); size=dims.cast<double>()*r;
  size_t count=size_t(dims.x())*dims.y()*dims.z();
  if(count>2000000) throw std::invalid_argument("planning grid exceeds two million cells");
  free.assign(count,0); distances_.assign(count,0);
}
int Grid::address(const Eigen::Vector3i& k) const {
  if((k.array()<0).any()||(k.array()>=dims.array()).any()) return -1;
  return (k.x()*dims.y()+k.y())*dims.z()+k.z();
}
Eigen::Vector3i Grid::index(const Eigen::Vector3d& p) const {
  if(!p.allFinite() || (p-origin).cwiseAbs().maxCoeff()>10000) return Eigen::Vector3i(-1,-1,-1);
  return ((p-origin)/resolution).array().floor().cast<int>();
}
Eigen::Vector3i Grid::index(int i) const {return {i/(dims.y()*dims.z()),(i/dims.z())%dims.y(),i%dims.z()};}
Eigen::Vector3d Grid::center(int i) const {return origin+(index(i).cast<double>()+Eigen::Vector3d::Constant(.5))*resolution;}
bool Grid::inside(const Eigen::Vector3d& p) const {return address(index(p))>=0;}
void Grid::setFree(const Eigen::Vector3i& k,bool value) {int i=address(k);if(i>=0)free[i]=value;}
void Grid::computeDistances() {
  // Separable squared Euclidean distance transform (lower envelope of
  // parabolas). Unknown and boundary cells seed the same blocked field.
  for(size_t i=0;i<free.size();++i) {
    auto k=index(i); bool boundary=(k.array()==0).any()||(k.array()==(dims.array()-1)).any();
    distances_[i]=free[i]&&!boundary?1e12:0;
  }
  for(int axis=0;axis<3;++axis) {
    int a=(axis+1)%3,b=(axis+2)%3,n=dims[axis];
    std::vector<double> f(n),z(n+1),out(n);std::vector<int> v(n);
    for(int i=0;i<dims[a];++i) for(int j=0;j<dims[b];++j) {
      Eigen::Vector3i k;k[a]=i;k[b]=j;
      for(int q=0;q<n;++q){k[axis]=q;f[q]=distances_[address(k)];}
      int h=0;v[0]=0;z[0]=-INFINITY;z[1]=INFINITY;
      for(int q=1;q<n;++q) {
        double cut;
        do {cut=((f[q]+q*q)-(f[v[h]]+v[h]*v[h]))/(2.0*(q-v[h]));if(cut<=z[h])--h;else break;} while(h>=0);
        ++h;v[h]=q;z[h]=cut;z[h+1]=INFINITY;
      }
      h=0;for(int q=0;q<n;++q){while(z[h+1]<q)++h;out[q]=(q-v[h])*(q-v[h])+f[v[h]];}
      for(int q=0;q<n;++q){k[axis]=q;distances_[address(k)]=out[q];}
    }
  }
  for(auto& d:distances_) d=std::sqrt(d)*resolution;
}
double Grid::distance(const Eigen::Vector3d& p) const {
  int i=address(index(p));if(i<0)return -resolution;
  // Distance to blocked voxel volumes, not only point centers; subtract
  // within-cell query displacement to preserve a conservative lower bound.
  return distances_[i]-(p-center(i)).norm()-std::sqrt(3.0)*resolution*.5;
}
bool Grid::segment(const Eigen::Vector3d& a,const Eigen::Vector3d& b,double clearance) const {
  if(!a.allFinite()||!b.allFinite()||!std::isfinite(clearance)||clearance<0)return false;
  double length=(b-a).norm();int count=std::max(1,int(std::ceil(length/(resolution*.25))));
  if(count>20000)return false;
  for(int i=0;i<=count;++i) if(distance(a+(b-a)*(double(i)/count))<clearance+length/(2*count))return false;
  return true;
}
std::vector<Eigen::Vector3d> Grid::route(const Eigen::Vector3d& start,const Eigen::Vector3d& goal,double clearance,double timeout,std::string& error) const {
  using Clock=std::chrono::steady_clock;auto begin=Clock::now();
  auto expired=[&]{return std::chrono::duration<double>(Clock::now()-begin).count()>timeout;};
  if(!inside(start)||!inside(goal)||distance(start)<clearance||distance(goal)<clearance){error="start or goal occupied, unknown or outside workspace";return {};}
  if(segment(start,goal,clearance))return {start,goal};
  int first=address(index(start)),last=address(index(goal));
  if(!segment(start,center(first),clearance)||!segment(center(last),goal,clearance)){error="no safe endpoint grid connection";return {};}
  std::vector<double> cost(free.size(),INFINITY);std::vector<int> parent(free.size(),-1);
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  cost[first]=0;open.push({(center(first)-center(last)).norm(),first});
  size_t expanded=0;
  while(!open.empty()) {
    if(expired()||++expanded>200000){error="global search resource deadline";return {};}
    auto entry=open.top();open.pop();int current=entry.second;
    if(entry.first>cost[current]+(center(current)-center(last)).norm()+1e-8)continue;
    if(current==last) {
      std::vector<Eigen::Vector3d> reverse{goal};
      for(int k=last;k!=-1;k=parent[k])reverse.push_back(center(k));reverse.push_back(start);
      std::reverse(reverse.begin(),reverse.end());
      std::vector<Eigen::Vector3d> result{start};size_t at=0;
      while(at+1<reverse.size()) {
        size_t next=at+1;
        for(size_t j=at+2;j<reverse.size()&&!expired();++j) if(segment(reverse[at],reverse[j],clearance))next=j;
        result.push_back(reverse[next]);at=next;
      }
      return result;
    }
    auto idx=index(current);auto position=center(current);
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z) {
      if(x==0&&y==0&&z==0)continue;int neighbor=address(idx+Eigen::Vector3i(x,y,z));if(neighbor<0)continue;
      auto end=center(neighbor);double candidate=cost[current]+(end-position).norm();
      if(candidate>=cost[neighbor]||!segment(position,end,clearance))continue;
      parent[neighbor]=current;cost[neighbor]=candidate;open.push({candidate+(end-center(last)).norm(),neighbor});
    }
  }
  error="no global route through observed free space";return {};
}
}
