#include <ducted_planning/grid.hpp>
#include <iostream>
using namespace ducted_planning;
int main() {
  Grid g({-3,-3,-1},{6,6,4},.2);std::string error;
  g.computeDistances();if(!g.route({-1,0,1},{1,0,1},.2,1,error).empty())return 1;
  std::fill(g.free.begin(),g.free.end(),1);
  // Wall blocks a direct route; the planner must go around its finite end.
  for(size_t i=0;i<g.free.size();++i){auto p=g.center(i);if(std::abs(p.x())<.15&&p.y()<1&&p.y()>-2.9)g.free[i]=0;}
  g.computeDistances();auto route=g.route({-1,0,1},{1,0,1},.2,2,error);
  if(route.size()<3){std::cerr<<error;return 2;}
  for(size_t i=1;i<route.size();++i)if(!g.segment(route[i-1],route[i],.2))return 3;
  // A full separating plane is impassable, including diagonal cell edges.
  for(size_t i=0;i<g.free.size();++i)if(std::abs(g.center(i).x())<.15)g.free[i]=0;
  g.computeDistances();if(!g.route({-1,0,1},{1,0,1},.2,2,error).empty())return 4;
  if(!g.route({NAN,0,1},{1,0,1},.2,2,error).empty())return 5;
  std::cout<<"observed-space global route checks passed\n";return 0;
}
