#include <ducted_mapping/observed_grid.hpp>
using namespace ducted_mapping;
int main() {
  ObservedGrid grid(.2,2000);
  for(int i=0;i<8;++i)grid.scan({{2.01,.01,.01}},{.01,.01,.01});
  if(grid.cells().at(grid.key({1.01,.01,.01}))>-.619) return 1;
  if(grid.cells().at(grid.key({2.01,.01,.01}))<0) return 2;
  if(grid.cells().count(grid.key({3.01,.01,.01})))return 3;
  for(int i=0;i<32;++i)grid.scan({{4.01,.01,.01}},{.01,.01,.01});
  if(grid.cells().at(grid.key({2.01,.01,.01}))>-.619)return 4;
  try{ObservedGrid limited(.2,1);limited.scan({{2,.01,.01}},{.01,.01,.01});return 5;}catch(const std::runtime_error&){}
  return 0;
}
