#include <gtest/gtest.h>
#include <plan_env/grid_map.h>

using V = Eigen::Vector3d;

TEST(LocalOccupancy, QuantizationDoesNotAddAnIsotropicVoxelDiagonal) {
  GridMap grid;
  // Same clearance as the second flight's resume. This side point is outside
  // the protected body but the previous isotropic stencil occupied the start.
  grid.reset(V::Zero(), 4., -2., 2., {V(0., -.79, .20)}, .6864952);
  EXPECT_EQ(0, grid.getInflateOccupancy(V::Zero()));
  EXPECT_TRUE(grid.segment(V::Zero(), V(.12, 0., 0.)));
}

TEST(LocalOccupancy, ProtectsBodyAcrossSourceAndQueryVoxelCorners) {
  const double clearance = .6864952;
  for (const V shift : {V(0., 0., 0.), V(.049, -.081, .037), V(-.099, .001, -.053)}) {
    const V point = shift + V(.013, -.027, .089);
    GridMap grid;
    grid.reset(shift, 4., -2., 2., {point}, clearance);
    for (int x=-4; x<=4; ++x) for (int y=-4; y<=4; ++y) for (int z=-4; z<=4; ++z) {
      V direction(x, y, z);
      if (direction.norm() == 0.) continue;
      const V query = point + direction.normalized()*(clearance-1e-7);
      EXPECT_EQ(1, grid.getInflateOccupancy(query));
    }
  }
}

TEST(LocalOccupancy, FullChordStillRejectsBetweenEndpointObstacles) {
  GridMap grid;
  grid.reset(V::Zero(), 4., -2., 2., {V(.8, 0., 0.)}, .25);
  EXPECT_EQ(0, grid.getInflateOccupancy(V::Zero()));
  EXPECT_EQ(0, grid.getInflateOccupancy(V(1.6, 0., 0.)));
  EXPECT_FALSE(grid.segment(V::Zero(), V(1.6, 0., 0.)));
  EXPECT_EQ(1, grid.getInflateOccupancy(V(0., 0., 2.01)));
  EXPECT_EQ(1, grid.getInflateOccupancy(V(3.8, 0., 0.)));
}

TEST(LocalOccupancy, RecordedSideBelowEndpointRemainsOutsideProtectedBody) {
  // Ground capture after the 17:38 flight: the 0.781 m away point was
  // falsely occupied by two voxel boxes at clearance 0.6883 m.
  const V start(.1399342105, .0738472049, .9908617278);
  const V goal(-.8419387217, .2633875567, .9908617278);
  const V point(-1.1174615362, -.2041891592, .4295462361);
  GridMap grid;
  grid.reset(start, 4., .39, 2.49, {point}, .6883046816);
  EXPECT_EQ(0, grid.getInflateOccupancy(goal));
  EXPECT_TRUE(grid.segment(start, goal));
  EXPECT_FALSE(grid.sourceMayOccupy(point, goal));
}

TEST(LocalOccupancy, ExactRefinementRetainsEveryPointInASharedVoxel) {
  GridMap grid;
  const V far(.599, .099, .099), near(.501, .001, .001);
  grid.reset(V::Zero(), 4., -2., 2., {far, near}, .5);
  EXPECT_EQ(1, grid.getInflateOccupancy(V(.002, .001, .001)));
  EXPECT_TRUE(grid.sourceMayOccupy(near, V(.002, .001, .001)));
  EXPECT_EQ(0, grid.getInflateOccupancy(V(-.099, -.099, -.099)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
