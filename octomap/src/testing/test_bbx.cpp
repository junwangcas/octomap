
#include <octomap/octomap.h>
#include "testing.h"

using namespace octomap;

int main(int /*argc*/, char** /*argv*/) {
  // 设置八叉树的分辨率为0.2米
  const float resolution = 0.2f;
  // 构建一个八叉树
  OcTree tree(resolution);

  // 设置一个外包矩形
  // Set up the bounding box.
  const float bbx_limit = 10.1f;
  point3d bbx_min(-bbx_limit, -bbx_limit, -bbx_limit);
  point3d bbx_max(bbx_limit, bbx_limit, bbx_limit);
  tree.setBBXMin(bbx_min);
  tree.setBBXMax(bbx_max);
  // 也就是说这个范围以内的才有效，这个范围以外的无效
  tree.useBBXLimit(true);

  //
  // Set up the point cloud. Offset the origin by half resolution to use the
  // center of a voxel.
  Pointcloud cloud;
  const point3d origin(resolution / 2.f, 5.f + resolution / 2.f, resolution / 2.f);
  // max range说的是局部坐标系下的东西
  const float maxrange = 8.f;
  // The first point is inside the bounding box and max range.
  const point3d point0 = origin + point3d(5.f, 0.f, 0.f);
  cloud.push_back(point0);
  // The second point is outside the bounding box but within max range.
  const point3d point1 = origin + point3d(0.f, 7.f, 0.f);
  cloud.push_back(point1);
  // The third point is inside the bounding box but outside max range.
  const point3d point2 = origin + point3d(0.f, 0.f, 9.f);
  cloud.push_back(point2);
  tree.insertPointCloud(cloud, origin, maxrange);

  // Check the point cloud insertion using ray casting.
  tree.setOccupancyThres(0.5f);
  point3d end_point;

  // 这个origin就是位置、方向就是pose
  // 一个原点 + 方向 + 终点
  // Searching in the x-direction from the origin finds the first point.
  bool ray_cast_ret = tree.castRay(origin, point3d(1.f, 0.f, 0.f), end_point);
  EXPECT_TRUE(ray_cast_ret);
  const float eps = 1e-3;
  EXPECT_NEAR(end_point.x(), point0.x(), eps);
  EXPECT_NEAR(end_point.y(), point0.y(), eps);
  EXPECT_NEAR(end_point.z(), point0.z(), eps);

  // Searching in the y-direction from the origin terminates just outside the
  // bounding box.
  ray_cast_ret = tree.castRay(origin, point3d(0.f, 1.f, 0.f), end_point);
  EXPECT_FALSE(ray_cast_ret);
  EXPECT_NEAR(end_point.x(), point1.x(), eps);
  EXPECT_NEAR(end_point.y(), bbx_limit + resolution, eps);
  EXPECT_NEAR(end_point.z(), point1.z(), eps);

  // Searching in the z-direction from the origin terminates at the max range.
  ray_cast_ret = tree.castRay(origin, point3d(0.f, 0.f, 1.f), end_point);
  EXPECT_FALSE(ray_cast_ret);
  EXPECT_NEAR(end_point.x(), point2.x(), eps);
  EXPECT_NEAR(end_point.y(), point2.y(), eps);
  EXPECT_NEAR(end_point.z(), origin.z() + maxrange, eps);
  return 0;
}
