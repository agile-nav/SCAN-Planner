#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pointcloud_transform.h"

TEST(PointCloudTransform, EmptyInputDoesNotCrash)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointCloud<pcl::PointXYZI> output;
  output.push_back(pcl::PointXYZI{});  // 确认输出会被清空

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  EXPECT_FALSE(local_sensing::transformPointCloudSafe(input, transform, output));
  EXPECT_TRUE(output.points.empty());
  EXPECT_EQ(output.width, 0u);
  EXPECT_EQ(output.height, 1u);
}

TEST(PointCloudTransform, TransformsNonEmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointXYZI a;
  a.x = 1.0f;
  a.y = 2.0f;
  a.z = 3.0f;
  input.push_back(a);
  pcl::PointXYZI b;
  b.x = -1.0f;
  b.y = 0.0f;
  b.z = 0.5f;
  input.push_back(b);

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform(0, 3) = 10.0;
  transform(1, 3) = -2.0;

  pcl::PointCloud<pcl::PointXYZI> output;
  EXPECT_TRUE(local_sensing::transformPointCloudSafe(input, transform, output));
  ASSERT_EQ(output.points.size(), 2u);
  EXPECT_NEAR(output.points[0].x, 11.0, 1e-5);
  EXPECT_NEAR(output.points[0].y, 0.0, 1e-5);
  EXPECT_NEAR(output.points[1].x, 9.0, 1e-5);
  EXPECT_NEAR(output.points[1].y, -2.0, 1e-5);
  EXPECT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
}

TEST(PointCloudTransform, IdentityKeepsCoordinates)
{
  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::PointXYZI point;
  point.x = 0.25f;
  point.y = -0.5f;
  point.z = 1.0f;
  input.push_back(point);

  pcl::PointCloud<pcl::PointXYZI> output;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  EXPECT_TRUE(local_sensing::transformPointCloudSafe(input, transform, output));
  ASSERT_EQ(output.points.size(), 1u);
  EXPECT_FLOAT_EQ(output.points[0].x, point.x);
  EXPECT_FLOAT_EQ(output.points[0].y, point.y);
  EXPECT_FLOAT_EQ(output.points[0].z, point.z);
}
