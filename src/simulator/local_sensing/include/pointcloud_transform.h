#ifndef LOCAL_SENSING_POINTCLOUD_TRANSFORM_H
#define LOCAL_SENSING_POINTCLOUD_TRANSFORM_H

// ============================================================================
// 文件：include/pointcloud_transform.h
//
// 空点云安全的坐标变换：pcl::transformPointCloud 在输入 width=0 时会做除法，
// 触发 SIGFPE。这里集中处理空输入，并用单测锁住这个行为。
// ============================================================================
#include <cstdint>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

namespace local_sensing
{
template <typename PointT, typename TransformT>
bool transformPointCloudSafe(const pcl::PointCloud<PointT> &input,
                             const TransformT &transform,
                             pcl::PointCloud<PointT> &output)
{
  output.clear();
  if (input.points.empty())
  {
    output.width = 0;
    output.height = 1;
    output.is_dense = true;
    return false;
  }

  pcl::transformPointCloud(input, output, transform);
  output.width = static_cast<std::uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return true;
}
}  // namespace local_sensing

#endif  // LOCAL_SENSING_POINTCLOUD_TRANSFORM_H
