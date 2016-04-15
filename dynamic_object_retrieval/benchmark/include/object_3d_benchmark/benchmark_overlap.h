#ifndef BENCHMARK_OVERLAP_H
#define BENCHMARK_OVERLAP_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

namespace benchmark_retrieval {

double compute_overlap(CloudT::Ptr& A, CloudT::Ptr& B, float resolution = 0.08);

} // namespace benchmark_retrieval

#endif // BENCHMARK_OVERLAP_H
