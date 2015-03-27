#ifndef myutil_H_
#define myutil_H_

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

int popcount_lauradoux(uint64_t *buf, uint32_t size);
template <typename T> Eigen::Matrix<T,4,4> getMat(const T* const camera, int mode = 0);
template <typename T> void transformPoint(const T* const camera, T * point, int mode = 0);
void getMat(const double* const camera, double * mat);

#endif
