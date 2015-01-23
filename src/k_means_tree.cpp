#include "k_means_tree/k_means_tree.h"

//#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <Eigen/Dense>

/*POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<33>,
        (float, histogram, histogram)
)*/

//template class k_means_tree<pcl::PointXYZ, 8>;
template class k_means_tree<pcl::PointXYZRGB, 8>;
template class k_means_tree<pcl::Histogram<100>, 8>;
template class k_means_tree<pcl::Histogram<33>, 8>;
template class k_means_tree<pcl::Histogram<128>, 8>;

template <>
typename map_proxy<pcl::PointXYZ>::map_type eig(pcl::PointXYZ& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZ>::const_map_type eig(const pcl::PointXYZ& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZRGB>::map_type eig(pcl::PointXYZRGB& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZRGB>::const_map_type eig(const pcl::PointXYZRGB& v)
{
    return v.getVector3fMap();
}
