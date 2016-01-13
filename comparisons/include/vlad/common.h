#ifndef VLAD_COMMON_H
#define VLAD_COMMON_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

const static int nbr_centers = 64;

template <typename PointT>
bool pcl_hist_inf(const PointT& v)
{
    return std::find_if(std::begin(v.histogram), std::end(v.histogram), [](float f) {
        return std::isnan(f) || std::isinf(f);
    }) != std::end(v.histogram);
}

std::pair<std::vector<CloudT::Ptr>, std::vector<boost::filesystem::path> >
    load_vlad_clouds(const std::vector<std::pair<float, std::string> >& matches);

#endif // VLAD_COMMON_H
