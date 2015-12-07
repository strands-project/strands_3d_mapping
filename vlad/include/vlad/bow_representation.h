#ifndef BOW_REPRESENTATION_H
#define BOW_REPRESENTATION_H

#include <iostream>
#include <cstring>
#include <Eigen/Dense>
#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <vlad/common.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using BowT = pcl::Histogram<nbr_centers>;
using BowCloudT = pcl::PointCloud<BowT>;

namespace pcl {
template <>
class DefaultPointRepresentation<BowT> : public PointRepresentation<BowT> {
public:
    DefaultPointRepresentation ()
    {
        nr_dimensions_ = nbr_centers;
    }

    virtual void
    copyToFloatArray (const BowT &p, float * out) const
    {
        for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = p.histogram[i];
    }
};
}

namespace bow_representation {

BowT encode_bow_point(HistCloudT::Ptr& features);
void encode_bow_representation(const boost::filesystem::path& data_path);
std::vector<std::pair<float, std::string> > query_bow_representation(const dynamic_object_retrieval::data_summary& summary,
                                                                     HistCloudT::Ptr& fcloud);

} // namespace bow_representation

#endif // BOW_REPRESENTATION_H
