#ifndef VLAD_REPRESENTATION_H
#define VLAD_REPRESENTATION_H

extern "C" {
#include <vlad.h>
#include <kmeans.h>
#include <generic.h>
#include <host.h>
#include <mathop.h>
}

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

const static int V = 250*nbr_centers;

using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using VladT = pcl::Histogram<V>;
using VladCloudT = pcl::PointCloud<VladT>;

namespace pcl {
template <>
class DefaultPointRepresentation<VladT> : public PointRepresentation<VladT> {
public:
    DefaultPointRepresentation ()
    {
        nr_dimensions_ = V;
    }

    virtual void
    copyToFloatArray (const VladT &p, float * out) const
    {
        for (int i = 0; i < nr_dimensions_; ++i)
            out[i] = p.histogram[i];
    }
};
}

namespace vlad_representation {

struct vlad_repr {
    vl_size dimension;
    vl_size numCenters;
    VlKMeans* kmeans;
};

void build_vlad_representation(const boost::filesystem::path& data_path,
                               vl_size numData, vlad_repr& repr);
void encode_vlad_representation(const boost::filesystem::path& data_path,
                                vlad_repr& repr);
VladT encode_vlad_point(HistCloudT::Ptr& features);
std::vector<std::pair<float, std::string> > query_vlad_representation(VladCloudT::Ptr& vcloud, pcl::KdTreeFLANN<VladT>& kdtree,
                                                                      const dynamic_object_retrieval::data_summary& summary,
                                                                      HistCloudT::Ptr& fcloud);
void vlad_l2_normalization(VladT& v);
void vlad_intranormalization(VladT& v);
void vlad_sqrt_normalization(VladT& v);

} // namespace vlad_representation

#endif // VLAD_REPRESENTATION_H
