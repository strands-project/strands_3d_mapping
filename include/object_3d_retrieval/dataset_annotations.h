#ifndef DATASET_ANNOTATIONS_H
#define DATASET_ANNOTATIONS_H

#include "object_3d_retrieval/object_retrieval.h"
#include <pcl/io/pcd_io.h>

namespace dataset_annotations {

using index_score = object_retrieval::index_score;
using PointT = object_retrieval::PointT;
using CloudT = object_retrieval::CloudT;
using HistT = object_retrieval::HistT;
using HistCloudT = object_retrieval::HistCloudT;

// OK
struct voxel_annotation {
    int segment_id;
    std::string segment_file;
    std::string scan_folder;
    int scan_id;
    std::string annotation;
    bool full;
    bool segment_covered;
    bool annotation_covered;
    template <typename Archive>
    void serialize(Archive& archive)
    {
        archive(segment_id, segment_file, scan_folder, scan_id, annotation, full, segment_covered, annotation_covered);
    }
};

std::string annotation_for_scan(int i, object_retrieval& obr);
std::pair<bool, bool> supervoxel_is_correct(CloudT::Ptr& cloud, const Eigen::Matrix3f& K, int minx, int maxx, int miny, int maxy);
voxel_annotation scan_for_supervoxel(int i, const Eigen::Matrix3f& K, object_retrieval& obr);
std::string annotation_for_supervoxel(int i, object_retrieval& obr);
void list_annotated_supervoxels(std::vector<voxel_annotation>& annotations, const string& annotations_file,
                                const Eigen::Matrix3f& K, object_retrieval& obr);
void calculate_correct_ratio(std::map<std::string, pair<float, int> >& instance_correct_ratios, voxel_annotation& a,
                             int scan_ind, std::vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose = true);
void calculate_correct_ratio(std::map<std::string, pair<float, int> >& instance_correct_ratios, const string& annotation,
                             int scan_ind, std::vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose = true);
void calculate_correct_ratio_exclude_sweep(std::map<std::string, pair<float, int> >& instance_correct_ratios, const std::string& annotation, int scan_ind,
                                           std::vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose = true);
void calculate_correct_ratio_exclude_sweep_precise(std::map<std::string, pair<float, int> >& instance_correct_ratios, const std::string& annotation, int scan_ind,
                                                   std::vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose = true);
void compute_decay_correct_ratios(std::vector<std::pair<float, int> >& decay_correct_ratios, std::vector<int>& intermediate_points,
                                  voxel_annotation& a, int scan_ind, std::vector<index_score>& scores, object_retrieval& obr_scans,
                                  int nbr_query, int noise_scans_size);


} // namespace dataset_annotations

#endif // DATASET_ANNOTATIONS_H
