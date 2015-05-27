#ifndef RETRIEVAL_CLIENT_H
#define RETRIEVAL_CLIENT_H

#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/descriptor_config.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<128>,
                                   (float[128], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<250>,
                                   (float[250], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<1344>,
                                   (float[1344], histogram, histogram)
)

namespace retrieval_client {

using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;
using PfhRgbT = pcl::Histogram<250>;
using PfhRgbCloudT = pcl::PointCloud<PfhRgbT>;
using ShotT = pcl::Histogram<1344>;
using ShotCloudT = pcl::PointCloud<ShotT>;
using HistT = object_retrieval::HistT;
using HistCloudT = pcl::PointCloud<HistT>;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using index_score = object_retrieval::index_score;

void compute_sift_features_detail(cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints, CloudT::Ptr& cloud, cv::Mat& image,
                                  cv::Mat& depth, int minx, int miny, const Eigen::Matrix3f& K);
void compute_sift_features_for_query(SiftCloudT::Ptr& desc_cloud, CloudT::Ptr& kp_cloud, CloudT::Ptr& cloud, Eigen::Matrix3f& K);
void save_sift_features(object_retrieval& obr_scans);
int scan_ind_for_segment(int i, object_retrieval& obr_segments);
void save_split_features(object_retrieval& obr_segments);
void save_pfhrgb_features_for_supervoxels(object_retrieval& obr_segments);
void save_shot_features_for_supervoxels(object_retrieval& obr_segments);
void compute_and_save_segments(object_retrieval& obr_scans);
void get_voxel_vectors_for_scan(CloudT::Ptr& voxel_centers, std::vector<double>& vocabulary_norms, std::vector<std::map<int, double> >& vocabulary_vectors,
                                std::vector<std::map<int, int> >& vocabulary_index_vectors, int i, object_retrieval& obr_scans);
void compute_voxel_centers(CloudT::Ptr& center_cloud, std::vector<CloudT::Ptr>& voxel_points, int min_features);
void get_voxels_and_points_for_scan(std::vector<HistCloudT::Ptr>& voxels, std::vector<CloudT::Ptr>& voxel_points, int i, object_retrieval& obr_scans);
void save_oversegmented_grouped_vocabulary_index_vectors(object_retrieval& obr_scans, object_retrieval& obr_segments);
void change_supervoxel_groups(object_retrieval& obr_voxels);
void read_supervoxel_groups(object_retrieval& obr_voxels);
void reweight_query_vocabulary(std::vector<tuple<int, int, double> >& reweighted_scores, object_retrieval& obr_segments,
                               object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, const Eigen::Matrix3f& K,
                               std::vector<tuple<int, int, double> >& tuple_scores, int noise_scans_size, HistCloudT::Ptr& query_features,
                               CloudT::Ptr& query_cloud, int nbr_initial_query);
void load_nth_keypoints_features_for_scan(CloudT::Ptr& keypoints, HistCloudT::Ptr& features,
                                          int i, std::vector<int>& indices, object_retrieval& obr_scans);
void compute_grown_segment_score(std::vector<double>& match_scores, HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, CloudT::Ptr& query_cloud,
                                 std::vector<index_score>& updated_scores, std::vector<std::vector<int> >& oversegment_indices,
                                 object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int noise_scans_size);
void compute_grow_subsegment_scores(std::vector<index_score>& updated_scores, std::vector<std::vector<int> >& oversegment_indices, std::vector<index_score>& scores,
                                    std::vector<int>& hints, HistCloudT::Ptr& features, std::map<vocabulary_tree<HistT, 8>::node*, int>& mapping, object_retrieval& obr_segments,
                                    object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int nbr_query, int noise_scans_size);
void find_top_oversegments_grow_and_score(std::vector<index_score>& first_scores, std::vector<index_score>& second_scores, std::vector<int>& hints, std::vector<std::vector<int> >& oversegment_indices,
                                          HistCloudT::Ptr& features, std::map<vocabulary_tree<HistT, 8>::node*, int>& mapping, object_retrieval& obr_segments,
                                          object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int nbr_query, int nbr_initial_query, int noise_scans_size);
void get_sift_features_for_segment(SiftCloudT::Ptr& sift_cloud, CloudT::Ptr& sift_keypoints, CloudT::Ptr& keypoints, const string& scan_folder);
void reweight_query_vocabulary_sift(std::vector<index_score>& second_scores, std::vector<index_score>& first_scores, CloudT::Ptr& query_cloud,
                                    HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, int query_id, int nbr_query,
                                    object_retrieval& obr_segments, object_retrieval& obr_segments_annotations, int noise_segments_size);
void reweight_query_vocabulary_sift(std::vector<index_score>& reweight_grown_scores, std::vector<index_score>& first_grown_scores,
                                    std::vector<index_score>& first_scores, std::vector<int>& hints, std::vector<std::vector<int> >& oversegment_indices, CloudT::Ptr& query_cloud,
                                    HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, int query_id, int nbr_query, int nbr_initial_query,
                                    object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, object_retrieval& obr_segments,
                                    object_retrieval& obr_segments_annotations, int noise_scans_size, std::map<vocabulary_tree<HistT, 8>::node*, int>& mapping,
                                    SiftCloudT* optional_sift_query_features = NULL, CloudT* optional_sift_query_keypoints = NULL);


} // namespace retrieval_client

#endif // RETRIEVAL_CLIENT_H
