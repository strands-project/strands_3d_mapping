#ifndef OBJECT_RETRIEVAL_H
#define OBJECT_RETRIEVAL_H

#include <vocabulary_tree/vocabulary_tree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class object_retrieval
{
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    using HistT = pcl::Histogram<131>;
    using HistCloudT = pcl::PointCloud<HistT>;
    using NormalT = pcl::Normal;
    using NormalCloudT = pcl::PointCloud<NormalT>;
    using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;

    std::string segment_path;
    vocabulary_tree<HistT, 8> vt;

    void visualize_cloud(CloudT::Ptr& cloud);
    void extract_features(std::vector<int>& inds, HistCloudT::Ptr& features, std::vector<CloudT::Ptr>& segments,
                          std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments, const Eigen::Matrix3f& K);
    void get_query_cloud(HistCloudT::Ptr& query_cloud, CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K);
    size_t write_segments(std::vector<CloudT::Ptr>& segments, std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments,  const Eigen::Matrix3f& K, vector<string>& files, size_t istart);
    void read_segments(std::vector<CloudT::Ptr>& segments, std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments,  Eigen::Matrix3f& K, size_t max_segments);
    bool read_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K, string& metadata, size_t segment_id);
    void write_vocabulary(vocabulary_tree<HistT, 8>& vt);
    void read_vocabulary(vocabulary_tree<HistT, 8>& vt);
    float calculate_similarity(CloudT::Ptr& cloud1, const Eigen::Matrix3f& K1,
                               CloudT::Ptr& cloud2, const Eigen::Matrix3f& K2);
    void compute_segments(std::vector<CloudT::Ptr>& sweeps, std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files);
    void process_segments();
    void query_vocabulary(vector<index_score>& scores, size_t query_ind, size_t nbr_query);

    void save_features(HistCloudT::Ptr& features, std::vector<int>& indices);
    bool load_features(HistCloudT::Ptr& features, std::vector<int>& indices);

    object_retrieval(const std::string& segment_path);
};

#endif // OBJECT_RETRIEVAL_H
