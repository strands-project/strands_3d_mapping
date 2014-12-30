#ifndef CONVEX_VOXEL_SEGMENTATION_H
#define CONVEX_VOXEL_SEGMENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
//#include <pcl/segmentation/impl/supervoxel_clustering.hpp>

class convex_voxel_segmentation
{
private:
    using edge_pair = std::pair<uint32_t,uint32_t>;
    using PointT = pcl::PointXYZRGB;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointNT = pcl::PointNormal ;
    using PointNCloudT = pcl::PointCloud<PointNT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using SuperVoxelT = pcl::Supervoxel<PointT>;

    bool display_segmentation;
    float voxel_resolution;

    //PointCloudT::Ptr getColoredVoxelCloud (pcl::SupervoxelClustering<PointT> &s) const;

    void connected_components(std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& edges) const;
    float graph_cut(std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& edges, const std::map<edge_pair, float>& weights) const;
    void addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                           pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                           std::string supervoxel_name,
                                           boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
                                           std::vector<bool>& local_pairs) const;
    void visualize_segmentation(std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                                std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                                PointCloudT::Ptr& voxel_centroid_cloud,
                                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                                PointNCloudT::Ptr sv_normal_cloud,
                                std::set<edge_pair>& remove_pairs) const;
    void visualize(const std::vector<std::set<size_t> >& groups, pcl::PointCloud<pcl::PointXYZL>::Ptr &labels,
                   std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                   std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                   PointCloudT::Ptr& voxel_centroid_cloud,
                   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                   PointNCloudT::Ptr sv_normal_cloud,
                   std::set<edge_pair>& remove_pairs) const;
    void segment_pointcloud(PointCloudT::Ptr& cloud,
                            std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                            std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                            PointCloudT::Ptr& voxel_centroid_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                            PointNCloudT::Ptr& sv_normal_cloud,
                            pcl::PointCloud<pcl::PointXYZL>::Ptr& labels) const;
    std::pair<bool, float> concave_relationship(pcl::Supervoxel<PointT>::Ptr& first_supervoxel,
                                                pcl::Supervoxel<PointT>::Ptr& second_supervoxel) const;
    void construct_new_edge_set(std::set<edge_pair>& new_edges, const std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& old_edges) const;
    void global_segmentation(std::vector<std::set<size_t> >& groups,
                             const std::set<edge_pair>& edges,
                             const std::map<edge_pair, float>& weights,
                             const std::map<size_t, size_t>& vertex_points) const;
    void local_convexity_segmentation(std::vector<PointCloudT::Ptr>& result, std::vector<NormalCloudT::Ptr> &segment_normals, PointCloudT::Ptr& cloud) const;
public:
    void segment_objects(std::vector<PointCloudT::Ptr>& result, std::vector<NormalCloudT::Ptr> &segment_normals, std::vector<PointCloudT::Ptr>& full_result, PointCloudT::Ptr& original) const;
    convex_voxel_segmentation(bool display_segmentation = false);
};

#endif // CONVEX_VOXEL_SEGMENTATION_H
