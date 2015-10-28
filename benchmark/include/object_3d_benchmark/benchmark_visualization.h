#ifndef BENCHMARK_VISUALIZATION_H
#define BENCHMARK_VISUALIZATION_H

#include <dynamic_object_retrieval/summary_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

namespace benchmark_retrieval {

cv::Mat make_image(std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> >& results);

template <typename IndexT, typename PathT>
std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> > get_cloud_and_path(
        std::vector<std::pair<PathT, IndexT> >& retrieved_paths)
{
    using path_index_type = std::pair<PathT, IndexT>;

    std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> > clouds;

    for (path_index_type s : retrieved_paths) {
        IndexT index;
        boost::filesystem::path path;
        std::tie(path, index) = s;
        std::cout << "Path: " << path.string() << " with score " << s.second.score << std::endl;
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(path.string(), *cloud);
        clouds.push_back(std::make_pair(cloud, path));
    }

    return clouds;
}

template <typename IndexT>
std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> > get_cloud_and_path(
        std::vector<std::pair<std::vector<boost::filesystem::path>, IndexT> >& retrieved_paths)
{
    using path_index_type = std::pair<std::vector<boost::filesystem::path>, IndexT>;

    std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> > clouds;

    for (path_index_type s : retrieved_paths) {
        IndexT index;
        std::vector<boost::filesystem::path> paths;
        std::tie(paths, index) = s;
        CloudT::Ptr cloud(new CloudT);
        std::cout << "Paths: " << std::endl;
        for (boost::filesystem::path& path : paths) {
            std::cout << path.string() << std::endl;
            CloudT::Ptr temp(new CloudT);
            pcl::io::loadPCDFile(path.string(), *temp);
            *cloud += *temp;
        }
        std::cout << " with score " << s.second.score << std::endl;
        clouds.push_back(std::make_pair(cloud, paths[0]));
    }

    return clouds;
}

template <typename IndexT, typename PathT>
cv::Mat make_visualization_image(std::vector<std::pair<PathT, IndexT> >& retrieved_paths, const Eigen::Matrix4f& T)
{
    std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> > clouds = get_cloud_and_path(retrieved_paths);
    for (auto& c : clouds) {
        for (PointT& p : c.first->points) {
            p.getVector4fMap() = T*p.getVector4fMap();
        }
    }
    return make_image(clouds);
}

} // namespace benchmark_retrieval

#endif // BENCHMARK_VISUALIZATION_H
