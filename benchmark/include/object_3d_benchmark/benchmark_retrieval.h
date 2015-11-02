#ifndef BENCHMARK_RETRIEVAL
#define BENCHMARK_RETRIEVAL

#include <metaroom_xml_parser/load_utilities.h>

namespace benchmark_retrieval {

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

double get_match_accuracy(CloudT::Ptr& object, CloudT::Ptr& cluster);
std::vector<std::pair<CloudT::Ptr, std::string> > find_labels(std::vector<CloudT::Ptr>& input_segmented_dynamics, const std::vector<boost::filesystem::path>& sweep_paths);
                                                              //semantic_map_load_utilties::LabelledData<PointT>& labelled_clusters);
std::pair<Eigen::Matrix3f, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > get_camera_matrix_and_transforms(const std::string& sweep_xml);
Eigen::Matrix4f get_global_camera_rotation(semantic_map_load_utilties::LabelledData<PointT>& labels);

template <typename IndexT>
std::pair<std::vector<CloudT::Ptr>, std::vector<boost::filesystem::path> > load_retrieved_clouds(std::vector<std::pair<boost::filesystem::path, IndexT> >& retrieved_paths)
{
    using path_index_type = std::pair<boost::filesystem::path, IndexT>;

    std::vector<CloudT::Ptr> clouds;
    std::vector<boost::filesystem::path> sweep_paths;
    clouds.reserve(retrieved_paths.size());
    sweep_paths.reserve(retrieved_paths.size());
    for (path_index_type s : retrieved_paths) {
        IndexT index;
        boost::filesystem::path path;
        tie(path, index) = s;
        std::cout << "Path: " << path.string() << std::endl;
        clouds.push_back(CloudT::Ptr(new CloudT));
        pcl::io::loadPCDFile(path.string(), *clouds.back());
        sweep_paths.push_back(path.parent_path().parent_path() / "room.xml");
    }

    return make_pair(clouds, sweep_paths);
}

template <typename IndexT>
std::pair<std::vector<CloudT::Ptr>, std::vector<boost::filesystem::path> > load_retrieved_clouds(std::vector<std::pair<std::vector<boost::filesystem::path>, IndexT> >& retrieved_paths)
{
    using path_index_type = std::pair<std::vector<boost::filesystem::path>, IndexT>;

    std::vector<CloudT::Ptr> clouds;
    std::vector<boost::filesystem::path> sweep_paths;
    clouds.reserve(retrieved_paths.size());
    sweep_paths.reserve(retrieved_paths.size());
    for (path_index_type s : retrieved_paths) {
        IndexT index;
        std::vector<boost::filesystem::path> paths;
        tie(paths, index) = s;
        clouds.push_back(CloudT::Ptr(new CloudT));
        std::cout << "Paths: " << std::endl;
        for (const boost::filesystem::path& path : paths) {
            std::cout << path.string() << std::endl;
            CloudT::Ptr temp(new CloudT);
            pcl::io::loadPCDFile(path.string(), *temp);
            *clouds.back() += *temp;
        }
        sweep_paths.push_back(paths[0].parent_path().parent_path() / "room.xml");
    }

    return make_pair(clouds, sweep_paths);
}

} // namespace benchmark_retrieval

#endif // BENCHMARK_RETRIEVAL
