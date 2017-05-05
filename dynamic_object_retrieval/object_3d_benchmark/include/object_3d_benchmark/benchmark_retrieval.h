#ifndef BENCHMARK_RETRIEVAL
#define BENCHMARK_RETRIEVAL

#include <pcl/common/transforms.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <Stopwatch.h>
#include "object_3d_benchmark/benchmark_result.h"
#include "object_3d_benchmark/benchmark_visualization.h" // maybe move the get_score_for_sweep to another header?

namespace benchmark_retrieval {

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;
using correct_ratio = std::pair<double, double>;

double get_match_accuracy(CloudT::Ptr& object, CloudT::Ptr& cluster);
std::vector<std::pair<CloudT::Ptr, std::string> > find_labels(std::vector<CloudT::Ptr>& input_segmented_dynamics, const std::vector<boost::filesystem::path>& sweep_paths);
                                                              //semantic_map_load_utilties::LabelledData<PointT>& labelled_clusters);
std::pair<Eigen::Matrix3f, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > get_camera_matrix_and_transforms(const std::string& sweep_xml);
std::tuple<Eigen::Matrix3f,
           std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >,
           sensor_msgs::CameraInfo>
get_camera_info_and_transforms(const std::string& sweep_xml);
Eigen::Matrix4f get_global_camera_rotation(semantic_map_load_utilties::LabelledData<PointT>& labels);
cv::Mat sweep_get_depth_at(const boost::filesystem::path& sweep_xml, size_t scan_index);
cv::Mat sweep_get_rgb_at(const boost::filesystem::path& sweep_xml, size_t scan_index);
CloudT::Ptr get_cloud_from_sweep_mask(CloudT::Ptr& sweep, cv::Mat& mask, const Eigen::Matrix4f& mask_transform, const Eigen::Matrix3f& K);

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
        std::tie(path, index) = s;
        std::cout << "Path: " << path.string() << std::endl;
        std::cout << "Index: " << index.index << std::endl;
        std::cout << "Score: " << index.score << std::endl;
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
        std::cout << "Paths with size " << paths.size() << ":" << std::endl;
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

// couldn't this just be a std::function, even if it's a closure?
template<typename RetrievalFunctionT>
std::pair<benchmark_retrieval::benchmark_result, std::vector<cv::Mat> > get_score_for_sweep(RetrievalFunctionT rfunc, const std::string& sweep_xml,
                                                                                            benchmark_retrieval::benchmark_result current_result)
{
    boost::filesystem::path sweep_path = boost::filesystem::path(sweep_xml);

    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    std::tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
    CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(sweep_xml);
    SurfelCloudT::Ptr surfel_map(new SurfelCloudT);
    pcl::io::loadPCDFile((sweep_path.parent_path() / "surfel_map.pcd").string(), *surfel_map);
    Eigen::Matrix4f T = benchmark_retrieval::get_global_camera_rotation(labels);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    std::vector<cv::Mat> visualizations;
    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
        CloudT::Ptr object_cloud;
        std::string query_label;
        cv::Mat query_image;
        cv::Mat query_mask;
        size_t scan_index;
        tie(object_cloud, query_label, query_image, query_mask, scan_index) = tup;
        bool found = false;
        for (const std::string& is_check : objects_to_check) {
            if (query_label.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                break;
            }
        }
        if (!found) {
            continue;
        }
        if (query_label == "chair3") {
            query_label = "chair2"; // these two objects are identical, will be counted as the same instance
        }

        Eigen::Vector4f center;
        pcl::compute3DCentroid(*object_cloud, center);
        if (center.head<3>().norm() > 2.5f) {
            continue;
        }

        cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(sweep_xml, scan_index);
        //cv::imshow("Query object", query_image);
        //cv::waitKey();

        //CloudT::Ptr query_cloud(new CloudT);
        //pcl::transformPointCloud(*object_cloud, *query_cloud, camera_transforms[scan_index]);
        CloudT::Ptr query_cloud = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, camera_transforms[scan_index], K);

        if (query_cloud->size() < 500) {
            continue;
        }

        TICK("total_query_vocabulary");
        std::vector<CloudT::Ptr> retrieved_clouds;
        std::vector<boost::filesystem::path> sweep_paths;

        // // auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, true);
        //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth, K, 50, vocabulary_path, summary, false);
        // this is really the only thing we need, can put this in a wrapper, (e.g lambda function?)
        //tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        std::tie(retrieved_clouds, sweep_paths) = rfunc(query_cloud, query_image, query_depth, surfel_map, K);
        for (int i = 0; i < sweep_paths.size() && i < retrieved_clouds.size(); ++i) {
            if (sweep_paths[i] == sweep_path) {
                retrieved_clouds.erase(retrieved_clouds.begin() + i);
                sweep_paths.erase(sweep_paths.begin() + i);
                --i;
            }
        }
        retrieved_clouds.resize(std::min(10, int(retrieved_clouds.size())));
        sweep_paths.resize(std::min(10, int(retrieved_clouds.size())));

        TOCK("total_query_vocabulary");

        TICK("find_labels");
        std::cout << "Finding labels..." << std::endl;
        std::vector<std::pair<CloudT::Ptr, std::string> > cloud_labels = benchmark_retrieval::find_labels(retrieved_clouds, sweep_paths);
        std::cout << "Finished finding labels..." << std::endl;
        TOCK("find_labels");

        std::vector<std::string> only_labels;
        for (const auto& tup : cloud_labels) only_labels.push_back(tup.second);
        cv::Mat inverted_mask;
        cv::bitwise_not(query_mask, inverted_mask);
        query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
        //cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, only_labels, T);
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_cloud, query_mask, sweep_path, K, camera_transforms[scan_index],
                                                                              query_label, retrieved_clouds, sweep_paths, only_labels, T);

        //cv::imshow("Image with labels", visualization);
        //cv::waitKey();
        visualizations.push_back(visualization);

        TICK("update_ratios");
        for (auto tup : cloud_labels) {
            CloudT::Ptr retrieved_cloud;
            std::string retrieved_label;
            tie(retrieved_cloud, retrieved_label) = tup;
            if (retrieved_label == "chair3") {
                retrieved_label = "chair2"; // these two objects are identical, will be counted as the same instance
            }

            current_result.ratio.first += double(retrieved_label == query_label);
            current_result.ratio.second += 1.0;

            correct_ratio& instance_ratio = current_result.instance_ratios[query_label];
            instance_ratio.first += double(retrieved_label == query_label);
            instance_ratio.second += 1.0;

            std::cout << "Query label: " << query_label << std::endl;
            std::cout << "Retrieved label: " << retrieved_label << std::endl;
        }
        TOCK("update_ratios");

        Stopwatch::getInstance().sendAll();
    }

    return std::make_pair(current_result, visualizations);
}


} // namespace benchmark_retrieval

#endif // BENCHMARK_RETRIEVAL
