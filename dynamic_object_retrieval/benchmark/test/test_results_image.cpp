#include <dynamic_object_retrieval/dynamic_retrieval.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>

#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

template <typename PathT>
bool is_path(const PathT& r, const boost::filesystem::path& p)
{
    return r.parent_path().parent_path() == p;
}

template <typename PathT>
bool is_path(const vector<PathT>& r, const boost::filesystem::path& p)
{
    return r.front().parent_path().parent_path() == p;
}

template<typename VocabularyT>
cv::Mat query_make_image(VocabularyT& vt, CloudT::Ptr& original_cloud, CloudT::Ptr& sweep_cloud, cv::Mat& query_image, cv::Mat& query_mask, cv::Mat& query_depth,
                         const string& query_label, const Eigen::Matrix4f& query_transform, const Eigen::Matrix4f& room_transform,
                         const Eigen::Matrix3f& K, const boost::filesystem::path& vocabulary_path,
                         const dynamic_object_retrieval::vocabulary_summary& summary, const boost::filesystem::path& sweep_path)
{
    using result_type = pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type>;

    vector<CloudT::Ptr> retrieved_clouds;
    vector<boost::filesystem::path> sweep_paths;

    CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, query_transform, K);
    SurfelCloudT::Ptr surfel_map(new SurfelCloudT);
    pcl::io::loadPCDFile((sweep_path.parent_path() / "surfel_map.pcd").string(), *surfel_map);

    Eigen::Vector4f center;
    pcl::compute3DCentroid(*original_cloud, center);
    if (center.head<3>().norm() > 2.5f) {
        return query_image.clone();
    }

    cout << "Query cloud size: " << refined_query->size() << endl;

    if (refined_query->size() < 500) { // this might happen if there are annotated objects in top layers of sweep
        return query_image.clone();
    }

    //auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 50, vocabulary_path, summary, true);
    //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth, K, 15,
    //                                                                   vocabulary_path, summary, surfel_map, false);
    auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth, K, 15,
                                                                       vocabulary_path, summary, surfel_map, false);
    std::remove_if(results.first.begin(), results.first.end(), [&](const result_type& r) {
        return is_path(r.first, sweep_path.parent_path());
    });
    results.first.resize(10);
    tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

    vector<string> dummy_labels;
    for (int i = 0; i < retrieved_clouds.size(); ++i) {
        dummy_labels.push_back(to_string(results.first[i].second.score));
    }
    cv::Mat inverted_mask;
    cv::bitwise_not(query_mask, inverted_mask);
    query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
    //cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, dummy_labels, room_transform);
    cv::Mat visualization = benchmark_retrieval::make_visualization_image(refined_query, query_mask, sweep_path, K, query_transform,
                                                                          query_label, retrieved_clouds, sweep_paths, dummy_labels, room_transform);

    return visualization;
}

template<typename VocabularyT>
cv::Mat reweight_query_make_image(VocabularyT& vt, CloudT::Ptr& sweep_cloud, cv::Mat& query_image, cv::Mat& query_mask, cv::Mat& query_depth,
                                  const string& query_label, const Eigen::Matrix4f& query_transform, const Eigen::Matrix4f& room_transform,
                                  const Eigen::Matrix3f& K, const boost::filesystem::path& vocabulary_path,
                                  const dynamic_object_retrieval::vocabulary_summary& summary, const boost::filesystem::path& sweep_path)
{
    using result_type = pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type>;

    vector<CloudT::Ptr> retrieved_clouds_initial;
    vector<boost::filesystem::path> sweep_paths_initial;

    CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, query_transform, K);
    SurfelCloudT::Ptr surfel_map(new SurfelCloudT);
    pcl::io::loadPCDFile((sweep_path.parent_path() / "surfel_map.pcd").string(), *surfel_map);

    cout << "Query cloud size: " << refined_query->size() << endl;

    if (refined_query->size() < 500) { // this might happen if there are annotated objects in top layers of sweep
        return query_image.clone();
    }

    cout << "Query cloud size: " << refined_query->size() << endl;

    //auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 50, vocabulary_path, summary, true);
    auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth, K, 20, vocabulary_path, summary, surfel_map, true);

    cout << "Finished querying!" << endl;

    std::remove_if(results.first.begin(), results.first.end(), [&](const result_type& r) {
        return is_path(r.first, sweep_path.parent_path());
    });
    results.first.resize(10);
    tie(retrieved_clouds_initial, sweep_paths_initial) = benchmark_retrieval::load_retrieved_clouds(results.first);

    vector<string> dummy_labels_initial;
    for (int i = 0; i < retrieved_clouds_initial.size(); ++i) {
        dummy_labels_initial.push_back(to_string(results.first[i].second.score));
    }
    cv::Mat inverted_mask;
    cv::bitwise_not(query_mask, inverted_mask);
    query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
    //cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, dummy_labels, room_transform);
    cv::Mat visualization_initial = benchmark_retrieval::make_visualization_image(refined_query, query_mask, sweep_path, K, query_transform,
                                                                                  query_label, retrieved_clouds_initial, sweep_paths_initial, dummy_labels_initial, room_transform);

    vector<CloudT::Ptr> retrieved_clouds_reweighted;
    vector<boost::filesystem::path> sweep_paths_reweighted;

    cout << "Results before removing query sweep: " << results.second.size() << endl;

    std::remove_if(results.second.begin(), results.second.end(), [&](const result_type& r) {
        return is_path(r.first, sweep_path.parent_path());
    });

    cout << "Results after removing query sweep: " << results.second.size() << endl;

    results.second.resize(10);

    tie(retrieved_clouds_reweighted, sweep_paths_reweighted) = benchmark_retrieval::load_retrieved_clouds(results.second);

    /*
        for (CloudT::Ptr& c : retrieved_clouds_reweighted) {
            dynamic_object_retrieval::visualize(c);
        }
        */

    vector<string> dummy_labels_reweighted;
    for (int i = 0; i < retrieved_clouds_reweighted.size(); ++i) {
        dummy_labels_reweighted.push_back(to_string(results.second[i].second.score));
    }
    //cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, dummy_labels, room_transform);
    cv::Mat visualization_reweighted = benchmark_retrieval::make_visualization_image(refined_query, query_mask, sweep_path, K, query_transform,
                                                                                     query_label, retrieved_clouds_reweighted, sweep_paths_reweighted, dummy_labels_reweighted, room_transform);

    cv::vconcat(visualization_initial, visualization_reweighted, visualization_initial);

    return visualization_initial;
}

template<typename VocabularyT>
void visualize_query_sweep(VocabularyT& vt, const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                           const dynamic_object_retrieval::vocabulary_summary& summary, const vector<string>& objects_to_check,
                           int object_nbr)
{
    static int object_counter = 0;

    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    boost::filesystem::path sweep_path = boost::filesystem::path(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
    CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(sweep_xml);
    Eigen::Matrix4f T = benchmark_retrieval::get_global_camera_rotation(labels);

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
        CloudT::Ptr query_cloud;
        string query_label;
        cv::Mat query_image;
        cv::Mat query_mask;
        size_t scan_index;
        tie(query_cloud, query_label, query_image, query_mask, scan_index) = tup;
        cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(sweep_xml, scan_index);

        /*
        for (PointT& p : query_cloud->points) {
            if (p.r < 50 && p.g < 50 && p.b < 50) {
                p.r = 20; p.g = 20; p.b = 20;
            }
        }
        */

        bool found = false;
        for (const string& is_check : objects_to_check) {
            if (query_label.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                break;
            }
        }
        if (!found) {
            continue;
        }

        if (object_nbr != -1 && object_counter < object_nbr) {
            ++object_counter;
            continue;
        }
        /*else if (object_nbr != -1 && object_counter > object_nbr) {
            exit(0);
        }*/

        cv::Mat visualization = query_make_image(vt, query_cloud, sweep_cloud, query_image, query_mask, query_depth,
                                                 query_label, camera_transforms[scan_index], T, K,
                                                 vocabulary_path, summary, sweep_path);

        //cv::Mat visualization = reweight_query_make_image(vt, sweep_cloud, query_image, query_mask, query_depth,
        //                                                  query_label, camera_transforms[scan_index], T, K,
        //                                                  vocabulary_path, summary, sweep_path);

        if (summary.subsegment_type == "convex_segment") {
            cv::Mat standard_visualization = query_make_image((vocabulary_tree<HistT, 8>&)vt, query_cloud, sweep_cloud, query_image, query_mask, query_depth,
                                                              query_label, camera_transforms[scan_index], T, K, vocabulary_path, summary, sweep_path);
            cv::vconcat(visualization, standard_visualization, visualization);
        }

        cv::imwrite(string("results_image") + to_string(object_counter) + ".png", visualization);

        //cv::imshow("Retrieved clouds", visualization);
        //cv::waitKey();

        ++object_counter;
    }
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide path to annotated sweep data..." << endl;
        cout << "And the path to the vocabulary..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    int object_nbr = -1;
    if (argc == 4) {
        object_nbr = atoi(argv[3]);
    }

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    // KTH Data:
    vector<string> objects_to_check = {"chair1"};//{"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};
    // G4S Data:
    //vector<string> objects_to_check = {"chair"}; //{"jacket", "chair", "plant", "printer", "bin"};
    //{"backpack", "trash", "desktop", "helmet", "chair", "pillow"};

    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_query_sweep(vt, xml, vocabulary_path, summary, objects_to_check, object_nbr);
        }
    }
    else {
        grouped_vocabulary_tree<HistT, 8> vt(vocabulary_path.string());
        dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
        vt.set_cache_path(vocabulary_path.string());
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
        cout << vt.size() << endl;
        //return 0;
        for (const string& xml : folder_xmls) {
            //visualize_query_sweep((vocabulary_tree<HistT, 8>&)vt, xml, vocabulary_path, summary, objects_to_check);
            visualize_query_sweep(vt, xml, vocabulary_path, summary, objects_to_check, object_nbr);
        }
    }

    return 0;
}
