#include <dynamic_object_retrieval/dynamic_retrieval.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>

#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

template<typename VocabularyT>
void visualize_query_sweep(VocabularyT& vt, const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                           const dynamic_object_retrieval::vocabulary_summary& summary, const vector<string>& objects_to_check)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
    Eigen::Matrix4f T = benchmark_retrieval::get_global_camera_rotation(labels);

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
        CloudT::Ptr query_cloud;
        string query_label;
        cv::Mat query_image;
        cv::Mat query_mask;
        size_t scan_index;
        tie(query_cloud, query_label, query_image, query_mask, scan_index) = tup;
        cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(sweep_xml, scan_index);

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

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;

        //auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 50, vocabulary_path, summary, true);
        auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth, K, 10, vocabulary_path, summary, false);
        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        vector<string> dummy_labels;
        for (int i = 0; i < retrieved_clouds.size(); ++i) {
            dummy_labels.push_back(string("result") + to_string(i));
        }
        cv::Mat inverted_mask;
        cv::bitwise_not(query_mask, inverted_mask);
        query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, dummy_labels, T);

        cv::imwrite("results_image.png", visualization);

        cv::imshow("Retrieved clouds", visualization);
        cv::waitKey();
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

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    vector<string> objects_to_check = {"backpack", "trash", "desktop", "helmet", "chair", "pillow"};

    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_query_sweep(vt, xml, vocabulary_path, summary, objects_to_check);
        }
    }
    else {
        grouped_vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_query_sweep(vt, xml, vocabulary_path, summary, objects_to_check);
        }
    }

    return 0;
}
