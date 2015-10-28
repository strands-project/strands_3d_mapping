#include <dynamic_object_retrieval/dynamic_retrieval.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>

#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

void visualize_query_sweep(const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                           const dynamic_object_retrieval::vocabulary_summary& summary)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);

    tf::StampedTransform tt = labels.transformToGlobal;
    Eigen::Affine3d e;
    tf::transformTFToEigen(tt, e);
    Eigen::Matrix4f T = e.matrix().cast<float>();
    T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectScanIndices)) {
        CloudT::Ptr query_cloud;
        string query_label;
        cv::Mat query_image;
        size_t scan_index;
        tie(query_cloud, query_label, query_image, scan_index) = tup;

        //vector<CloudT::Ptr> retrieved_clouds;
        cv::Mat visualization;
        if (summary.vocabulary_type == "standard") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, false);
            //retrieved_clouds = benchmark_retrieval::load_retrieved_clouds(results.first);
            visualization = benchmark_retrieval::make_visualization_image(results.first, T);
        }
        else if (summary.vocabulary_type == "incremental") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<grouped_vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, false);
            cout << "Loading clouds..." << endl;
            //retrieved_clouds = benchmark_retrieval::load_retrieved_clouds(results.first);
            visualization = benchmark_retrieval::make_visualization_image(results.first, T);
            cout << "Finished loading clouds..." << endl;
        }

        cv::imshow("Query object", query_image);
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

    /*
    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(folder_xmls[0]);
    Eigen::Matrix4f T = camera_transforms[0];
    cout << T << endl;
    int dummy;
    cin >> dummy;
    T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    */

    for (const string& xml : folder_xmls) {
        visualize_query_sweep(xml, vocabulary_path, summary);
    }

    return 0;
}
