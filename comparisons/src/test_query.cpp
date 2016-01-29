#include <vlad/vlad_representation.h>
#include <vlad/bow_representation.h>
#include <vlad/common.h>
#include <tf_conversions/tf_eigen.h>

#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_visualization.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

void visualize_query_sweep(const string& sweep_xml,
                           const dynamic_object_retrieval::data_summary& summary,
                           const vector<string>& objects_to_check,
                           VladCloudT::Ptr& vcloud, pcl::KdTreeFLANN<VladT>& kdtree)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
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

        CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, camera_transforms[scan_index], K);

        HistCloudT::Ptr query_features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        pfhrgb_estimation::compute_surfel_features(query_features, keypoints, refined_query);
        //vector<pair<float, string> > matches =
        //    bow_representation::query_bow_representation(summary, query_features);
        vector<pair<float, string> > matches =
            vlad_representation::query_vlad_representation(vcloud, kdtree, summary, query_features);

        vector<string> dummy_labels;
        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        for (int i = 0; i < matches.size(); ++i) {
            dummy_labels.push_back(string("Score: ") + to_string(matches[i].first));
            retrieved_clouds.push_back(CloudT::Ptr(new CloudT));
            cout << matches[i].second << endl;
            pcl::io::loadPCDFile(matches[i].second, *retrieved_clouds.back());
            boost::filesystem::path path = boost::filesystem::path(matches[i].second).parent_path().parent_path();
            sweep_paths.push_back(path);
            //cout << retrieved_clouds.back()->size() << endl;
        }
        cv::Mat inverted_mask;
        cv::bitwise_not(query_mask, inverted_mask);
        query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, dummy_labels, T);
        cv::imwrite("results_image.png", visualization);
        cv::imshow("Retrieved clouds", visualization);
        cv::waitKey();
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide data set path..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    //bow_representation::encode_bow_representation(data_path);
    vlad_representation::vlad_repr repr;
    repr.dimension = 250; // pfh feature dimension
    repr.numCenters = nbr_centers; // seems good defaults

    vlad_representation::build_vlad_representation(data_path, 400000, repr);
    vlad_representation::encode_vlad_representation(data_path, repr);

    vl_kmeans_delete(repr.kmeans);
    return 0;

    dynamic_object_retrieval::data_summary summary;
    summary.load(data_path);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    vector<string> objects_to_check = {"backpack", "trash", "desktop", "helmet", "chair", "pillow"};
    VladCloudT::Ptr vcloud(new VladCloudT);
    pcl::KdTreeFLANN<VladT> kdtree;

    int counter = 0;
    for (const string& xml : folder_xmls) {
        if (counter < 0) {
            ++counter;
            continue;
        }
        visualize_query_sweep(xml, summary, objects_to_check, vcloud, kdtree);
        ++counter;
    }

    return 0;
}
