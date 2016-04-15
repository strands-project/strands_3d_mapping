#include <dynamic_object_retrieval/dynamic_retrieval.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>
#include <object_3d_benchmark/benchmark_overlap.h>

#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

bool check_object(const string& query_label, const vector<string>& objects_to_check)
{
    for (const string& is_check : objects_to_check) {
        if (query_label.compare(0, is_check.size(), is_check) == 0) {
            return true;
        }
    }
    return false;
}

vector<tuple<CloudT::Ptr, boost::filesystem::path, size_t> > get_static_instances(CloudT::Ptr query_cloud, const string& query_label,
                                                                                  const boost::filesystem::path& data_path)
{
    const double overlap_threshold = 0.2;
    const bool visualize = false;

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    vector<tuple<CloudT::Ptr, boost::filesystem::path, size_t> > matches;
    for (const string& xml : folder_xmls) {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(xml);

        if (!check_object(query_label, labels.objectLabels)) {
            continue;
        }

        boost::filesystem::path xml_path = boost::filesystem::path(xml);
        // get all convex segments for this sweep
        dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(xml_path.parent_path());
        dynamic_object_retrieval::sweep_convex_segment_map segment_paths(xml_path.parent_path());
        dynamic_object_retrieval::sweep_convex_segment_index_map indices(xml_path.parent_path());

        for (auto tup : dynamic_object_retrieval::zip(segments, segment_paths, indices)) {
            // here it seems like we're gonna have to wrap our own solution by wrapping two octrees (or voxelgrids?)
            CloudT::Ptr c;
            boost::filesystem::path path;
            size_t index;
            tie(c, path, index) = tup;

            double overlap_fraction = benchmark_retrieval::compute_overlap(query_cloud, c);

            if (visualize && overlap_fraction > overlap_threshold) {
                CloudT::Ptr visualization_cloud(new CloudT);
                *visualization_cloud += *query_cloud;
                *visualization_cloud += *c;
                cout << "overlap fraction: " << overlap_fraction << endl;
                dynamic_object_retrieval::visualize(visualization_cloud);
            }

            if (overlap_fraction > overlap_threshold) {
                matches.push_back(make_tuple(CloudT::Ptr(new CloudT(*c)), path, index));
                break;
            }
        }

        if (matches.size() >= 24) {
            break;
        }
    }

    return matches;
}

CloudT::Ptr find_closest_segment(CloudT::Ptr& label_cloud, const string& sweep_xml)
{
    boost::filesystem::path xml_path = sweep_xml;
    dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(xml_path.parent_path());

    for (CloudT::Ptr& c : segments) {
        if (benchmark_retrieval::compute_overlap(label_cloud, c) > 0.1) {
            return CloudT::Ptr(new CloudT(*c));
        }
    }

    return CloudT::Ptr(new CloudT);
}

template<typename VocabularyT>
void visualize_static_instances(VocabularyT& vt, const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                                const dynamic_object_retrieval::vocabulary_summary& summary, const vector<string>& objects_to_check)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
    boost::filesystem::path data_path = summary.noise_data_path; // we are using the same for both right now

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

        if (!check_object(query_label, objects_to_check)) {
            continue;
        }

        // comment this if we should query for actual cloud
        /*
        query_cloud = find_closest_segment(query_cloud, sweep_xml);
        if (query_cloud->empty()) {
            continue;
        }
        dynamic_object_retrieval::visualize(query_cloud);
        */

        auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth, K, 0, vocabulary_path, summary, false);
        //tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        vector<size_t> result_indices;
        result_indices.reserve(results.first.size());
        for (auto tup : results.first) {
            typename VocabularyT::result_type index = tup.second;
            result_indices.push_back(index.index);
        }

        // in addition to this I also need their number in the vocabulary
        auto matches = get_static_instances(query_cloud, query_label, data_path);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<string> labels;
        vector<boost::filesystem::path> sweep_paths;
        for (auto tup : matches) {
            auto it = std::find(result_indices.begin(), result_indices.end(), std::get<2>(tup));
            if (it != result_indices.end()) {
                size_t dist = std::distance(result_indices.begin(), it);
                retrieved_clouds.push_back(get<0>(tup));
                labels.push_back(to_string(dist) + "/" + to_string(result_indices.size()) + "\n" + to_string(results.first[dist].second.score));
                sweep_paths.push_back(get<1>(tup).parent_path().parent_path());
            }
        }

        cv::Mat inverted_mask;
        cv::bitwise_not(query_mask, inverted_mask);
        query_image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, sweep_paths, labels, T);

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

    vector<string> objects_to_check = {"backpack2"};

    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_static_instances(vt, xml, vocabulary_path, summary, objects_to_check);
        }
    }
    /*else {
        grouped_vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_static_instances(vt, xml, vocabulary_path, summary, objects_to_check);
        }
    }*/

    return 0;
}
