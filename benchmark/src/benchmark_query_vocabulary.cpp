#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <Stopwatch.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/transforms.h>
#include <time.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

// there is no way to associate the extracted segments directly with any particular object
// how do we get the annotation of an object?

using correct_ratio = pair<double, double>;

pair<benchmark_retrieval::benchmark_result, vector<cv::Mat> > get_score_for_sweep(const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                                                                                  const dynamic_object_retrieval::vocabulary_summary& summary,
                                                                                  benchmark_retrieval::benchmark_result current_result)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
    Eigen::Matrix4f T = benchmark_retrieval::get_global_camera_rotation(labels);

    vector<cv::Mat> visualizations;
    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectScanIndices)) {
        CloudT::Ptr object_cloud;
        string query_label;
        cv::Mat query_image;
        size_t scan_index;
        tie(object_cloud, query_label, query_image, scan_index) = tup;
        cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(sweep_xml, scan_index);
        //cv::imshow("Query object", query_image);
        //cv::waitKey();

        CloudT::Ptr query_cloud(new CloudT);
        pcl::transformPointCloud(*object_cloud, *query_cloud, camera_transforms[scan_index]);

        TICK("total_query_vocabulary");
        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        if (summary.vocabulary_type == "standard") {
            //auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, true);
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, query_image, query_depth, K, 50, vocabulary_path, summary, false);
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);
        }
        else if (summary.vocabulary_type == "incremental") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<grouped_vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, false);
            cout << "Loading clouds..." << endl;
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);
            cout << "Finished loading clouds..." << endl;
        }
        TOCK("total_query_vocabulary");

        TICK("find_labels");
        cout << "Finding labels..." << endl;
        vector<pair<CloudT::Ptr, string> > cloud_labels = benchmark_retrieval::find_labels(retrieved_clouds, sweep_paths);
        cout << "Finished finding labels..." << endl;
        TOCK("find_labels");

        vector<string> only_labels;
        for (const auto& tup : cloud_labels) only_labels.push_back(tup.second);
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(query_image, query_label, retrieved_clouds, only_labels, T);

        //cv::imshow("Image with labels", visualization);
        //cv::waitKey();
        visualizations.push_back(visualization);

        TICK("update_ratios");
        for (auto tup : cloud_labels) {
            CloudT::Ptr retrieved_cloud;
            string retrieved_label;
            tie(retrieved_cloud, retrieved_label) = tup;

            current_result.ratio.first += double(retrieved_label == query_label);
            current_result.ratio.second += 1.0;

            correct_ratio& instance_ratio = current_result.instance_ratios[query_label];
            instance_ratio.first += double(retrieved_label == query_label);
            instance_ratio.second += 1.0;

            cout << "Query label: " << query_label << endl;
            cout << "Retrieved label: " << retrieved_label << endl;
        }
        TOCK("update_ratios");

        Stopwatch::getInstance().sendAll();
    }

    return make_pair(current_result, visualizations);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide path to annotated sweep data..." << endl;
        cout << "And the path to the vocabulary..." << endl;
        return -1;
    }

    Stopwatch::getInstance().setCustomSignature(32434);

    TICK("run");

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    // create a new folder to store the benchmark
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

    boost::filesystem::path benchmark_path = vocabulary_path / (string("benchmark ") + buffer);
    boost::filesystem::create_directory(benchmark_path);

    TICK("load_summary");
    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);
    TOCK("load_summary");

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    benchmark_retrieval::benchmark_result benchmark(summary);

    int counter = 0;
    for (const string& xml : folder_xmls) {
        TICK("get_score_for_sweep");
        vector<cv::Mat> visualizations;
        tie(benchmark, visualizations) = get_score_for_sweep(xml, vocabulary_path, summary, benchmark);
        for (cv::Mat& vis : visualizations) {
            std::stringstream ss;
            ss << "query_image" << std::setw(4) << std::setfill('0') << counter;
            cv::imwrite((benchmark_path / (ss.str() + ".png")).string(), vis);
            ++counter;
        }
        TOCK("get_score_for_sweep");
    }

    TOCK("run");

    cout << "Got overall ratio: " << benchmark.ratio.first / benchmark.ratio.second << endl;

    for (const pair<string, correct_ratio>& instance_ratio : benchmark.instance_ratios) {
        cout << "Ratio for instance " << instance_ratio.first << ": " << instance_ratio.second.first/instance_ratio.second.second << endl;
    }

    benchmark_retrieval::save_benchmark(benchmark, benchmark_path);

    Stopwatch::getInstance().sendAll();

    return 0;
}
