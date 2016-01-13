#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <Stopwatch.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <time.h>
#include <dynamic_object_retrieval/definitions.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>

#include <vlad/common.h>
#include <vlad/vlad_representation.h>

#include <k_means_tree/k_means_tree.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

// there is no way to associate the extracted segments directly with any particular object
// how do we get the annotation of an object?

benchmark_retrieval::benchmark_result run_benchmark(const vector<string>& folder_xmls,
                                                    const dynamic_object_retrieval::data_summary& summary,
                                                    const boost::filesystem::path& benchmark_path)
{

    dynamic_object_retrieval::vocabulary_summary vocabulary_summary;
    benchmark_retrieval::benchmark_result benchmark(vocabulary_summary);

    VladCloudT::Ptr vcloud(new VladCloudT);
    pcl::KdTreeFLANN<VladT> kdtree;

    int counter = 0;
    for (const string& xml : folder_xmls) {
        TICK("get_score_for_sweep");
        vector<cv::Mat> visualizations;
        auto rfunc = [&](CloudT::Ptr& query_cloud, cv::Mat& query_image, cv::Mat& query_depth, const Eigen::Matrix3f& K) {
            HistCloudT::Ptr query_features(new HistCloudT);
            CloudT::Ptr keypoints(new CloudT);
            pfhrgb_estimation::compute_features(query_features, keypoints, query_cloud);
            vector<pair<float, string> > matches =
                vlad_representation::query_vlad_representation(vcloud, kdtree, summary, query_features);
            return load_vlad_clouds(matches);
        };

        tie(benchmark, visualizations) = benchmark_retrieval::get_score_for_sweep(rfunc, xml, benchmark);
        cout << "Performed vlad query" << endl;
        for (cv::Mat& vis : visualizations) {
            std::stringstream ss;
            ss << "query_image" << std::setw(4) << std::setfill('0') << counter;
            cv::imwrite((benchmark_path / (ss.str() + ".png")).string(), vis);
            ++counter;
        }
        TOCK("get_score_for_sweep");
    }

    return benchmark;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide path to annotated sweep data..." << endl;
        return -1;
    }

    Stopwatch::getInstance().setCustomSignature(32434);

    TICK("run");

    boost::filesystem::path data_path(argv[1]);

    // create a new folder to store the benchmark
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

    boost::filesystem::path benchmark_path = string("benchmark ") + buffer + " VLAD";
    boost::filesystem::create_directory(benchmark_path);

    TICK("load_summary");
    dynamic_object_retrieval::data_summary summary;
    summary.load(data_path);
    TOCK("load_summary");

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    benchmark_retrieval::benchmark_result benchmark = run_benchmark(folder_xmls, summary, benchmark_path);

    TOCK("run");

    cout << "Got overall ratio: " << benchmark.ratio.first / benchmark.ratio.second << endl;

    for (const pair<string, benchmark_retrieval::correct_ratio>& instance_ratio : benchmark.instance_ratios) {
        cout << "Ratio for instance " << instance_ratio.first << ": " << instance_ratio.second.first/instance_ratio.second.second << endl;
    }

    benchmark_retrieval::save_benchmark(benchmark, benchmark_path);

    Stopwatch::getInstance().sendAll();

    return 0;
}
