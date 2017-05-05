#define VT_PRECOMPILE
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
#include <dynamic_object_retrieval/definitions.h>
	
#define CONVEX_SEGMENTS_ONLY 0

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

template<typename VocabularyT>
benchmark_retrieval::benchmark_result run_benchmark(const vector<string>& folder_xmls, const boost::filesystem::path& vocabulary_path,
                                                    const dynamic_object_retrieval::vocabulary_summary& summary,
                                                    const boost::filesystem::path& benchmark_path)
{

    benchmark_retrieval::benchmark_result benchmark(summary);
    VocabularyT vt;
    if (vt.empty()) {
        dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
        vt.set_cache_path(vocabulary_path.string());
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
    }

    int counter = 0;
    for (const string& xml : folder_xmls) {

        if (counter == 717 || counter == 833) { // have a bug in visualization of these sweeps for re-weight querying causing crash, TODO: fix this
            ++counter;
            continue;
        }

        TICK("get_score_for_sweep");
        vector<cv::Mat> visualizations;
        auto rfunc = [&](CloudT::Ptr& query_cloud, cv::Mat& query_image, cv::Mat& query_depth, SurfelCloudT::Ptr& surfel_map, const Eigen::Matrix3f& K) {
#if CONVEX_SEGMENTS_ONLY
            auto results = dynamic_object_retrieval::query_reweight_vocabulary((vocabulary_tree<HistT, 8>&)vt, query_cloud, query_image, query_depth,
                                                                               K, 15, vocabulary_path, summary, surfel_map, false);
#else
            // no re-weighting
            //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth,
            //                                                                   K, 15, vocabulary_path, summary, surfel_map, false);
            // re-weighting
            auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth,
                                                                               K, 15, vocabulary_path, summary, surfel_map, true);
#endif
            // no re-weighting
            //return benchmark_retrieval::load_retrieved_clouds(results.first);
            // re-weighting
            return benchmark_retrieval::load_retrieved_clouds(results.second);
        };

        tie(benchmark, visualizations) = benchmark_retrieval::get_score_for_sweep(rfunc, xml, benchmark);
        for (cv::Mat& vis : visualizations) {
            std::stringstream ss;
            ss << "query_image" << std::setw(4) << std::setfill('0') << counter;
            cv::imwrite((benchmark_path / (ss.str() + ".png")).string(), vis);
            ++counter;
        }
        TOCK("get_score_for_sweep");

        benchmark_retrieval::save_benchmark(benchmark, benchmark_path);
    }

    return benchmark;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide path to the vocabulary and the annotated data path(s) to query..." << endl;
        return -1;
    }

    Stopwatch::getInstance().setCustomSignature(32434);

    TICK("run");

    boost::filesystem::path vocabulary_path(argv[1]);
    vector<string> folder_xmls;
    for (int i = 2; i < argc; ++i) {
        boost::filesystem::path data_path(argv[i]);
        vector<string> data_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
        folder_xmls.insert(folder_xmls.end(), data_xmls.begin(), data_xmls.end());
    }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    /*
    for (const string& xml : folder_xmls) {
        cout << xml << endl;
    }
    return 0;
    */

    // create a new folder to store the benchmark
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

#if CONVEX_SEGMENTS_ONLY
    boost::filesystem::path benchmark_path = vocabulary_path / (string("benchmark ") + buffer + " CONVEX SURFELS");
#else
    boost::filesystem::path benchmark_path = vocabulary_path / (string("benchmark ") + buffer + " INCREMENTAL SURFELS");
#endif
    boost::filesystem::create_directory(benchmark_path);

    TICK("load_summary");
    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);
    TOCK("load_summary");

    benchmark_retrieval::benchmark_result benchmark;
    if (summary.vocabulary_type == "standard") {
        //benchmark = run_benchmark<vocabulary_tree<HistT, 8> >(folder_xmls, vocabulary_path, summary, benchmark_path);
    }
    else if (summary.vocabulary_type == "incremental") {
        benchmark = run_benchmark<grouped_vocabulary_tree<HistT, 8> >(folder_xmls, vocabulary_path, summary, benchmark_path);
    }

    TOCK("run");

    cout << "Got overall ratio: " << benchmark.ratio.first / benchmark.ratio.second << endl;

    for (const pair<string, benchmark_retrieval::correct_ratio>& instance_ratio : benchmark.instance_ratios) {
        cout << "Ratio for instance " << instance_ratio.first << ": " << instance_ratio.second.first/instance_ratio.second.second << endl;
    }

    benchmark_retrieval::save_benchmark(benchmark, benchmark_path);

    Stopwatch::getInstance().sendAll();

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    std::cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << std::endl;

    return 0;
}
