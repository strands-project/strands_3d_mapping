#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <pcl/features/cvfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/features/esf.h>

#include <flann/io/hdf5.h>

#include "eigen_cereal/eigen_cereal.h"
#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;
using EFSs = pcl::PointCloud<pcl::ESFSignature640>;
using esf_model = pair<string, vector<float> > ;

inline void nearestKSearch(flann::Index<flann::L1<float> > &index, const esf_model &model,
                           int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
    // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
    memcpy(&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch(p, indices, distances, k, flann::SearchParams(512));
    delete[] p.ptr();
}

void load_signatures(vector<esf_model>& models, const boost::filesystem::path& data_path)
{
    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_segment_map segment_paths(data_path);
    dynamic_object_retrieval::convex_sweep_index_map sweep_indices(data_path);

    int last_sweep = -1;
    size_t counter = 0;
    // compute and save ESF signatires
    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_paths, sweep_indices)) {
        CloudT::Ptr cloud;
        boost::filesystem::path path;
        size_t sweep_index;
        tie(cloud, path, sweep_index) = tup;
        if (sweep_index != last_sweep) {
            last_sweep = sweep_index;
            counter = 0;
        }

        std::stringstream ss;
        ss << "esf_feature" << std::setw(4) << std::setfill('0') << counter;
        boost::filesystem::path esf_file = path.parent_path() / (ss.str() + ".cereal");
        models.push_back(esf_model());
        ifstream in(esf_file.string(), std::ios::binary);
        {
            cereal::BinaryInputArchive archive_i(in);
            archive_i(models.back()); // Is reading of the K matrix slowing this down?
        }
        in.close();

        ++counter;
    }
}

void load_tree(flann::Index<flann::L1<float> >** index, flann::Matrix<float>& data,
               vector<esf_model>& models, const boost::filesystem::path& data_path)
{
    boost::filesystem::path kdtree_idx_file_name = data_path / "kdtree_esf.idx";
    boost::filesystem::path training_data_h5_file_name = data_path / "training_data_esf.h5";
    boost::filesystem::path training_data_list_file_name = data_path / "training_data_esf.list";

    // Check if the data has already been saved to disk
    if (!boost::filesystem::exists(training_data_h5_file_name) || !boost::filesystem::exists(training_data_list_file_name)) {
        pcl::console::print_error ("Could not find training data models files %s and %s!\n",
                                   training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());
        exit(-1);
    }
    else {
        load_signatures(models, data_path);
        flann::load_from_file(data, training_data_h5_file_name.string(), "training_data_esf");
        pcl::console::print_highlight("Training data found. Loaded %d ESF models from %s/%s.\n",
                                      (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    }

    // Check if the tree index has already been saved to disk
    if (!boost::filesystem::exists(kdtree_idx_file_name)) {
        pcl::console::print_error("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
        exit(-1);
    }
    else {
        *index = new flann::Index<flann::L1<float> >(data, flann::SavedIndexParams(kdtree_idx_file_name.string()));
        (*index)->buildIndex();
        pcl::console::print_highlight("Loaded and build the data index.\n");
    }
}

void query_tree(vector<pair<float, int> >& scores, esf_model& histogram,
                flann::Index<flann::L1<float> >& index, vector<esf_model>& models, int number_query)
{
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    nearestKSearch(index, histogram, number_query, k_indices, k_distances);

    for (int i = 0; i < number_query; ++i) {
        pair<float, int> s;
        s.first = k_distances[0][i];
        s.second = k_indices[0][i];
        scores.push_back(s);
    }
}

void compute_esf_model(esf_model& histogram, CloudT::ConstPtr cloud)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Compute ESF
    pcl::ESFEstimation<PointT, pcl::ESFSignature640> EFS;
    EFS.setInputCloud(cloud);
    EFS.setSearchMethod(tree);
    EFSs::Ptr efss(new EFSs);
    EFS.compute(*efss);

    if (efss->size() != 1) {
        cout << "EFH feature has size: " << efss->size() << endl;
        exit(-1);
    }

    histogram.first = "";
    histogram.second.resize(640);
    for (size_t j = 0; j < 640; ++j) {
        histogram.second[j] = efss->at(0).histogram[j];
    }
}

vector<pair<float, string> > query_esf_representation(flann::Index<flann::L1<float> >* index, flann::Matrix<float>& data,
                                                      vector<esf_model>& models, const dynamic_object_retrieval::data_summary& summary,
                                                      esf_model& histogram)
{
    int K = 15;

    vector<pair<float, int> > scores;
    query_tree(scores, histogram, *index, models, K);

    vector<pair<float, string> > results;
    for (int i = 0; i < K; i++)
    {
        string cpath = summary.index_convex_segment_paths[scores[i].second];
        results.push_back(make_pair(scores[i].first, cpath));
    }
    //results.resize(10);

    return results;
}

pair<vector<CloudT::Ptr>, vector<boost::filesystem::path> > load_clouds(const vector<pair<float, string> >& matches)
{
    pair<vector<CloudT::Ptr>, vector<boost::filesystem::path> > results;
    for (auto tup : matches) {
        results.first.push_back(CloudT::Ptr(new CloudT));
        results.second.push_back(boost::filesystem::path(tup.second).parent_path().parent_path() / "room.xml");
        pcl::io::loadPCDFile(tup.second, *results.first.back());
    }
    return results;
}

benchmark_retrieval::benchmark_result run_benchmark(const vector<string>& folder_xmls,
                                                    const dynamic_object_retrieval::data_summary& summary,
                                                    const boost::filesystem::path& benchmark_path,
                                                    const boost::filesystem::path& data_path)
{
    flann::Index<flann::L1<float> >* index;
    flann::Matrix<float> data;
    vector<esf_model> models;
    load_tree(&index, data, models, data_path);

    dynamic_object_retrieval::vocabulary_summary vocabulary_summary;
    benchmark_retrieval::benchmark_result benchmark(vocabulary_summary);

    int counter = 0;
    for (const string& xml : folder_xmls) {
        TICK("get_score_for_sweep");
        vector<cv::Mat> visualizations;
        auto rfunc = [&](CloudT::Ptr& query_cloud, cv::Mat& query_image, cv::Mat& query_depth, const Eigen::Matrix3f& K) {
            esf_model histogram;
            compute_esf_model(histogram, query_cloud);
            vector<pair<float, string> > matches =
                query_esf_representation(index, data, models, summary, histogram);
            return load_clouds(matches);
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

    delete index;
    return benchmark;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the data..." << endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

    // create a new folder to store the benchmark
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

    boost::filesystem::path benchmark_path = string("benchmark ") + buffer + " ESF";
    boost::filesystem::create_directory(benchmark_path);

    dynamic_object_retrieval::data_summary summary;
    summary.load(data_path);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    benchmark_retrieval::benchmark_result benchmark = run_benchmark(folder_xmls, summary, benchmark_path, data_path);

    cout << "Got overall ratio: " << benchmark.ratio.first / benchmark.ratio.second << endl;

    for (const pair<string, benchmark_retrieval::correct_ratio>& instance_ratio : benchmark.instance_ratios) {
        cout << "Ratio for instance " << instance_ratio.first << ": " << instance_ratio.second.first/instance_ratio.second.second << endl;
    }

    benchmark_retrieval::save_benchmark(benchmark, benchmark_path);

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    return 0;
}
