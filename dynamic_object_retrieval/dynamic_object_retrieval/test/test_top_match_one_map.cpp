#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

template <typename PathT>
bool is_not_path(const PathT& r, const boost::filesystem::path& p)
{
    return r.parent_path().parent_path() != p;
}

template <typename PathT>
bool is_not_path(const vector<PathT>& r, const boost::filesystem::path& p)
{
    if (r.front().parent_path().parent_path() == p) {
        cout << r.front().parent_path().parent_path().string() << endl;
    }
    return r.front().parent_path().parent_path() != p;
}

template <typename PathT>
CloudT::Ptr load_cloud(const PathT& r)
{
    CloudT::Ptr cloud(new CloudT);
    pcl::io::loadPCDFile(r.string(), *cloud);
    return cloud;
}

template <typename PathT>
CloudT::Ptr load_cloud(const vector<PathT>& r)
{
    CloudT::Ptr cloud(new CloudT);
    for (const PathT& p : r) {
        CloudT::Ptr temp_cloud(new CloudT);
        pcl::io::loadPCDFile(p.string(), *temp_cloud);
        *cloud += *temp_cloud;
    }
    return cloud;
}

template<typename VocabularyT>
CloudT::Ptr top_query_cloud(VocabularyT& vt, CloudT::Ptr& query_cloud, const boost::filesystem::path& vocabulary_path,
                            const dynamic_object_retrieval::vocabulary_summary& summary, const boost::filesystem::path& sweep_path)
{
    using result_type = pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type>;

    cv::Mat query_image;
    cv::Mat query_depth;
    Eigen::Matrix3f K;
    auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, query_image, query_depth, K, 0, vocabulary_path, summary, false);
    std::remove_if(results.first.begin(), results.first.end(), [&](const result_type& r) {
        return is_not_path(r.first, sweep_path);
    });
    cout << "Entries in map: " << results.first.size() << endl;
    for (int i = 0; i < results.first.size(); ++i) {
        //cout << results.first[i].first.string() << endl;
        CloudT::Ptr temp_cloud = load_cloud(results.first[i].first);
        dynamic_object_retrieval::visualize(temp_cloud);
    }
    results.first.resize(1);
    CloudT::Ptr results_cloud = load_cloud(results.first[0].first);
    dynamic_object_retrieval::visualize(results_cloud);

    return results_cloud;
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        cout << "Please provide path a query point cloud..." << endl;
        cout << "And the path to the vocabulary..." << endl;
        cout << "And the sweep path to look in..." << endl;
        return -1;
    }

    boost::filesystem::path cloud_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);
    boost::filesystem::path sweep_path(argv[3]);

    CloudT::Ptr query_cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *query_cloud);

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    CloudT::Ptr result_cloud;
    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        result_cloud = top_query_cloud(vt, query_cloud, vocabulary_path, summary, sweep_path);
    }
    else {
        grouped_vocabulary_tree<HistT, 8> vt(vocabulary_path.string());
        result_cloud = top_query_cloud(vt, query_cloud, vocabulary_path, summary, sweep_path);
    }
    pcl::io::savePCDFileBinary("my_top_query.pcd", *result_cloud);

    return 0;
}
