#include "dynamic_object_retrieval/dynamic_retrieval.h"
#include "dynamic_object_retrieval/visualize.h"
#include "extract_sift/extract_sift.h"

#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include <cereal/archives/binary.hpp>
#include <pcl/io/pcd_io.h>

#include "dynamic_object_retrieval/definitions.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

using namespace std;

template <typename IndexT, typename PathT>
void visualize_retrieved_paths(vector<pair<PathT, IndexT> >& retrieved_paths)
{
    using path_index_type = pair<PathT, IndexT>;

    for (path_index_type s : retrieved_paths) {
        IndexT index;
        boost::filesystem::path path;
        tie(path, index) = s;
        cout << "Path: " << path.string() << " with score " << s.second.score << endl;
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(path.string(), *cloud);
        dynamic_object_retrieval::visualize(cloud);
    }
}

template <typename IndexT>
void visualize_retrieved_paths(vector<pair<vector<boost::filesystem::path>, IndexT> >& retrieved_paths)
{
    using path_index_type = pair<vector<boost::filesystem::path>, IndexT>;

    for (path_index_type s : retrieved_paths) {
        IndexT index;
        vector<boost::filesystem::path> paths;
        tie(paths, index) = s;
        CloudT::Ptr cloud(new CloudT);
        cout << "Paths: " << endl;
        for (boost::filesystem::path& path : paths) {
            cout << path.string() << endl;
            CloudT::Ptr temp(new CloudT);
            pcl::io::loadPCDFile(path.string(), *temp);
            *cloud += *temp;
        }
        cout << " with score " << s.second.score << endl;
        dynamic_object_retrieval::visualize(cloud);
    }
}

template <typename VocabularyT>
void query_and_visualize(const boost::filesystem::path& cloud_path, const boost::filesystem::path& vocabulary_path,
                         const dynamic_object_retrieval::vocabulary_summary& summary, bool do_reweighting)
{
    using result_type = vector<pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    result_type retrieved_paths;
    result_type reweighted_paths;
    VocabularyT vt;
    tie(retrieved_paths, reweighted_paths) = dynamic_object_retrieval::query_reweight_vocabulary(vt, cloud_path, 10, vocabulary_path, summary, do_reweighting);
    visualize_retrieved_paths(retrieved_paths);
    visualize_retrieved_paths(reweighted_paths);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: ./dynamic_query_vocabulary /path/to/vocabulary /path/to/cloud" << endl;
        return 0;
    }

    boost::filesystem::path vocabulary_path(argv[1]);
    boost::filesystem::path cloud_path(argv[2]);
    bool do_reweighting = false;

    string stem = cloud_path.stem().string();
    string num_str = stem.substr(stem.length() - 4);
    boost::filesystem::path feature_path = cloud_path.parent_path() / (string("feature") + num_str + ".pcd");

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    CloudT::Ptr query_cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *query_cloud);
    dynamic_object_retrieval::visualize(query_cloud);

    if (summary.vocabulary_type == "standard") {
        query_and_visualize<vocabulary_tree<HistT, 8> >(feature_path, vocabulary_path, summary, do_reweighting);
    }
    else if (summary.vocabulary_type == "incremental") {
        query_and_visualize<grouped_vocabulary_tree<HistT, 8> >(feature_path, vocabulary_path, summary, do_reweighting);
    }

    return 0;
}
