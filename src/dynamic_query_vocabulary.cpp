#include "dynamic_object_retrieval/dynamic_retrieval.h"
#include "dynamic_object_retrieval/visualize.h"
#include "extract_sift/extract_sift.h"

#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include <cereal/archives/binary.hpp>
#include <pcl/io/pcd_io.h>

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

using namespace std;

template <typename IndexT>
void visualize_retrived_paths(vector<pair<boost::filesystem::path, IndexT> >& retrieved_paths)
{
    using path_index_type = pair<boost::filesystem::path, IndexT>;

    for (path_index_type s : retrieved_paths) {
        IndexT index;
        boost::filesystem::path path;
        tie(path, index) = s;
        cout << "Path: " << path.string() << endl;
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(path.string(), *cloud);
        dynamic_object_retrieval::visualize(cloud);
    }
}

template <typename VocabularyT>
void query_and_visualize(const boost::filesystem::path& cloud_path, const boost::filesystem::path& vocabulary_path,
                         const dynamic_object_retrieval::vocabulary_summary& summary)
{
    using result_type = vector<pair<boost::filesystem::path, typename VocabularyT::result_type> >;

    result_type retrieved_paths;
    result_type reweighted_paths;
    tie(retrieved_paths, reweighted_paths) = dynamic_object_retrieval::query_reweight_vocabulary<VocabularyT>(cloud_path, 10, vocabulary_path, summary);
    visualize_retrived_paths(retrieved_paths);
    visualize_retrived_paths(reweighted_paths);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: ./dynamic_query_vocabulary /path/to/vocabulary /path/to/cloud" << endl;
        return 0;
    }

    boost::filesystem::path vocabulary_path(argv[1]);
    boost::filesystem::path cloud_path(argv[2]);

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    if (summary.vocabulary_type == "standard") {
        query_and_visualize<vocabulary_tree<HistT, 8> >(cloud_path, vocabulary_path, summary);
    }
    else if (summary.vocabulary_type == "incremental") {
        query_and_visualize<grouped_vocabulary_tree<HistT, 8> >(cloud_path, vocabulary_path, summary);
    }

    return 0;
}
