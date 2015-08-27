#include "dynamic_object_retrieval/dynamic_retrieval.h"
#include "dynamic_object_retrieval/summary_iterators.h"

using namespace std;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

namespace dynamic_object_retrieval {

vector<boost::filesystem::path> get_retrieved_paths(const vector<index_score>& scores, const vocabulary_summary& summary)
{
    // here we'll make use of the info saved in the data_summaries
    // maybe we should cache these as well as they might get big?
    data_summary noise_summary;
    noise_summary.load(boost::filesystem::path(summary.noise_data_path));
    data_summary annotated_summary;
    annotated_summary.load(boost::filesystem::path(summary.annotated_data_path));

    vector<boost::filesystem::path> retrieved_paths;
    size_t offset = summary.nbr_noise_segments;
    for (index_score s : scores) {
        if (s.first < offset) {
            retrieved_paths.push_back(boost::filesystem::path(noise_summary.index_convex_segment_paths[s.first]));
        }
        else {
            retrieved_paths.push_back(boost::filesystem::path(annotated_summary.index_convex_segment_paths[s.first-offset]));
        }
    }

    return retrieved_paths;
}

vector<path_index_score> get_retrieved_path_scores(const vector<index_score>& scores, const vocabulary_summary& summary)
{
    vector<boost::filesystem::path> paths = get_retrieved_paths(scores, summary);
    vector<path_index_score> path_scores;
    for (auto tup : zip(paths, scores)) {
        path_scores.push_back(make_tuple(get<0>(tup), get<1>(tup).first, get<1>(tup).second));
    }
    return path_scores;
}

}
