#include "object_3d_benchmark/benchmark_segmentation.h"
#include "object_3d_benchmark/benchmark_overlap.h"

#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>

using namespace std;

using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

grouped_vocabulary_tree<HistT, 8> vt;
map<grouped_vocabulary_tree<HistT, 8>::node*, int> mapping;
map<int, grouped_vocabulary_tree<HistT, 8>::node*> inverse_mapping;

vector<CloudT::Ptr> perform_incremental_segmentation(CloudT::Ptr& training_map, CloudT::Ptr& training_object,
                                                     CloudT::Ptr& query_map, CloudT::Ptr& query_object,
                                                     const boost::filesystem::path& query_map_path)
{
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(query_map_path.parent_path().parent_path().parent_path().string(), true);

    int sweep_index;
    bool found = false;
    for (sweep_index = 0; sweep_index < folder_xmls.size(); ++sweep_index) {
        boost::filesystem::path sweep_path = boost::filesystem::path(folder_xmls[sweep_index]).parent_path();
        if (sweep_path == query_map_path) {
            found = true;
            break;
        }
    }
    if (!found) {
        cout << "WTF? Did not find any matching sweeps..." << endl;
        exit(-1);
    }

    dynamic_object_retrieval::sweep_convex_segment_cloud_map segment_map(query_map_path);
    vector<CloudT::Ptr> segments;
    for (CloudT::Ptr& segment : segment_map) {
        segments.push_back(CloudT::Ptr(new CloudT(*segment)));
    }

    // first, we need to extract some features for the query cloud
    HistCloudT::Ptr query_cloud(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pfhrgb_estimation::compute_surfel_features(query_cloud, keypoints, training_object, false, true);

    vector<vocabulary_vector> vectors;
    set<pair<int, int> > adjacencies;

    vt.load_cached_vocabulary_vectors_for_group(vectors, adjacencies, sweep_index); // sweep index needed!

    std::vector<grouped_vocabulary_tree<HistT, 8>::result_type> scores;
    vt.top_combined_similarities(scores, query_cloud, 0);
    found = false;
    int start_index;
    float min_score = std::numeric_limits<float>::infinity();
    for (int i = 0; i < scores.size(); ++i) {
        if (scores[i].group_index == sweep_index) {
            if (benchmark_retrieval::compute_overlap(segments[scores[i].subgroup_index], query_object) > 0.0 &&
                scores[i].score < min_score) {
                found = true;
                start_index = scores[i].subgroup_index;
            }
        }
    }
    if (!found) {
        cout << "WTF? Did not find any matching min segment..." << endl;
        exit(-1);
    }

    vector<int> selected_indices;
    // get<1>(scores[i])) is actually the index within the group!
    double score = vt.compute_min_combined_dist(selected_indices, query_cloud, vectors, adjacencies,
                                                mapping, inverse_mapping, start_index); // segment index in sweep needed!

    vector<CloudT::Ptr> single_cloud;
    single_cloud.push_back(CloudT::Ptr(new CloudT));

    for (int segment_index : selected_indices) {
        *single_cloud[0] += *segments[segment_index];
    }

    return single_cloud;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide the path to the annotated data and the vocabulary path...";
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    vt.set_cache_path(vocabulary_path.string());
    dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
    vt.set_min_match_depth(3);
    vt.compute_normalizing_constants();
    vt.get_node_mapping(mapping);
    for (const pair<grouped_vocabulary_tree<HistT, 8>::node*, int>& u : mapping) {
        inverse_mapping.insert(make_pair(u.second, u.first));
    }

    map<string, pair<float, int> > overlap_ratios = benchmark_retrieval::get_segmentation_scores_for_data(&perform_incremental_segmentation, data_path);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    pair<float, int> total_ratio;
    map<string, pair<float, int> > category_ratios;
    for (const pair<string, pair<float, int> >& ratio : overlap_ratios) {
        bool found = false;
        string category;
        for (const std::string& is_check : objects_to_check) {
            if (ratio.first.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                category = is_check;
                break;
            }
        }
        if (!found) {
            continue;
        }
        total_ratio.first += ratio.second.first; total_ratio.second += ratio.second.second;

        pair<float, int>& category_ratio = category_ratios[category];
        category_ratio.first += ratio.second.first;
        category_ratio.second += ratio.second.second;
    }

    total_ratio.first /= float(total_ratio.second);
    cout << "Total ratio: " << total_ratio.first << " in " << total_ratio.second << " places" << endl;
    for (pair<const string, pair<float, int> >& ratio : category_ratios) {
        ratio.second.first /= float(ratio.second.second);
        cout << ratio.first << ": " << ratio.second.first << " in " << ratio.second.second << " places" << endl;
    }

    return 0;
}
