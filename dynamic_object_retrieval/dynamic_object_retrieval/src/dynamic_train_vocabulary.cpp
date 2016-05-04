#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/visualize.h"

#include <object_3d_retrieval/supervoxel_segmentation.h>

#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include <cereal/archives/binary.hpp>
#include <pcl/common/centroid.h>
#include "dynamic_object_retrieval/definitions.h"

#define WITH_NOISE_SET 0

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

// TODO: finish this
set<pair<int, int> > compute_group_adjacencies_subsegments(CloudT::Ptr& centroids, float adj_dist)
{
    set<pair<int, int> > adjacencies;

    for (int i = 0; i < centroids->size(); ++i) {
        for (int j = 0; j < i; ++j) {
            if ((centroids->at(i).getVector3fMap()-centroids->at(j).getVector3fMap()).norm() < adj_dist) {
                adjacencies.insert(make_pair(i, j));
                // adjacencies.insert(make_pair(j, i));
            }
        }
    }

    return adjacencies;
}

set<pair<int, int> > compute_group_adjacencies_supervoxels(const boost::filesystem::path& segment_path)
{
    set<pair<int, int> > adjacencies;

    boost::filesystem::path graph_path = segment_path / "graph.cereal";
    supervoxel_segmentation ss;
    supervoxel_segmentation::Graph g;
    ss.load_graph(g, graph_path.string());

    typename boost::property_map<supervoxel_segmentation::Graph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, g);

    // now iterate over all of the adjacencies in the original graph and add them to the set data structure
    using edge_iterator = boost::graph_traits<supervoxel_segmentation::Graph>::edge_iterator;
    edge_iterator edge_it, edge_end;
    for (tie(edge_it, edge_end) = boost::edges(g); edge_it != edge_end; ++edge_it) {
        supervoxel_segmentation::Vertex u = source(*edge_it, g);
        supervoxel_segmentation::Vertex v = target(*edge_it, g);
        supervoxel_segmentation::vertex_name_property from = boost::get(vertex_name, u);
        supervoxel_segmentation::vertex_name_property to = boost::get(vertex_name, v);
        adjacencies.insert(make_pair(from.m_value, to.m_value));
    }

    return adjacencies;
}

template <typename SegmentMapT>
size_t add_segments(SegmentMapT& segment_features, const boost::filesystem::path& vocabulary_path,
                    const vocabulary_summary& summary, bool training, size_t offset)
{
    size_t min_segment_features = summary.min_segment_features;
    size_t max_training_features = summary.max_training_features;
    size_t max_append_features = summary.max_append_features;

    vocabulary_tree<HistT, 8> vt;

    if (!training) {
        load_vocabulary(vt, vocabulary_path);
    }

    HistCloudT::Ptr features(new HistCloudT);
    vector<int> indices;

    size_t counter = 0;
    // add an iterator with the segment nbr???
    for (HistCloudT::Ptr features_i : segment_features) {

        // train on a subset of the provided features
        if (training && features->size() > max_training_features) {
            vt.set_input_cloud(features, indices);
            vt.add_points_from_input_cloud(false);
            features->clear();
            indices.clear();
            training = false;
        }

        if (!training && features->size() > max_append_features) {
            vt.append_cloud(features, indices, false);
            features->clear();
            indices.clear();
        }

        if (features_i->size() < min_segment_features) {
            ++counter;
            continue;
        }
        features->insert(features->end(), features_i->begin(), features_i->end());
        for (size_t i = 0; i < features_i->size(); ++i) {
            indices.push_back(offset + counter);
        }

        ++counter;
    }

    // append the rest
    if (features->size() > 0) {
        vt.append_cloud(features, indices, false);
    }

    save_vocabulary(vt, vocabulary_path);

    return counter;
}

// with some small differences, this could be used instead of the first one
// add VocabularyT::index_type in the classes
// also, add an iterator that returns segment and sweep index for segments
// TODO: we really just need the sweep path map, could use that instead of sweep index map
template <typename VocabularyT, typename SegmentMapT, typename KeypointMapT, typename SweepIndexMapT, typename SegmentPathMapT>
pair<size_t, size_t> add_segments_grouped(SegmentMapT& segment_features, KeypointMapT& segment_keypoints,
                                          SweepIndexMapT& sweep_indices, SegmentPathMapT& segment_paths,
                                          const boost::filesystem::path& vocabulary_path, const vocabulary_summary& summary,
                                          bool training, const size_t sweep_offset, size_t offset)
{
    size_t min_segment_features = summary.min_segment_features;
    size_t max_training_features = summary.max_training_features;
    size_t max_append_features = summary.max_append_features;

    VocabularyT vt(vocabulary_path.string());
    vt.set_min_match_depth(3);

    if (!training) {
        load_vocabulary(vt, vocabulary_path);
    }

    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr centroids(new CloudT);
    vector<set<pair<int, int> > > adjacencies;
    //vector<pair<int, int> > indices;
    vector<typename VocabularyT::index_type> indices;

    size_t counter = 0;
    size_t sweep_i;
    size_t last_sweep = 0;
    size_t sweep_counter = 0;
    boost::filesystem::path segment_path;
    // add an iterator with the segment nbr??? maybe not
    // but! add an iterator with the sweep nbr!
    for (auto tup : zip(segment_features, segment_keypoints, sweep_indices, segment_paths)) {

        HistCloudT::Ptr features_i;
        CloudT::Ptr keypoints_i;
        boost::filesystem::path last_segment = segment_path;
        tie(features_i, keypoints_i, sweep_i, segment_path) = tup;

        //cout << "Sweep: " << sweep_i << endl;

        // train on a subset of the provided features
        if (sweep_i != last_sweep) {

            if (summary.subsegment_type == "supervoxel" || summary.subsegment_type == "convex_segment") {
                // is the "segment_path" here actually the new segment, not the last one that we're interested in?
                adjacencies.push_back(compute_group_adjacencies_supervoxels(last_segment.parent_path()));
            }
            else {
                adjacencies.push_back(compute_group_adjacencies_subsegments(centroids, 0.3f));
            }
            centroids->clear();

            if (training && features->size() > max_training_features) {
                cout << "Training using features from " << sweep_i << " sweeps" << endl;

                vt.set_input_cloud(features, indices);
                vt.add_points_from_input_cloud(adjacencies, false);
                features->clear();
                indices.clear();
                adjacencies.clear();
                training = false;
            }

            if (!training && features->size() > max_append_features) {
                cout << "Appending " << features->size() << " points in " << adjacencies.size() << " groups" << endl;

                cout << adjacencies.size() << endl;
                cout << features->size() << endl;
                vt.append_cloud(features, indices, adjacencies, false);
                features->clear();
                indices.clear();
                adjacencies.clear();
            }

            last_sweep = sweep_i;
            sweep_counter = 0;
        }

        if (features_i->size() < min_segment_features) {
            ++counter;
            ++sweep_counter;
            continue;
        }

        Eigen::Vector4f point;
        pcl::compute3DCentroid(*keypoints_i, point);
        centroids->push_back(PointT());
        centroids->back().getVector4fMap() = point;
        features->insert(features->end(), features_i->begin(), features_i->end());

        //pair<int, int> index(sweep_offset + sweep_i, offset + counter);
        typename VocabularyT::index_type index(sweep_offset + sweep_i, offset + counter, sweep_counter);
        for (size_t i = 0; i < features_i->size(); ++i) {
            indices.push_back(index);
        }

        ++counter;
        ++sweep_counter;
    }

    // append the rest
    cout << "Appending " << features->size() << " points in " << adjacencies.size() << " groups" << endl;

    if (features->size() > 0) {
        if (summary.subsegment_type == "supervoxel" || summary.subsegment_type == "convex_segment") {
            adjacencies.push_back(compute_group_adjacencies_supervoxels(segment_path.parent_path()));
        }
        else {
            adjacencies.push_back(compute_group_adjacencies_subsegments(centroids, 0.3f));
        }
        vt.append_cloud(features, indices, adjacencies, false);
    }

    save_vocabulary(vt, vocabulary_path);

    return make_pair(counter, sweep_i + 1);
}

void train_vocabulary(const boost::filesystem::path& vocabulary_path)
{
    vocabulary_summary summary;
    summary.load(vocabulary_path);

    summary.min_segment_features = 30;
    summary.max_training_features = 400000; // 150000;
    summary.max_append_features = 1000000; // 1000000;

    boost::filesystem::path noise_data_path = summary.noise_data_path;
#if WITH_NOISE_SET
    boost::filesystem::path annotated_data_path = summary.annotated_data_path;
#endif

    if (summary.vocabulary_type == "standard") {
        convex_feature_cloud_map noise_segment_features(noise_data_path);
        summary.nbr_noise_segments = add_segments(noise_segment_features, vocabulary_path, summary, true, 0);
#if WITH_NOISE_SET
        convex_feature_cloud_map annotated_segment_features(annotated_data_path);
        summary.nbr_annotated_segments = add_segments(annotated_segment_features, vocabulary_path, summary, false, summary.nbr_noise_segments);
#endif
    }
    else if (summary.vocabulary_type == "incremental" && summary.subsegment_type == "convex_segment") {
        convex_feature_cloud_map noise_segment_features(noise_data_path);
        convex_keypoint_cloud_map noise_segment_keypoints(noise_data_path);
        convex_sweep_index_map noise_sweep_indices(noise_data_path);
        convex_segment_map noise_segment_paths(noise_data_path);
        tie(summary.nbr_noise_segments, summary.nbr_noise_sweeps) =
                add_segments_grouped<grouped_vocabulary_tree<HistT, 8> >(
                    noise_segment_features, noise_segment_keypoints, noise_sweep_indices,
                    noise_segment_paths, vocabulary_path, summary, true, 0, 0);
#if WITH_NOISE_SET
        convex_feature_cloud_map annotated_segment_features(annotated_data_path);
        convex_keypoint_cloud_map annotated_segment_keypoints(annotated_data_path);
        convex_sweep_index_map annotated_sweep_indices(annotated_data_path);
        convex_segment_map annotated_segment_paths(annotated_data_path);
        tie(summary.nbr_annotated_segments, summary.nbr_annotated_sweeps) =
                add_segments_grouped<grouped_vocabulary_tree<HistT, 8> >(
                    annotated_segment_features, annotated_segment_keypoints, annotated_sweep_indices,
                    annotated_segment_paths, vocabulary_path, summary, false, summary.nbr_noise_sweeps, summary.nbr_noise_segments);
#endif
    }
    else if (summary.vocabulary_type == "incremental") {
        subsegment_feature_cloud_map noise_segment_features(noise_data_path);
        subsegment_keypoint_cloud_map noise_segment_keypoints(noise_data_path);
        subsegment_sweep_index_map noise_sweep_indices(noise_data_path);
        subsegment_map noise_segment_paths(noise_data_path);
        tie(summary.nbr_noise_segments, summary.nbr_noise_sweeps) =
                add_segments_grouped<grouped_vocabulary_tree<HistT, 8> >(
                    noise_segment_features, noise_segment_keypoints, noise_sweep_indices,
                    noise_segment_paths, vocabulary_path, summary, true, 0, 0);
#if WITH_NOISE_SET
        subsegment_feature_cloud_map annotated_segment_features(annotated_data_path);
        subsegment_keypoint_cloud_map annotated_segment_keypoints(annotated_data_path);
        subsegment_sweep_index_map annotated_sweep_indices(annotated_data_path);
        subsegment_map annotated_segment_paths(annotated_data_path);
        tie(summary.nbr_annotated_segments, summary.nbr_annotated_sweeps) =
                add_segments_grouped<grouped_vocabulary_tree<HistT, 8> >(
                    annotated_segment_features, annotated_segment_keypoints, annotated_sweep_indices,
                    annotated_segment_paths, vocabulary_path, summary, false, summary.nbr_noise_sweeps, summary.nbr_noise_segments);
#endif
    }

    summary.save(vocabulary_path);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the vocabulary..." << endl;
        return 0;
    }

    train_vocabulary(boost::filesystem::path(argv[1]));

    return 0;
}
