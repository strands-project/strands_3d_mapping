#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/visualize.h"

#include <object_3d_retrieval/supervoxel_segmentation.h>

#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <eigen_cereal/eigen_cereal.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree.h>
#include "dynamic_object_retrieval/definitions.h"
#include "object_3d_benchmark/benchmark_retrieval.h"
#include "dynamic_object_retrieval/dynamic_retrieval.h"

#include "density_learning/surfel_features.h"

/**
 * Notes on what is done in this file:
 *
 * We basically iterate over segment sizes / keypoint density given size / size of rest of vocabulary
 * and for all of these combinations we compute a score that is inserted into a 3-dimensional matrix
 *
 * This matrix is then used to optimize over the best comibination of segment size -> keypoint density
 * for maximizing the overall retrieval score
 *
 * The working hypothesis here is that the success of the retrieval for one given size and density
 * depends mostly on the overall number of features of the other sizes in the vocabulary and not so
 * much on the exact distribution of features between the different sizes. This is the main
 * assumption here to make the optimization tractable
 */

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

float compute_cloud_volume(CloudT::Ptr& cloud)
{
    float resolution = 0.05f;
    pcl::octree::OctreePointCloud<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    float centers = octree.getOccupiedVoxelCenters(dummy);
    return centers*resolution*resolution*resolution;
}

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

// with some small differences, this could be used instead of the first one
// add VocabularyT::index_type in the classes
// also, add an iterator that returns segment and sweep index for segments
// TODO: we really just need the sweep path map, could use that instead of sweep index map
template <typename VocabularyT, typename SegmentMapT, typename KeypointMapT, typename SweepIndexMapT, typename SegmentPathMapT>
tuple<size_t, size_t, int, int>
add_segments(VocabularyT& vt,
             SegmentMapT& segment_features, KeypointMapT& segment_keypoints,
             SweepIndexMapT& sweep_indices, SegmentPathMapT& segment_paths,
             const boost::filesystem::path& vocabulary_path,
             const boost::filesystem::path& training_vocabulary_path,
             const vocabulary_summary& summary,
             const size_t sweep_offset, size_t offset,
             const vector<float>& dividers, int current_ind,
             float density_threshold, float other_density_threshold)
{
    size_t min_segment_features = summary.min_segment_features;
    size_t max_append_features = summary.max_append_features;

    int current_nbr_features = 0;
    int other_nbr_features = 0;

    //VocabularyT vt(temp_path.string());
    if (sweep_offset == 0) {
        load_vocabulary(vt, vocabulary_path);
        vt.clear();
        vt.set_cache_path(training_vocabulary_path.string());
        vt.set_min_match_depth(3);
    }

    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr centroids(new CloudT);
    vector<set<pair<int, int> > > adjacencies;
    vector<typename VocabularyT::index_type> indices;

    size_t counter = 0;
    size_t sweep_i;
    size_t last_sweep = 0;
    size_t sweep_counter = 0;
    boost::filesystem::path segment_path;
    // add an iterator with the segment nbr??? maybe not
    // but! add an iterator with the sweep nbr!
    for (auto tup : zip(segment_features, segment_keypoints, sweep_indices, segment_paths)) {

        boost::filesystem::path feature_path;
        boost::filesystem::path keypoint_path;
        boost::filesystem::path last_segment = segment_path;
        boost::tie(feature_path, keypoint_path, sweep_i, segment_path) = tup;
        feature_path = feature_path.parent_path() / (string("density_") + feature_path.filename().string());
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());

        HistCloudT::Ptr features_all(new HistCloudT);
        pcl::io::loadPCDFile(feature_path.string(), *features_all);
        CloudT::Ptr keypoints_all(new CloudT);
        pcl::io::loadPCDFile(keypoint_path.string(), *keypoints_all);
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(segment_path.string(), *cloud);

        float volume = compute_cloud_volume(cloud);
        int size_ind;
        for (size_ind = 0; size_ind < dividers.size() && volume > dividers[size_ind]; ++size_ind) {}
        size_ind--;
        float threshold;
        if (size_ind == current_ind) {
            threshold = density_threshold;
        }
        else {
            threshold = other_density_threshold;
        }

        HistCloudT::Ptr features_i(new HistCloudT);
        CloudT::Ptr keypoints_i(new CloudT);
        for (int i = 0; i < features_all->size(); ++i) {
            if (keypoints_all->at(i).rgb < threshold) {
                features_i->push_back(features_all->at(i));
                keypoints_i->push_back(keypoints_all->at(i));
            }
        }

        if (size_ind == current_ind) {
            current_nbr_features += features_i->size();
        }
        else {
            other_nbr_features += features_i->size();
        }

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

            if (features->size() > max_append_features) {
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

    //save_vocabulary(vt, temp_path);

    return make_tuple(counter, sweep_i + 1, current_nbr_features, other_nbr_features);
}

/*
 * We need to take in a discretization of the keypoints here to be able to get a mean number
 * of keypoints for every threshold.
 *
 *
 */

vector<float> draw_histogram(const vector<float>& vals, int groups)
{
    vector<float> counts(1000); // discretize at 0.05?

    int max_ind = 0;
    int total = 0;
    for (float f : vals) {
        int ind = int((f+0.0000001) / (0.05f*0.05f*0.05f));
        if (ind > max_ind) {
            max_ind = ind;
        }
        if (ind >= counts.size()) {
            counts.resize(ind+1, 0);
        }
        counts[ind] += 1;
        ++total;
    }

    int delta = int(float(total)/float(groups));

    float max_elem = *std::max_element(counts.begin(), counts.end());

    int bar_width = 2;
    cv::Mat hist = cv::Mat::ones(800, bar_width*(max_ind+1), CV_8UC3);

    vector<float> dividers;
    dividers.push_back(0.0f);
    int current_traversed = 0;
    for (int j = 0, rows = hist.rows; j < max_ind; j++) {
        for (int b = 0; b < bar_width; ++b) {
            cv::line(
                hist,
                cv::Point(j*bar_width+b, rows),
                cv::Point(j*bar_width+b, rows - (counts[j] * rows/max_elem)),
                cv::Scalar(0,255,0),
                1, 8, 0
            );
        }
        current_traversed += counts[j];
        if (current_traversed > delta) {
            dividers.push_back(float(j)*0.05f*0.05f*0.05f);
            current_traversed = 0;
            cv::line(
                hist,
                cv::Point(j*bar_width, rows),
                cv::Point(j*bar_width, rows - (counts[j] * rows/max_elem)),
                cv::Scalar(255,0,0),
                1, 8, 0
            );
        }
    }

    cv::imshow("Histogram", hist);
    cv::waitKey();

    return dividers;
}

template <typename SegmentMapT, typename KeypointMapT>
vector<float> discretize_sizes(SegmentMapT& clouds, KeypointMapT& keypoint_paths)
{
    vector<float> volumes;
    for (auto tup : zip(clouds, keypoint_paths)) {
        boost::filesystem::path keypoint_path;
        CloudT::Ptr cloud;
        boost::tie(cloud, keypoint_path) = tup;
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());
        CloudT::Ptr keypoints(new CloudT);
        pcl::io::loadPCDFile(keypoint_path.string(), *keypoints);
        float volume = compute_cloud_volume(cloud);
        volumes.push_back(volume);
    }
    vector<float> dividers = draw_histogram(volumes, 10);
    return dividers;
}

template <typename SegmentMapT, typename KeypointMapT>
vector<float> learn_size_mappings(SegmentMapT& clouds, KeypointMapT& keypoint_paths,
                                  const vector<float>& dividers, const vector<float>& densities)
{
    vector<vector<float> > size_density_threshold_mapping(dividers.size());
    vector<vector<int> > normalizations(dividers.size());

    for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {
        size_density_threshold_mapping[size_ind].resize(densities.size(), 0.0f);
        normalizations[size_ind].resize(densities.size(), 0);
    }

    // first, order the thresholds for one set of keypoints
    for (auto tup : zip(clouds, keypoint_paths)) {
        boost::filesystem::path keypoint_path;
        CloudT::Ptr cloud;
        boost::tie(cloud, keypoint_path) = tup;
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());
        CloudT::Ptr keypoints(new CloudT);
        pcl::io::loadPCDFile(keypoint_path.string(), *keypoints);
        std::sort(keypoints->points.begin(), keypoints->points.end(), [](const PointT& p, const PointT& q) {
           return p.rgb < q.rgb;
        });
        float volume = compute_cloud_volume(cloud);
        int size_ind;
        for (size_ind = 0; size_ind < dividers.size() && volume > dividers[size_ind]; ++size_ind) {}
        size_ind--;
        for (int i = 0; i < keypoints->size(); ++i) {
            float density = float(i+1)/volume;
            int dens_ind;
            for (dens_ind = 0; dens_ind < densities.size() && density > densities[dens_ind]; ++dens_ind) {}
            dens_ind--;
            size_density_threshold_mapping[size_ind][dens_ind] += keypoints->at(i).rgb;
            normalizations[size_ind][dens_ind] += 1;
        }
    }

    std::vector<float> density_threshold_mapping(densities.size(), 0.0f);
    for (int dens_ind = 0; dens_ind < densities.size(); ++dens_ind) {
        int normalization = 0;
        for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {
            density_threshold_mapping[dens_ind] += size_density_threshold_mapping[size_ind][dens_ind];
            normalization += normalizations[size_ind][dens_ind];
            //size_density_threshold_mapping[size_ind][dens_ind] /= float(normalizations[size_ind][dens_ind]);
        }
        density_threshold_mapping[dens_ind] /= float(normalization);
    }

    return density_threshold_mapping;
}

tuple<vocabulary_summary, int, int>
train_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path,
                 const boost::filesystem::path& train_vocabulary_path,
                 const vector<float>& dividers, int current_ind,
                 float density_threshold, float other_density_threshold)
{
    vocabulary_summary summary;
    summary.load(vocabulary_path);

    summary.min_segment_features = 30;
    summary.max_training_features = 150000; // 600000; // for pfgrgb
    summary.max_append_features = 1000000; // 1000000; // for pfgrgb

    boost::filesystem::path noise_data_path = summary.noise_data_path;
    //boost::filesystem::path annotated_data_path = summary.annotated_data_path;

    // we can easily fill out some of the parts of the matrix here,
    // keeping the threshold constant or with some linear dependence on the volume
    // but how do we get the other feature sizes to a specific size to fill out the
    // matrix? maybe sample a subset and try to approximately learn how threshold
    // maps to number of features for different sizes? that's a pretty good idea...
    // actually we can just load enough of features for one size and then see where
    // we need to set the threshold to get certain sizes! great stuff

    int current_nbr_features = 0;
    int other_nbr_features = 0;

    int t1, t2;
    if (summary.vocabulary_type == "incremental" && summary.subsegment_type == "convex_segment") {
        convex_feature_map noise_segment_features(noise_data_path);
        //convex_feature_map annotated_segment_features(annotated_data_path);
        convex_keypoint_map noise_segment_keypoints(noise_data_path);
        //convex_keypoint_map annotated_segment_keypoints(annotated_data_path);
        convex_sweep_index_map noise_sweep_indices(noise_data_path);
        //convex_sweep_index_map annotated_sweep_indices(annotated_data_path);
        convex_segment_map noise_segment_paths(noise_data_path);
        //convex_segment_map annotated_segment_paths(annotated_data_path);
        tie(summary.nbr_noise_segments, summary.nbr_noise_sweeps, t1, t2) =
                add_segments(vt,
                    noise_segment_features, noise_segment_keypoints, noise_sweep_indices,
                    noise_segment_paths, vocabulary_path, train_vocabulary_path,
                    summary, 0, 0, dividers, current_ind, density_threshold, other_density_threshold);
        current_nbr_features += t1; other_nbr_features += t2;
        /*
        tie(summary.nbr_annotated_segments, summary.nbr_annotated_sweeps, t1, t2) =
                add_segments(vt,
                    annotated_segment_features, annotated_segment_keypoints, annotated_sweep_indices,
                    annotated_segment_paths, vocabulary_path, train_vocabulary_path,
                    summary, summary.nbr_noise_sweeps, summary.nbr_noise_segments,
                    dividers, current_ind, density_threshold, other_density_threshold);
        current_nbr_features += t1; other_nbr_features += t2;
        */
    }
    else if (summary.vocabulary_type == "incremental") {
        subsegment_feature_map noise_segment_features(noise_data_path);
        //subsegment_feature_map annotated_segment_features(annotated_data_path);
        subsegment_keypoint_map noise_segment_keypoints(noise_data_path);
        //subsegment_keypoint_map annotated_segment_keypoints(annotated_data_path);
        subsegment_sweep_index_map noise_sweep_indices(noise_data_path);
        //subsegment_sweep_index_map annotated_sweep_indices(annotated_data_path);
        subsegment_map noise_segment_paths(noise_data_path);
        //subsegment_map annotated_segment_paths(annotated_data_path);
        tie(summary.nbr_noise_segments, summary.nbr_noise_sweeps, t1, t2) =
                add_segments(vt,
                    noise_segment_features, noise_segment_keypoints, noise_sweep_indices,
                    noise_segment_paths, vocabulary_path, train_vocabulary_path,
                    summary, 0, 0, dividers, current_ind, density_threshold, other_density_threshold);
        current_nbr_features += t1; other_nbr_features += t2;
        /*
        tie(summary.nbr_annotated_segments, summary.nbr_annotated_sweeps, t1, t2) =
                add_segments(vt,
                    annotated_segment_features, annotated_segment_keypoints, annotated_sweep_indices,
                    annotated_segment_paths, vocabulary_path, train_vocabulary_path,
                    summary, summary.nbr_noise_sweeps, summary.nbr_noise_segments,
                    dividers, current_ind, density_threshold, other_density_threshold);
        current_nbr_features += t1; other_nbr_features += t2;
        */
    }

    return make_tuple(summary, current_nbr_features, other_nbr_features);
}

float compute_vocabulary_error(grouped_vocabulary_tree<HistT, 8>& vt, int current_ind, const vector<float>& dividers,
                               float threshold, const boost::filesystem::path& vocabulary_path,
                               const boost::filesystem::path& train_vocabulary_path,
                               const map<string, string>& path_labels, const vocabulary_summary& summary)
{
    boost::filesystem::path data_path(summary.noise_data_path);

    int correct = 0;
    int total = 0;

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
    for (const string& xml : folder_xmls) {

        //CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(xml);
        //boost::filesystem::path sweep_path = boost::filesystem::path(xml);
        SurfelCloudT::Ptr surfel_cloud = surfel_features::load_surfel_cloud_for_sweep(xml);
        Eigen::Matrix3f K;
        vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
        tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(xml);

        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(xml);
        //Eigen::Matrix4f room_transform = benchmark_retrieval::get_global_camera_rotation(labels);

        for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
            CloudT::Ptr query_cloud;
            string query_label;
            cv::Mat query_image;
            cv::Mat query_mask;
            size_t scan_index;
            tie(query_cloud, query_label, query_image, query_mask, scan_index) = tup;
            cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(xml, scan_index);

            CloudT::Ptr refined_query;
            NormalCloudT::Ptr normals;
            tie(refined_query, normals) = surfel_features::cloud_normals_from_surfel_mask(surfel_cloud, query_mask, camera_transforms[scan_index], K);
            //CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, camera_transforms[scan_index], K);
            float volume = compute_cloud_volume(refined_query);
            int size_ind;
            for (size_ind = 0; size_ind < dividers.size() && volume > dividers[size_ind]; ++size_ind) {}
            size_ind--;
            if (size_ind != current_ind) {
                continue;
            }

            HistCloudT::Ptr features(new HistCloudT);
            CloudT::Ptr keypoints(new CloudT);
            surfel_features::compute_features(features, keypoints, refined_query, normals, threshold, false);
            cout << "Features size: " << features->size() << endl;
            auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth, K, 10, train_vocabulary_path, summary, false);
            //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, features, 10, train_vocabulary_path, summary);

#if 0
            cout << "Results.size: " << results.first.size() << endl;
            //*query_cloud += *refined_query;
            //dynamic_object_retrieval::visualize(query_cloud);
            //dynamic_object_retrieval::visualize(refined_query);
            vector<CloudT::Ptr> retrieved_clouds;
            vector<boost::filesystem::path> sweep_paths;
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);
            vector<string> dummy_labels;
            for (int i = 0; i < retrieved_clouds.size(); ++i) {
                dummy_labels.push_back(to_string(results.first[i].second.score));
            }
            cv::Mat visualization = benchmark_retrieval::make_visualization_image(refined_query, query_mask, sweep_path, K, camera_transforms[scan_index],
                                                                                  query_label, retrieved_clouds, sweep_paths, dummy_labels, room_transform);
            if (!visualization.empty()) {
                cv::imshow("Query results", visualization);
                cv::waitKey();
            }
#endif

            for (const auto& r : results.first) {
                for (const boost::filesystem::path& path : r.first) {
                    cout << "Path: " << path.string() << endl;
                    if (path_labels.count(path.string()) > 0 && path_labels.at(path.string()) == query_label) {
                        ++correct;
                        break;
                    }
                }
                ++total;
            }
        }
    }

    return float(total - correct) / float(total);
}

tuple<Eigen::MatrixXf, Eigen::MatrixXi, Eigen::MatrixXi>
compute_size_density_errors(int size_ind, const vector<float>& densities, const vector<float>& dividers,
                            const vector<float>& density_threshold_mapping, const boost::filesystem::path& vocabulary_path,
                            const map<string, string>& path_labels)
{
    boost::filesystem::path train_vocabulary_path = vocabulary_path / "training1";
    boost::filesystem::create_directory(train_vocabulary_path);

    Eigen::MatrixXf errors(densities.size(), densities.size());
    Eigen::MatrixXi current_features(densities.size(), densities.size());
    Eigen::MatrixXi other_features(densities.size(), densities.size());

    for (int dens_ind = 0; dens_ind < densities.size(); ++dens_ind) {
        for (int others_dens = 0; others_dens < densities.size(); ++others_dens) {
            grouped_vocabulary_tree<HistT, 8> vt(train_vocabulary_path.string());
            vocabulary_summary summary;
            int current_nbr_features; int other_nbr_features;
            tie(summary, current_nbr_features, other_nbr_features) = train_vocabulary(vt, vocabulary_path, train_vocabulary_path, dividers, size_ind,
                                                                                      density_threshold_mapping[dens_ind], density_threshold_mapping[others_dens]);
            summary.save(train_vocabulary_path);
            int query_dens = dens_ind + 1;
            if (query_dens == densities.size()) query_dens = dens_ind;
            errors(dens_ind, others_dens) = compute_vocabulary_error(vt, size_ind, dividers, density_threshold_mapping[query_dens],
                                                                     vocabulary_path, train_vocabulary_path, path_labels, summary);
            current_features(dens_ind, others_dens) = current_nbr_features;
            other_features(dens_ind, others_dens) = other_nbr_features;
        }
    }

    return make_tuple(errors, current_features, other_features);
}

void test_querying(const vector<float>& densities, const vector<float>& dividers,
                   const vector<float>& density_threshold_mapping, const boost::filesystem::path& vocabulary_path,
                   const map<string, string>& path_labels)
{
    boost::filesystem::path train_vocabulary_path = vocabulary_path / "training2";
    boost::filesystem::create_directory(train_vocabulary_path);

    int test_size = 8;

    grouped_vocabulary_tree<HistT, 8> vt(train_vocabulary_path.string());
    vocabulary_summary summary;

    // START DEBUG
    /*
    summary.load(vocabulary_path);
    load_vocabulary(vt, vocabulary_path);
    vt.set_min_match_depth(3);
    vt.compute_normalizing_constants();
    */
    //END DEBUG
    int current_nbr_features; int other_nbr_features;
    tie(summary, current_nbr_features, other_nbr_features) = train_vocabulary(vt, vocabulary_path, train_vocabulary_path, dividers, test_size,
                                                                              density_threshold_mapping[4], density_threshold_mapping[3]);
    summary.save(train_vocabulary_path);

    float error = compute_vocabulary_error(vt, test_size, dividers, density_threshold_mapping[4],
                                           vocabulary_path, train_vocabulary_path, path_labels, summary);
    cout << "Error: " << error << endl;
}

template <typename DataT>
void save_data(const boost::filesystem::path& path, const DataT& data)
{
    ofstream out(path.string());
    {
        cereal::JSONOutputArchive archive_o(out);
        archive_o(data);
    }
    out.close();
}

template <typename DataT>
void save_binary_data(const boost::filesystem::path& path, const DataT& data)
{
    ofstream out(path.string(), std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(data);
    }
    out.close();
}

template <typename DataT>
void load_data(const boost::filesystem::path& path, DataT& data)
{
    ifstream in(path.string());
    {
        cereal::JSONInputArchive archive_i(in);
        archive_i(data);
    }
    in.close();
}

template <typename DataT>
void load_binary_data(const boost::filesystem::path& path, DataT& data)
{
    ifstream in(path.string(), std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(data);
    }
    in.close();
}

void setup_discretizations(const boost::filesystem::path& training_path, const boost::filesystem::path& data_path)
{
    /*vector<float> densities = {0.0f, 0.1f / (0.05f*0.05f*0.05f), 0.2f / (0.05f*0.05f*0.05f), 0.3f / (0.05f*0.05f*0.05f), 0.4f / (0.05f*0.05f*0.05f),
                               0.5f / (0.05f*0.05f*0.05f), 0.6f / (0.05f*0.05f*0.05f), 0.7f / (0.05f*0.05f*0.05f), 0.8f / (0.05f*0.05f*0.05f),
                               0.9f / (0.05f*0.05f*0.05f), 1.0f / (0.05f*0.05f*0.05f), 1.1f / (0.05f*0.05f*0.05f), 1.2f / (0.05f*0.05f*0.05f)};*/

    /*vector<float> densities = {0.0f, 0.2f / (0.05f*0.05f*0.05f), 0.4f / (0.05f*0.05f*0.05f), 0.6f / (0.05f*0.05f*0.05f),
                               0.8f / (0.05f*0.05f*0.05f), 1.0f / (0.05f*0.05f*0.05f), 1.2f / (0.05f*0.05f*0.05f)};*/

    vector<float> densities = {0.0f, 0.2f / (0.05f*0.05f*0.05f), 0.4f / (0.05f*0.05f*0.05f),
                               0.6f / (0.05f*0.05f*0.05f), 0.8f / (0.05f*0.05f*0.05f)};

    save_data(training_path / "densities.json", densities);

    vector<float> dividers;
    {
        convex_keypoint_map noise_segment_keypoints(data_path);
        convex_segment_cloud_map noise_segments(data_path);
        dividers = discretize_sizes(noise_segments, noise_segment_keypoints);
    }
    save_data(training_path / "dividers.json", dividers);

    vector<float> density_threshold_mapping;
    {
        convex_keypoint_map noise_segment_keypoints(data_path);
        convex_segment_cloud_map noise_segments(data_path);
        density_threshold_mapping = learn_size_mappings(noise_segments, noise_segment_keypoints,
                                                        dividers, densities);
    }
    save_data(training_path / "density_threshold_mapping.json", density_threshold_mapping);

    cout << "Densities to thresholds: " << endl;
    for (int dens_ind = 0; dens_ind < densities.size(); ++dens_ind) {
        cout << density_threshold_mapping[dens_ind] << " ";
    }
    cout << endl;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the vocabulary..." << endl;
        return 0;
    }

    //train_vocabulary(boost::filesystem::path(argv[1]));
    boost::filesystem::path vocabulary_path(argv[1]); // DEBUG
    vocabulary_summary summary;
    summary.load(vocabulary_path);
    boost::filesystem::path data_path(summary.noise_data_path);

    boost::filesystem::path training_path = vocabulary_path / "training";
    boost::filesystem::create_directory(training_path);

    /*
    setup_discretizations(training_path, data_path);
    return 0;
    */

    vector<float> densities;
    load_data(training_path / "densities.json", densities);
    vector<float> dividers;
    load_data(training_path / "dividers.json", dividers);
    vector<float> density_threshold_mapping;
    load_data(training_path / "density_threshold_mapping.json", density_threshold_mapping);

    map<string, string> path_labels;
    load_binary_data(data_path / "segment_path_labels.cereal", path_labels);

    //test_querying(densities, dividers, density_threshold_mapping, vocabulary_path, path_labels);
    //return 0;

    // 5 / 10
    //for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {

    //for (int size_ind = 0; size_ind < 5; ++size_ind) {
    for (int size_ind = 9; size_ind > 6; --size_ind) {
        Eigen::MatrixXf size_errors;
        Eigen::MatrixXi size_nbr_features;
        Eigen::MatrixXi size_nbr_other_features;

        tie(size_errors, size_nbr_features, size_nbr_other_features) = compute_size_density_errors(size_ind, densities, dividers,
                                                                                                   density_threshold_mapping, vocabulary_path,
                                                                                                   path_labels);
        save_binary_data(training_path / (string("errors") + to_string(size_ind) + ".cereal"), size_errors);
        save_binary_data(training_path / (string("features") + to_string(size_ind) + ".cereal"), size_nbr_features);
        save_binary_data(training_path / (string("others") + to_string(size_ind) + ".cereal"), size_nbr_other_features);
        cout << "Size " << dividers[size_ind] << " densities error matrix: " << endl;
        cout << size_errors << endl;
    }

    return 0;
}
