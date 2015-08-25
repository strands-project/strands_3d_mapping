#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/visualize.h"

#include "vocabulary_tree/vocabulary_tree.h"

#include <cereal/archives/binary.hpp>

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

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
    vt.append_cloud(features, indices, false);
    save_vocabulary(vt, vocabulary_path);

    return counter;
}

void train_vocabulary(const boost::filesystem::path& vocabulary_path)
{
    vocabulary_summary summary;
    summary.load(vocabulary_path);

    boost::filesystem::path noise_data_path = summary.noise_data_path;
    boost::filesystem::path annotated_data_path = summary.annotated_data_path;

    if (summary.vocabulary_type == "standard") {
        convex_feature_cloud_map noise_segment_features(noise_data_path);
        convex_feature_cloud_map annotated_segment_features(annotated_data_path);
        summary.nbr_noise_segments = add_segments(noise_segment_features, vocabulary_path, summary, true, 0);
        summary.nbr_annotated_segments = add_segments(annotated_segment_features, vocabulary_path, summary, false, summary.nbr_noise_segments);
    }
    else if (summary.vocabulary_type == "incremental") {
        subsegment_feature_cloud_map noise_segment_features(noise_data_path);
        subsegment_feature_cloud_map annotated_segment_features(annotated_data_path);
        summary.nbr_noise_segments = add_segments(noise_segment_features, vocabulary_path, summary, true, 0);
        summary.nbr_annotated_segments = add_segments(annotated_segment_features, vocabulary_path, summary, false, summary.nbr_noise_segments);
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
