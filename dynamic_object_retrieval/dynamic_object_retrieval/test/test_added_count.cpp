#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <dynamic_object_retrieval/definitions.h>

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

template <typename FeaturesT>
size_t sum_features(FeaturesT& features, size_t min_features)
{
    size_t sum = 0;
    for (HistCloudT::Ptr& f : features) {
        if (f->size() >= min_features) {
            sum += f->size();
        }
    }

    return sum;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the vocabulary..." << endl;
        return 0;
    }

    boost::filesystem::path vocabulary_path(argv[1]);

    vocabulary_summary summary;
    summary.load(vocabulary_path);

    size_t vocabulary_size;
    size_t feature_count = 0;

    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        load_vocabulary(vt, vocabulary_path);
        vocabulary_size = vt.size();
        convex_feature_cloud_map noise_features(boost::filesystem::path(summary.noise_data_path));
        convex_feature_cloud_map annotated_features(boost::filesystem::path(summary.annotated_data_path));
        feature_count += sum_features(noise_features, summary.min_segment_features);
        feature_count += sum_features(annotated_features, summary.min_segment_features);

    }
    else if (summary.vocabulary_type == "incremental") {
        grouped_vocabulary_tree<HistT, 8> vt;
        load_vocabulary(vt, vocabulary_path);
        vocabulary_size = vt.size();
        subsegment_feature_cloud_map noise_features(boost::filesystem::path(summary.noise_data_path));
        subsegment_feature_cloud_map annotated_features(boost::filesystem::path(summary.annotated_data_path));
        feature_count += sum_features(noise_features, summary.min_segment_features);
        feature_count += sum_features(annotated_features, summary.min_segment_features);
    }

    cout << "Number of features in vocabulary: " << vocabulary_size << endl;
    cout << "Number of counted features: " << feature_count << endl;

    assert(vocabulary_size == feature_count);

    return 0;
}
