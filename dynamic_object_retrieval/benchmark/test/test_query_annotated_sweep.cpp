#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_object_retrieval/definitions.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

// there is no way to associate the extracted segments directly with any particular object
// how do we get the annotation of an object?

template<typename VocabularyT>
void query_annotated_sweep(LabelT& labels, dynamic_object_retrieval::vocabulary_summary& summary,
                           const Eigen::Matrix3f& K, const boost::filesystem::path& vocabulary_path)
{
    VocabularyT vt;

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels)) {
        CloudT::Ptr query_cloud;
        string query_label;
        tie(query_cloud, query_label) = tup;

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;

        auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(vt, query_cloud, K, 10, vocabulary_path, summary);
        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.second);

        cout << "Finding labels..." << endl;
        vector<pair<CloudT::Ptr, string> > cloud_labels = benchmark_retrieval::find_labels(retrieved_clouds, sweep_paths);
        cout << "Finished finding labels..." << endl;

        for (auto tup : cloud_labels) {
            CloudT::Ptr retrieved_cloud;
            string retrieved_label;
            tie(retrieved_cloud, retrieved_label) = tup;

            cout << "Query label: " << query_label << endl;
            cout << "Retrieved label: " << retrieved_label << endl;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide sweep xml to load and query for its objects..." << endl;
        cout << "And the path to the vocabulary..." << endl;
        return -1;
    }

    string sweep_xml(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);

    if (summary.vocabulary_type == "standard") {
        query_annotated_sweep<vocabulary_tree<HistT, 8> >(labels, summary, K, vocabulary_path);
    }
    else if (summary.vocabulary_type == "incremental") {
        query_annotated_sweep<grouped_vocabulary_tree<HistT, 8> >(labels, summary, K, vocabulary_path);
    }

    return 0;
}
