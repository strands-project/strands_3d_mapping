#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

// there is no way to associate the extracted segments directly with any particular object
// how do we get the annotation of an object?

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

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels)) {
        CloudT::Ptr query_cloud;
        string query_label;
        tie(query_cloud, query_label) = tup;

        vector<CloudT::Ptr> retrieved_clouds;
        if (summary.vocabulary_type == "standard") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary);
            retrieved_clouds = benchmark_retrieval::load_retrieved_clouds(results.second);
        }
        else if (summary.vocabulary_type == "incremental") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<grouped_vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary);
            cout << "Loading clouds..." << endl;
            retrieved_clouds = benchmark_retrieval::load_retrieved_clouds(results.second);
            cout << "Finished loading clouds..." << endl;
        }

        cout << "Finding labels..." << endl;
        vector<pair<CloudT::Ptr, string> > cloud_labels = benchmark_retrieval::find_labels(retrieved_clouds, labels);
        cout << "Finished finding labels..." << endl;

        for (auto tup : cloud_labels) {
            CloudT::Ptr retrieved_cloud;
            string retrieved_label;
            tie(retrieved_cloud, retrieved_label) = tup;

            cout << "Query label: " << query_label << endl;
            cout << "Retrieved label: " << retrieved_label << endl;
        }
    }

    return 0;
}
