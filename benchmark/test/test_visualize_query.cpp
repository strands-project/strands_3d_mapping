#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/transforms.h>

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

void visualize_query_sweep(const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                           const dynamic_object_retrieval::vocabulary_summary& summary)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectScanIndices)) {
        CloudT::Ptr object_cloud;
        string query_label;
        cv::Mat query_image;
        size_t scan_index;
        tie(object_cloud, query_label, query_image, scan_index) = tup;
        //cv::imshow("Query object", query_image);
        //cv::waitKey();

        cout << "Querying object with label: " << query_label << endl;
        dynamic_object_retrieval::visualize(object_cloud);

        CloudT::Ptr query_cloud(new CloudT);
        pcl::transformPointCloud(*object_cloud, *query_cloud, camera_transforms[scan_index]);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        if (summary.vocabulary_type == "standard") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, false);
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);
        }
        else if (summary.vocabulary_type == "incremental") {
            auto results = dynamic_object_retrieval::query_reweight_vocabulary<grouped_vocabulary_tree<HistT, 8> >(query_cloud, K, 10, vocabulary_path, summary, false);
            cout << "Loading clouds..." << endl;
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);
            cout << "Finished loading clouds..." << endl;
        }

        for (CloudT::Ptr& cloud : retrieved_clouds) {
            dynamic_object_retrieval::visualize(cloud);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide path to annotated sweep data..." << endl;
        cout << "And the path to the vocabulary..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& xml : folder_xmls) {
        visualize_query_sweep(xml, vocabulary_path, summary);
    }

    return 0;
}

