#define VT_PRECOMPILE
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
#include <dynamic_object_retrieval/definitions.h>
#include <tf_conversions/tf_eigen.h>

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
void visualize_query_sweep(VocabularyT& vt, const string& sweep_xml, const boost::filesystem::path& vocabulary_path,
                           const dynamic_object_retrieval::vocabulary_summary& summary)
{
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    Eigen::Matrix3f K;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
    CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(sweep_xml);

    //auto sweep_data = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_xml, vector<string>{"RoomIntermediateCloud"}, false);

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages,
                                                  labels.objectMasks, labels.objectScanIndices)) {
        CloudT::Ptr object_cloud;
        string query_label;
        cv::Mat query_image;
        cv::Mat query_mask;
        size_t scan_index;
        tie(object_cloud, query_label, query_image, query_mask, scan_index) = tup;

        //Eigen::Affine3d e;
        //tf::transformTFToEigen(sweep_data.vIntermediateRoomCloudTransformsRegistered[scan_index], e);
        //Eigen::Matrix4f T = e.inverse().matrix().cast<float>();
        Eigen::Matrix4f T = camera_transforms[scan_index];

        CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, T, K);
        //cv::imshow("Query object", query_image);
        //cv::waitKey();

        cout << "Querying object with label: " << query_label << endl;
        //*refined_query += *object_cloud;
        dynamic_object_retrieval::visualize(refined_query);

        CloudT::Ptr query_cloud(new CloudT);
        pcl::transformPointCloud(*object_cloud, *query_cloud, camera_transforms[scan_index]);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;

        auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, query_cloud, K, 10, vocabulary_path, summary, false);
        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

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

    if (summary.vocabulary_type == "standard") {
        vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_query_sweep(vt, xml, vocabulary_path, summary);
        }
    }
    else {
        grouped_vocabulary_tree<HistT, 8> vt;
        for (const string& xml : folder_xmls) {
            visualize_query_sweep(vt, xml, vocabulary_path, summary);
        }
    }

    return 0;
}

