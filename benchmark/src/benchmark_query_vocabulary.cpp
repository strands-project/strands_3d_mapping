#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

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
    semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

    image_geometry::PinholeCameraModel model = data.vIntermediateRoomCloudCamParams[0];
    cv::Matx33d cvK = model.intrinsicMatrix();
    Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);
    Eigen::Matrix3f K = dK.cast<float>().transpose();

    cout << K << endl;
    return 0;

    for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels)) {
        CloudT::Ptr object_cloud;
        string object_label;
        tie(object_cloud, object_label) = tup;

        if (summary.vocabulary_type == "standard") {
            dynamic_object_retrieval::query_reweight_vocabulary<vocabulary_tree<HistT, 8> >(object_cloud, K, 10, vocabulary_path, summary);
        }
        else if (summary.vocabulary_type == "incremental") {
            dynamic_object_retrieval::query_reweight_vocabulary<grouped_vocabulary_tree<HistT, 8> >(object_cloud, K, 10, vocabulary_path, summary);
        }
    }

    return 0;
}
