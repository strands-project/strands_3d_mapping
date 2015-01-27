#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/extract_clusters.h>

namespace semantic_map_load_utilties
{

    /********************************************** MERGED CLOUD UTILITIES ****************************************************************************************/
    template <class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> loadMergedCloudFromSingleSweep(std::string sweepXmlPath, bool verbose=false);

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudFromMultipleSweeps(std::string folderPath, bool verbose=false);

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false);

    /********************************************** INTERMEDIATE CLOUD UTILITIES ****************************************************************************************/

    template <class PointType>
    struct IntermediateCloudCompleteData
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  vIntermediateRoomClouds;
        std::vector<tf::StampedTransform>                           vIntermediateRoomCloudTransforms;
        std::vector<image_geometry::PinholeCameraModel>             vIntermediateRoomCloudCamParams;
        std::vector<cv::Mat>                                        vIntermediateRGBImages; // type CV_8UC3
        std::vector<cv::Mat>                                        vIntermediateDepthImages; // type CV_16UC1
    };

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadIntermediateCloudsFromSingleSweep(std::string sweepXmlPath, bool verbose=false);

    template <class PointType>
    IntermediateCloudCompleteData<PointType> loadIntermediateCloudsCompleteDataFromSingleSweep(std::string sweepXmlPath, bool verbose=false);

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsFromMultipleSweeps(std::string folderPath, bool verbose=false);

    template <class PointType>
    std::vector<IntermediateCloudCompleteData<PointType>>  loadIntermediateCloudsCompleteDataFromMultipleSweeps(std::string folderPath, bool verbose=false);

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false);

    template <class PointType>
    std::vector<IntermediateCloudCompleteData<PointType>> loadIntermediateCloudsCompleteDataForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false);

    /********************************************** INTERMEDIATE POSITION IMAGES UTILITIES ****************************************************************************************/

    template <class PointType>
    std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages> loadIntermediatePositionImagesFromSingleSweep(std::string sweepXmlPath, bool verbose=false);

    template <class PointType>
    std::vector<std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages>> loadIntermediatePositionImagesFromMultipleSweeps(std::string folderPath, bool verbose=false);

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediatePositionImagesForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false);

    /********************************************** SWEEP XML UTILITIES ****************************************************************************************/
    template <class PointType>
    std::vector<std::string>  getSweepXmls(std::string folderPath, bool verbose = false);

    template <class PointType>
    std::vector<std::string>  getSweepXmlsForTopologicalWaypoint(std::string folderPath, std::string waypoint, bool verbose = false);

    /********************************************** DYNAMIC CLUSTER UTILITIES ****************************************************************************************/
    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadDynamicClustersFromSingleSweep(std::string sweepXmlPath, bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000);

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersFromMultipleSweeps(std::string folderPath, bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000);

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000);


#include "load_utilities.hpp"

}
