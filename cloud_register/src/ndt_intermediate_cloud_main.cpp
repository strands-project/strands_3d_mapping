#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ndt.h"
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "ndtRegistration.h"

#include "metaroom.h"
#include "roomXMLparser.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;
int
main (int argc, char** argv)
{

    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML("/home/rares/.semanticMap/20131206/patrol_run_6/room_0/room.xml",false);
    vector<string> allCloudsFilenames = aRoom.getIntermediateCloudsFilenames();

//    int noClouds = 8;
    int noClouds = allCloudsFilenames.size();

    pcl::PointCloud<PointType>::Ptr accumulatedCloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr simpleCloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr tempCloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    std::vector<pcl::PointCloud<PointType>::Ptr> allClouds;

    pcl::io::loadPCDFile<PointType> (allCloudsFilenames[0], *cloud);
    *accumulatedCloud += *cloud; // add the first cloud (nothing to register it to)
    *simpleCloud += *cloud;

    *tempCloud = *cloud;
    allClouds.push_back(tempCloud);

    for (int i=1; i<4; i++)
    {
        ROS_INFO_STREAM("Loading intermediate cloud filename "<<allCloudsFilenames[i]);
        pcl::PointCloud<PointType>::Ptr intcloud (new pcl::PointCloud<PointType>);
        pcl::io::loadPCDFile<PointType> (allCloudsFilenames[i], *intcloud);
        *simpleCloud += *intcloud;
        // register with NDT
        Eigen::Matrix4f finalTransformation;
        pcl::PointCloud<PointType>::Ptr output_cloud = NdtRegistration<PointType>::registerClouds(intcloud ,cloud,finalTransformation);
        *accumulatedCloud += *output_cloud;
        ROS_INFO_STREAM("Registered cloud "<<allCloudsFilenames[i]<<". Transformation "<<finalTransformation);
        *cloud = *intcloud;

        pcl::PointCloud<PointType>::Ptr tempCloud2 (new pcl::PointCloud<PointType>);
        *cloud = *intcloud;
        *tempCloud2 = *intcloud;
        allClouds.push_back(tempCloud2);
    }



    for (int i=4; i<noClouds; i++)
    {
        ROS_INFO_STREAM("Loading intermediate cloud filename "<<allCloudsFilenames[i]);
        pcl::PointCloud<PointType>::Ptr intcloud (new pcl::PointCloud<PointType>);
        pcl::io::loadPCDFile<PointType> (allCloudsFilenames[i], *intcloud);
        *simpleCloud += *intcloud;
        // register with NDT
        Eigen::Matrix4f finalTransformation;
        pcl::PointCloud<PointType>::Ptr output_cloud = NdtRegistration<PointType>::registerClouds(intcloud ,allClouds[i-4],finalTransformation);
        *accumulatedCloud += *output_cloud;
        ROS_INFO_STREAM("Registered cloud "<<allCloudsFilenames[i]<<". Transformation "<<finalTransformation);

        pcl::PointCloud<PointType>::Ptr tempCloud2 (new pcl::PointCloud<PointType>);
        *tempCloud2 = *intcloud;
        allClouds.push_back(tempCloud2);

    }

    ROS_INFO_STREAM("Downsampling final cloud");

    pcl::PointCloud<PointType>::Ptr downsampledCloud (new pcl::PointCloud<PointType>);

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud (accumulatedCloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*downsampledCloud);

    ROS_INFO_STREAM("Saving final cloud");
    // save for debugging
    {
        std::stringstream ss;
        ss << "final";
        ss <<".pcd";
        pcl::io::savePCDFile (ss.str(), *downsampledCloud, true);
    }

    vg.setInputCloud (simpleCloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*downsampledCloud);

    {
        std::stringstream ss;
        ss << "simple";
        ss <<".pcd";
        pcl::io::savePCDFile (ss.str(), *downsampledCloud, true);
    }

}
