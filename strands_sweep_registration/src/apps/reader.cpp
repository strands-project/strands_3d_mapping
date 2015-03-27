#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string.h>

#include "Frame.h"
#include "Camera.h"
#include "Sweep.h"

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"

#include "simple_xml_parser.h"//../../scitos_3d_mapping/metaroom_xml_parser/include/
#include "simple_summary_parser.h"
//#include "load_utilities.h"

#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transformation_from_correspondences.h>

#include "tf_conversions/tf_eigen.h"

#include <semantic_map/room_xml_parser.h>
#include <semantic_map/reg_transforms.h>
#include <pcl_ros/transforms.h>

#include "RobotContainer.h"
/*
pcl::PointCloud<PointType>::Ptr rebuildRegisteredCloud(std::string room_xml){
    cout<<"Rebuilding registered cloud for "<<room_xml<<endl;
    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(room_xml,true);
    parser.setRootFolderFromRoomXml(room_xml);

    std::vector<tf::StampedTransform> cloudTransformsReg = aRoom.getIntermediateCloudTransformsRegistered();
    std::vector<pcl::PointCloud<PointType>::Ptr> clouds= aRoom.getIntermediateClouds();

    pcl::PointCloud<PointType>::Ptr mergedCloudRegistered(new pcl::PointCloud<PointType>);
    if (cloudTransformsReg.size() == clouds.size()){
        for (size_t j=0; j<clouds.size(); j++){
            pcl::PointCloud<PointType> transformed_cloud;
            pcl_ros::transformPointCloud(*clouds[j], transformed_cloud,cloudTransformsReg[j]);
            *mergedCloudRegistered+=transformed_cloud;
        }
        //aRoom.setCompleteRoomCloud(mergedCloudRegistered);
        //parser.saveRoomAsXML(aRoom);
    } else {
        cout<<"Cannot build registered cloud, the registered intermediate cloud transforms have not been set "<<endl;
    }
	return mergedCloudRegistered;
}

*/
typedef pcl::PointXYZRGB PointType;
template <class PointType> std::vector<std::string> getSweepXmlsForTopologicalWaypoint(std::string folderPath, std::string waypoint, bool verbose= false){
    SimpleSummaryParser summary_parser;
    summary_parser.createSummaryXML(folderPath);
    auto sweep_xmls = summary_parser.getRooms();

    std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
    for (size_t i=0; i<sweep_xmls.size(); i++){
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
        waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
    }
    return waypointToSweepsMap[waypoint];
}


int main(int argc, char **argv){

	unsigned int gx = 17;
	unsigned int todox = 17;
	unsigned int gy = 3;
	unsigned int todoy = 3;

    unsigned int start_sweep = 0;
    unsigned int stop_sweep = 5;

    unsigned int sweeps_for_training = 1000;

	std::string folderPath;
	if (argc < 2){
        folderPath = "/home/rares/Data/Original_data/";
	}else{
		folderPath = argv[1];
	}

    SimpleSummaryParser summary_parser(folderPath + "/index.xml");
    summary_parser.createSummaryXML(folderPath);

 
    std::vector<std::string> allSweeps;

    if(false){
		auto sweep_xmls = summary_parser.getRooms();
		for (size_t i=0; i<sweep_xmls.size(); i++){
		    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), false);
			printf("%s\n",sweep.roomWaypointId.c_str());
			allSweeps.push_back(sweep_xmls[i].roomXmlFile);
		}
	}else{
        allSweeps = getSweepXmlsForTopologicalWaypoint<PointType>(folderPath, "WayPoint16");
	}

    std::sort(allSweeps.begin(), allSweeps.end());

    unsigned int nr_sweeps = allSweeps.size();

	RobotContainer * rc = new RobotContainer(gx,todox,gy,todoy);
    rc->initializeCamera(540.0, 540.0,319.5, 219.5, 640, 480);

	for (unsigned int i = 0; i < nr_sweeps; i++){
		if(i < start_sweep || i > stop_sweep){continue;}
//        rc->addToTraining(allSweeps[i]);
        rc->addToTrainingORBFeatures(allSweeps[i]);
	}
    std::vector<Eigen::Matrix4f> cameraPoses = rc->train();
    std::vector<tf::StampedTransform> registeredPoses;

    for (auto eigenPose : cameraPoses)
    {
        tf::StampedTransform tfStamped;
        tfStamped.frame_id_ = "temp";
        tfStamped.child_frame_id_ = "temp";
        tf::Transform tfTr;
        const Eigen::Affine3d eigenTr(eigenPose.cast<double>());
        tf::transformEigenToTF(eigenTr, tfTr);
        tfStamped.setOrigin(tfTr.getOrigin());
        tfStamped.setBasis(tfTr.getBasis());
        registeredPoses.push_back(tfStamped);
    }

    std::string file = semantic_map_registration_transforms::saveRegistrationTransforms(registeredPoses);
    std::vector<tf::StampedTransform> transforms = semantic_map_registration_transforms::loadRegistrationTransforms("default", true);

    double*** rawPoses = rc->poses;
    unsigned int x,y;
    file = semantic_map_registration_transforms::saveRegistrationTransforms(rawPoses, rc->todox,rc->todoy);
    rawPoses = semantic_map_registration_transforms::loadRegistrationTransforms(x,y);

    // delete and re-initialize
//    delete rc;



    rc->alignAndStoreSweeps();
    rc->saveAllSweeps("/home/rares/Data/Test_Registration/");
//	for (unsigned int i = stop_sweep+1; i < nr_sweeps; i++){
//		if(i > 10){continue;}
//		rc->addToTraining(allSweeps[i]);
//	}
//    rc->alignAndStoreSweeps();
	return 0;
}
