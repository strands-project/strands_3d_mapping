#include "ros/ros.h"
#include "std_msgs/String.h"
#include "quasimodo_msgs/model.h"
#include "modelupdater/ModelUpdater.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "read_rares_client");
	ros::NodeHandle n;
	ros::ServiceClient index_frame_client = n.serviceClient<quasimodo_msgs::index_frame>("index_frame");



	unsigned int start_sweep = 1;
	unsigned int stop_sweep = 3000;

	std::string		folderPath;
	if (argc < 2){	folderPath = "/media/johane/SSDstorage/KTH_longterm_dataset_labels";}
	else{			folderPath = argv[1];}

	std::string summaryXMLPath = folderPath + "/index.xml";
	SimpleSummaryParser summary_parser(summaryXMLPath);
	summary_parser.createSummaryXML(folderPath);

std::vector<cv::Mat> rgb_images;
std::vector<cv::Mat> depth_images;
std::vector<Eigen::Affine3d> poses;

    std::vector<SimpleSummaryParser::EntityStruct> allSweeps = summary_parser.getRooms();

	for (unsigned int i = start_sweep+1; i < allSweeps.size(); i+= 1){
		if(i < start_sweep || i > stop_sweep){continue;}
		std::cout << "Parsing " << allSweeps[i].roomXmlFile << std::endl;

		SimpleXMLParser<pcl::PointXYZRGB> parser;
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData	= parser.loadRoomFromXML(allSweeps[i].roomXmlFile);
		if(roomData.vIntermediateRoomClouds.size() == 0){continue;}

		std::vector<tf::StampedTransform> original_transforms	= roomData.vIntermediateRoomCloudTransforms;//Registered;
		std::vector<cv::Mat> rgbimages							= roomData.vIntermediateRGBImages; // type CV_8UC3
		std::vector<cv::Mat> depthimages						= roomData.vIntermediateDepthImages; // type CV_16UC1

		for(unsigned int j = 4; j < 5 && j < rgbimages.size(); j+=1){
			Eigen::Affine3d mat;
			tf::transformTFToEigen(original_transforms[j],mat);
			//Eigen::Matrix4d tr	= mat.matrix();
			cv::Mat rgb			= rgbimages[j];
			cv::Mat depth		= depthimages[j];

            rgb_images.push_back(rgb);
            depth_images.push_back(depth);
            poses.push_back(mat);
		}
	}

    while(true){
        char str [80];
        printf ("add frames?");
        scanf ("%79s",str);
		printf("strating to add\n");
        for(unsigned int i = 0; i < rgb_images.size(); i++){
            geometry_msgs::Pose		pose;
            tf::poseEigenToMsg (poses[i], pose);

            cv_bridge::CvImage rgbBridgeImage;
            rgbBridgeImage.image = rgb_images[i];
            rgbBridgeImage.encoding = "bgr8";

            cv_bridge::CvImage depthBridgeImage;
            depthBridgeImage.image = depth_images[i];
            depthBridgeImage.encoding = "mono16";

            quasimodo_msgs::index_frame ifsrv;
            ifsrv.request.frame.capture_time = ros::Time();
            ifsrv.request.frame.pose		= pose;
            ifsrv.request.frame.frame_id	= -1;
            ifsrv.request.frame.rgb			= *(rgbBridgeImage.toImageMsg());
            ifsrv.request.frame.depth		= *(depthBridgeImage.toImageMsg());

            if (index_frame_client.call(ifsrv)){//Add frame to model server
                int frame_id = ifsrv.response.frame_id;
                ROS_INFO("frame_id%i", frame_id );
             }else{ROS_ERROR("Failed to call service index_frame");}
        }
		printf("done adding\n");
    }
}
