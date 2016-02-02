#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"
/*
#include <modelupdater/ModelUpdater.h>
#include <core/RGBDFrame.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include "simple_xml_parser.h"
#include "simple_summary_parser.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <map>

std::map<int , reglib::Camera *>		cameras;
std::map<int , reglib::RGBDFrame *>		frames;
std::map<int , reglib::Model *>			models;
std::map<int , reglib::ModelUpdater *>	updaters;
reglib::RegistrationPPR *				registration;
*/
using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

int main(int argc, char** argv)
{

	//Wait for data...
	//New data in:
		//while true
		//Search for more of the same <-- Nils stuff, Rares stuff, other?
		//if the same is found
			//Register
			//Fuse
		//else
			//Break


    ros::init(argc, argv, "quasimodo_test_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    string data_path;
    pn.param<std::string>("data_path", data_path, std::string(""));
	data_path = "/media/johane/SSDstorage/KTH_longterm_dataset_labels";
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);



    for (const string& sweep_xml : folder_xmls) {
		printf("doing sweep\n");
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
		//auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_xml, std::vector<std::string>{"RoomCompleteCloud", "RoomIntermediateCloud"},false, false);
		printf("sweep_xml:%s\n",sweep_xml.c_str());
		for (int i = 0; i < labels.objectLabels.size(); ++i) {
			//printf("doing label: %s\n",labels.objectLabels[i].c_str());
			if(labels.objectLabels[i].compare("chair1") == 0){
				printf("doing label: %s\n",labels.objectLabels[i].c_str());
				unsigned int frame_indice = labels.objectScanIndices[i];
				//printf("frame_indice: %i\n",frame_indice);
				char buff [1024];
				sprintf(buff,"%srgb_%04i.jpg",sweep_xml.substr(0,sweep_xml.size()-8).c_str(),frame_indice);
				string rgbpath = string(buff);
				sprintf(buff,"%sdepth_%04i.png",sweep_xml.substr(0,sweep_xml.size()-8).c_str(),frame_indice);
				string depthpath = string(buff);

				cv::Mat maskimage	= labels.objectMasks[i];
				cv::Mat rgbimage	= cv::imread(rgbpath.c_str(),	-1);
				cv::Mat depthimage	= cv::imread(depthpath.c_str(),	-1);

				cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
				cv::imshow(		"maskimage",	maskimage);
				cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );// Create a window for display.
				cv::imshow(		"rgbimage",		rgbimage );                   // Show our image inside it.
				cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );// Create a window for display.
				cv::imshow(		"depthimage",	depthimage );                   // Show our image inside it.
				cv::waitKey(30);                                          // Wait for a keystroke in the window
				//printf("depthpath: %s\n",depthpath.c_str());

			}
		}
/*
        semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

        geometry_msgs::Transform T;
        tf::transformTFToMsg(labels.transformToGlobal, T);
        image_geometry::PinholeCameraModel cm = data.vIntermediateRoomCloudCamParams[0];

        for (int i = 0; i < labels.objectClouds.size(); ++i) {
			printf("doing label: %s\n",labels.objectLabels[i].c_str());
            quasimodo_msgs::query_cloud srv;

            pcl::toROSMsg(*labels.objectClouds[i], srv.request.cloud);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            labels.objectImages[i].copyTo(cv_ptr->image);
            cv_ptr->encoding = "bgr8";
            srv.request.image = *cv_ptr->toImageMsg();

            srv.request.camera = cm.cameraInfo();

            ros::ServiceClient client = n.serviceClient<quasimodo_msgs::query_cloud>("quasimodo_retrieval_service");
            if (!client.call(srv)) {
                ROS_ERROR("Failed to call service quasimodo_retrieval_service");
                return -1;
            }

            quasimodo_msgs::visualize_query vis_srv;

            cv::Mat inverted_mask;
            cv::bitwise_not(labels.objectMasks[i], inverted_mask);
            cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
            cv_pub_ptr->image = labels.objectImages[i];
            cv_pub_ptr->image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
            cv_pub_ptr->encoding = "bgr8";
            vis_srv.request.image = *cv_pub_ptr->toImageMsg();

            vis_srv.request.camera = cm.cameraInfo();
            vis_srv.request.room_transform = T;
            vis_srv.request.result = srv.response.result;

            ros::ServiceClient vis_client = n.serviceClient<quasimodo_msgs::visualize_query>("quasimodo_visualization_service");
            if (!vis_client.call(vis_srv)) {
                ROS_ERROR("Failed to call service quasimodo_visualization_service");
                return -1;
            }
        }
*/
    }

    return 0;
}

/*
#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

int main(int argc, char** argv)
{
	string waypId = "WayPoint16"; // the only one for which there is labelled data
	bool visualize = true; // change this if you want
	string dataPath = "";

	if (argc == 3)
	{
		dataPath = argv[1];
		waypId = argv[2];
	} else {
		cout<<"Please provide data path and waypoint ID as arguments"<<endl;
		return -1;
	}

	pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
	p->addCoordinateSystem();

	vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, waypId);
	ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

	for (size_t i=0; i<matchingObservations.size();i++)
	{
		semantic_map_load_utilties::LabelledData<PointType> data = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointType>(matchingObservations[i], false);

		if (data.objectClouds.size() == 0) continue; // no labelled objects

		// To transform to the map frame of reference:
		//        static tf::StampedTransform world_transform = data.transformToGlobal;
		//        pcl_ros::transformPointCloud(*data.completeCloud, *data.completeCloud,world_transform);
		//        for (auto object: data.labelledObjects)
		//        {
		//            pcl_ros::transformPointCloud(*(object->m_points), *(object->m_points),world_transform);
		//        }


		if (visualize)
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (data.completeCloud, 255, 0, 0);
			p->addPointCloud (data.completeCloud,cloud_handler,"complete_cloud");
		}

		cout<<"Now looking at "<<matchingObservations[i]<<"  acquisition time "<<data.sweepTime<<"   labelled objects  "<<data.objectClouds.size()<<endl;
		for ( size_t j=0; j<data.objectClouds.size(); j++ )
		{
			CloudPtr object = data.objectClouds[j];
			string label = data.objectLabels[j];
			cout<<"Labelled object "<<j<<"  points "<<object->points.size()<<"  label  "<<label<<endl;
			if (visualize)
			{
				stringstream ss;ss<<"object"<<j;
				p->addPointCloud(object,ss.str());
			}
		}

		if (visualize)
		{
			p->spin();
			p->removeAllPointClouds();
		}

	}
}*/
