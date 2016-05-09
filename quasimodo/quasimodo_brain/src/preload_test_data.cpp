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


using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

int main(int argc, char** argv){

	string data_path = "/media/johane/SSDstorage/icradata/strands.pdc.kth.se/public/ICRA-2016-Data/controlled_experiments/object_1_cooler_box";
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);

	std::vector<cv::Mat> rgbs;
	std::vector<cv::Mat> depths;
	std::vector<cv::Mat> masks;
    for (const string& sweep_xml : folder_xmls) {
		printf("doing sweep\n");
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
		printf("sweep_xml:%s\n",sweep_xml.c_str());
		for (int i = 0; i < labels.objectLabels.size(); ++i) {
			if(true || labels.objectLabels[i].compare("chair1") == 0){
				printf("doing label: %s\n",labels.objectLabels[i].c_str());
				unsigned int frame_indice = labels.objectScanIndices[i];
				char buff [1024];
				sprintf(buff,"%srgb_%04i.jpg",sweep_xml.substr(0,sweep_xml.size()-8).c_str(),frame_indice);
				string rgbpath = string(buff);
				sprintf(buff,"%sdepth_%04i.png",sweep_xml.substr(0,sweep_xml.size()-8).c_str(),frame_indice);
				string depthpath = string(buff);

				cv::Mat maskimage	= labels.objectMasks[i];
				cv::Mat rgbimage	= cv::imread(rgbpath.c_str(),	-1);
				cv::Mat depthimage	= cv::imread(depthpath.c_str(),	-1);

				rgbs.push_back(rgbimage);
				depths.push_back(depthimage);
				masks.push_back(maskimage);
				
				cv::Mat rgbimage_flip;       	
				cv::flip(rgbimage, rgbimage_flip, 1);
				cv::Mat depthimage_flip;       	
				cv::flip(depthimage, depthimage_flip, 1);
				cv::Mat maskimage_flip;       	
				cv::flip(maskimage, maskimage_flip, 1);
				
				rgbs.push_back(rgbimage_flip);
				depths.push_back(depthimage_flip);
				masks.push_back(maskimage_flip);
				
				//cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
				//cv::imshow(		"maskimage",	maskimage);
				cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
				cv::imshow(		"rgbimage",		rgbimage );
				//cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
				//cv::imshow(		"depthimage",	depthimage );
				cv::waitKey(0);
			}
		}
		if(rgbs.size() >= 160){break;}
		printf("rgbs: %i\n",rgbs.size());
    }
    
	ros::init(argc, argv, "use_rares_client");
	ros::NodeHandle n;
	ros::ServiceClient model_from_frame_client	= n.serviceClient<quasimodo_msgs::model_from_frame>("model_from_frame");
	ros::ServiceClient fuse_models_client		= n.serviceClient<quasimodo_msgs::fuse_models>(		"fuse_models");
	ros::ServiceClient get_model_client			= n.serviceClient<quasimodo_msgs::get_model>(		"get_model");
	ros::ServiceClient index_frame_client		= n.serviceClient<quasimodo_msgs::index_frame>(		"index_frame");

	
    while(true){
        char str [80];
        printf ("build model?");
        scanf ("%79s",str);
		int nr_todo = atoi(str);
		if(str[0] == 'q'){exit(0);}
		
		std::vector<cv::Mat> current_rgbs;
		std::vector<cv::Mat> current_depths;
		std::vector<cv::Mat> current_masks;
		
		for (unsigned int i = 0; i < nr_todo && i < rgbs.size(); i++){
        	printf("adding frame %i\n",i);
        	
        	//int id = rand()%rgbs.size();
        	int id = (i)%rgbs.size();
        	
        	cv::Mat maskimage	= masks[id];
			cv::Mat rgbimage	= rgbs[id];
			cv::Mat depthimage	= depths[id];
			
			
			char buff [1024];
			sprintf(buff,"rgbimage");
			
			//cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
			//cv::imshow(		"maskimage",	maskimage);
			cv::namedWindow(buff,		cv::WINDOW_AUTOSIZE );
			cv::imshow(		buff,		rgbimage );
			//cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
			//cv::imshow(		"depthimage",	depthimage );
			unsigned char retval = cv::waitKey(0);
			printf("retval: %i\n",retval);
			if(retval != 27){
				current_rgbs.push_back(rgbimage);
				current_depths.push_back(depthimage);
				current_masks.push_back(maskimage);
			}
		}
		
		
        for (unsigned int i = 0; i < current_rgbs.size(); i++){
        	printf("start adding frame %i\n",i);
        	
        	cv::Mat maskimage	= current_masks[i];
			cv::Mat rgbimage	= current_rgbs[i];
			cv::Mat depthimage	= current_depths[i];

			geometry_msgs::Pose		pose;
			tf::poseEigenToMsg (Eigen::Affine3d::Identity(), pose);

            cv_bridge::CvImage rgbBridgeImage;
            rgbBridgeImage.image = rgbimage;
            rgbBridgeImage.encoding = "bgr8";

            cv_bridge::CvImage depthBridgeImage;
            depthBridgeImage.image = depthimage;
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

        	printf("stop adding frame %i\n",i);
        }

		for (unsigned int i = 0; i < nr_todo && i < current_rgbs.size() ; i++){
			printf("start adding mask %i\n",i);
			cv::Mat mask	= current_masks[i];

			cv_bridge::CvImage maskBridgeImage;
			maskBridgeImage.image = mask;
			maskBridgeImage.encoding = "mono8";

			quasimodo_msgs::model_from_frame mff;
			mff.request.mask = *(maskBridgeImage.toImageMsg());
			mff.request.frame_id = i;

            if (model_from_frame_client.call(mff)){//Build model from frame
                int model_id = mff.response.model_id;
                if(model_id > 0){
                    ROS_INFO("model_id%i", model_id );
                }
            }else{ROS_ERROR("Failed to call service index_frame");}

			printf("stop adding mask %i\n",i);
        }

    }
}
