#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/String.h"

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/retrieval_query.h"
#include "quasimodo_msgs/retrieval_result.h"
#include "soma2_msgs/SOMA2Object.h"

#include "soma_manager/SOMA2InsertObjs.h"

//#include "modelupdater/ModelUpdater.h"
//#include "/home/johane/catkin_ws_dyn/src/quasimodo_models/include/modelupdater/ModelUpdater.h"
#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <map>

#include "ModelDatabase/ModelDatabase.h"

#include <thread>

#include <sys/time.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int main(int argc, char **argv){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(0.9,0.9,0.9);

	std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clouds;
	std::vector<Eigen::Matrix4d> relativeposes;
	for(int arg = 1; arg < argc; arg++){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (argv[arg], *cloud) == -1){
			printf ("Error: Couldn't read file %s\n",argv[arg]);
			return (-1);
		}
		clouds.push_back(cloud);
		relativeposes.push_back(Eigen::Matrix4d::Identity());
	}


	reglib::MassRegistrationPPR * massreg = new reglib::MassRegistrationPPR(0.1);
	massreg->timeout = 60;
	massreg->viewer = viewer;
	massreg->visualizationLvl = 1;

	massreg->setData(clouds);

	massreg->stopval = 0.001;
	massreg->steps = 10;

	reglib::MassFusionResults mfr = massreg->getTransforms(relativeposes);

	return 0;
}
