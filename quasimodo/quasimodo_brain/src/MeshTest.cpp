/*
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

#include "metaroom_xml_parser/load_utilities.h"
//#include "metaroom_xml_parser/simple_summary_parser.h"



#include <dirent.h>


using namespace std;

using PointT = pcl::PointXYZRGB;missunderstoo
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
*/
#include "ros/ros.h"
#include <string.h>

#include "../../quasimodo_models/include/modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include "../../quasimodo_models/include/mesher/Mesh.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

pcl::visualization::PCLVisualizer* viewer;


int main(int argc, char** argv){


    viewer = new pcl::visualization::PCLVisualizer (argc, argv, "MeshTest");
    viewer->addCoordinateSystem();
    viewer->setBackgroundColor(1, 1, 1);

    reglib::Camera * camera = new reglib::Camera();
	for(int ar = 1; ar < argc; ar++){
        string path = std::string(argv[ar]);
        pcl::PCDReader reader;
        boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
        reader.read (path, *cloud);

        cloud->width = 640;
        cloud->height = 480;

        cv::Mat mask;
        mask.create(cloud->height,cloud->width,CV_8UC1);
        unsigned char * maskdata = (unsigned char *)mask.data;

        cv::Mat rgb;
        rgb.create(cloud->height,cloud->width,CV_8UC3);
        unsigned char * rgbdata = (unsigned char *)rgb.data;

        cv::Mat depth;
        depth.create(cloud->height,cloud->width,CV_16UC1);
        unsigned short * depthdata = (unsigned short *)depth.data;

        unsigned int nr_data = cloud->height * cloud->width;
        for(unsigned int j = 0; j < nr_data; j++){
            maskdata[j] = 255;
            PointType p = cloud->points[j];
            rgbdata[3*j+0]	= p.b;
            rgbdata[3*j+1]	= p.g;
            rgbdata[3*j+2]	= p.r;
            depthdata[j]	= short(5000.0 * p.z);
        }

//        cv::namedWindow("rgbimage",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"rgbimage",	rgb);
//        cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"depthimage",	depth);
//        cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"mask",	mask);
//        cv::waitKey(0);

        reglib::RGBDFrame * frame   = new reglib::RGBDFrame(camera,rgb, depth);
        reglib::Model * model       = new reglib::Model(frame,mask);
        reglib::Mesh  * mesh        = new reglib::Mesh();
        mesh->build(model,0);
        mesh->show(viewer);

    }
}
