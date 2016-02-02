#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>

#include <sensor_msgs/PointCloud2.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include <tf_conversions/tf_eigen.h>
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"

#include <cv_bridge/cv_bridge.h>

#include <metaroom_xml_parser/load_utilities.h>
#include "metaroom_xml_parser/load_utilities.h"
#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "ModelDatabase/ModelDatabase.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;



int main(int argc, char** argv){

	ModelDatabase * db = new ModelDatabaseBasic();
	
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	string data_path = "/media/johane/SSDstorage/KTH_longterm_dataset_labels";
	if(argc == 2){data_path = argv[2];}
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);

    for (const string& sweep_xml : folder_xmls) {
		printf("doing sweep\n");
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
		printf("sweep_xml:%s\n",sweep_xml.c_str());

		for (int i = 0; i < labels.objectLabels.size(); ++i) {
			
			if(labels.objectLabels[i].compare("chair1") == 0 || labels.objectLabels[i].compare("chair2") == 0 ){
			
				boost::shared_ptr<pcl::PointCloud<PointT>> cloud = labels.objectClouds[i];
							
				pcl::NormalEstimation<PointT, pcl::Normal> ne;
				ne.setInputCloud (cloud);
				
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
				ne.setSearchMethod (tree);

				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
				ne.setRadiusSearch (0.03);
				ne.compute (*cloud_normals);
				
				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
				cloud2->points.resize(cloud->points.size());
				
				for(unsigned int ind = 0; ind < cloud->points.size(); ind++){
					cloud2->points[ind].x = cloud->points[ind].x;
					cloud2->points[ind].y = cloud->points[ind].y;
					cloud2->points[ind].z = cloud->points[ind].z;
					cloud2->points[ind].r = cloud->points[ind].r;
					cloud2->points[ind].g = cloud->points[ind].g;
					cloud2->points[ind].b = cloud->points[ind].b;
					cloud2->points[ind].normal_x = cloud_normals->points[ind].normal_x;
					cloud2->points[ind].normal_y = cloud_normals->points[ind].normal_y;
					cloud2->points[ind].normal_z = cloud_normals->points[ind].normal_z;
				}
				
				clouds.push_back(cloud);
				int id = db->add(cloud2);
				std::vector<int> res = db->search(id,5);
				printf("nr_matches: %i \n",res.size());
				
				//viewer->removeAllPointClouds();
				//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
				//viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
				//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
				//viewer->spin();
				/*
				for(unsigned int ind = 0; ind < res.size(); ind++){
					printf("result: %i %i\n",clouds.size(),ind);
					viewer->removeAllPointClouds();
					pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(clouds[res[ind]]);
					viewer->addPointCloud<pcl::PointXYZRGB> (clouds[res[ind]], rgb, "cloud");
					viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
					viewer->spin();
				}
				*/

			}
		}

    }

}
