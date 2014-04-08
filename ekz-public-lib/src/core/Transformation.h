#ifndef Transformation_H_
#define Transformation_H_
#include <pcl/visualization/cloud_viewer.h>

//other
#include <utility>
#include "RGBDFrame.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//#include "/home/johane/catkin_ws/src/6dslam/g2o/g2o/types/slam3d/se3quat.h"

using namespace std;

class Transformation {
	public:
		RGBDFrame * src;
		RGBDFrame * dst;
		vector< pair <KeyPoint * ,KeyPoint * > > matches;
		vector< pair <Plane * ,Plane * > > plane_matches;
		Eigen::Matrix4f transformationMatrix;
		double weight;
		double time;
		
		Transformation * clone();
		void print();

		IplImage * getMatchImg();
		void saveProblem(string path);

		Eigen::Matrix4f getTrue(){

			Eigen::Matrix4f src_mat = src->input->gt.matrix();
			Eigen::Matrix4f dst_mat = dst->input->gt.matrix();
			Eigen::Matrix4f true_trans_mat = dst_mat.inverse()*src_mat;

			return (dst_mat.inverse())*(src_mat);

		}

		void getError(float & rot, float & trans){
			Eigen::Matrix4f src_mat = src->input->gt.matrix();
			Eigen::Matrix4f dst_mat = dst->input->gt.matrix();
			Eigen::Matrix4f true_mat = src_mat.inverse() * dst_mat;
			Eigen::Matrix4f trans_mat = transformationMatrix.inverse();
			Eigen::Matrix4f gt = true_mat * trans_mat.inverse();
			rot = 0;
			trans = 0;
			for(int i = 0; i < 3 ; i++){
				for(int j = 0; j < 3 ; j++){
					if(i == j){	rot+=(1-gt(i,j))*(1-gt(i,j));}
					else{		rot+=gt(i,j)*gt(i,j);}
				}
				trans+=gt(i,3)*gt(i,3);
			}
			trans = sqrt(trans);
		}

		void printError(){
			float rot_err, trans_err;
			getError(rot_err, trans_err);
			printf("rot_err %f trans_err %f\n",rot_err,trans_err);
		}

		void show(bool pcd);
		void show(pcl::visualization::CloudViewer * viewer);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
