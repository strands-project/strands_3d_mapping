#include "ModelUpdaterBasic.h"

namespace reglib
{

ModelUpdaterBasic::ModelUpdaterBasic(Registration * registration_){
	registration = registration_;
	model = new reglib::Model();
}

ModelUpdaterBasic::ModelUpdaterBasic(Model * model_, Registration * registration_){
printf("new ModelUpdaterBasic() with frame\n");
    registration = registration_;
    model = model_;
printf("ModelUpdaterBasic::line:%i\n",__LINE__);

}

ModelUpdaterBasic::~ModelUpdaterBasic(){}

FusionResults ModelUpdaterBasic::registerModel(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	printf("registerModel\n");
	return FusionResults();
}

void ModelUpdaterBasic::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	//Blind addidtion of new model to old model...

	if(model->frames.size() > 0){
		Model * m1 = 0;
		Model * m2 = 0;
		m1 = model;
		m2 = model2;
/*
		if(model->frames.size() > model2->frames.size()){
			m1 = model;
			m2 = model2;
		}else{
			m1 = model2;
			m2 = model;
		}

		if(model->frames.size() <= model2->frames.size()){
			printf("invert\n");
			Eigen::Matrix4d iguess = guess.inverse();
			guess = iguess;
			printf("inverted\n");
		}
*/
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		std::vector<double> weights1;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		std::vector<double> weights2;
		unsigned int n1 = m1->frames.size();
		unsigned int n2 = m2->frames.size();
        printf("n1: %i n2: %i\n",n1,n2);
		for(unsigned int i = 0; i < n1;i++){
			RGBDFrame * frame	= m1->frames[i];
			cv::Mat		mask	= m1->masks[i]; unsigned char * maskdata = (unsigned char *)mask.data;
			Eigen::Matrix4d	pose = m1->relativeposes[i];

			const double m00 = pose(0,0); const double m01 = pose(0,1); const double m02 = pose(0,2); const double m03 = pose(0,3);
			const double m10 = pose(1,0); const double m11 = pose(1,1); const double m12 = pose(1,2); const double m13 = pose(1,3);
			const double m20 = pose(2,0); const double m21 = pose(2,1); const double m22 = pose(2,2); const double m23 = pose(2,3);

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_cloud = frame->getPCLcloud();

            unsigned int nr_data = source_cloud->points.size();
            //printf("nr_data: %i\n",nr_data);
            //cv::imshow( "rgb", frame->rgb);
            //cv::imshow( "depth", frame->depth);
            //cv::imshow( "mask", mask);
            //cv::waitKey(0);
            for(unsigned int k = 0; k < nr_data;k++){
				pcl::PointXYZRGBNormal p = source_cloud->points[k];
				if(maskdata[k] == 255 && p.z > 0 && !isnan(p.normal_x)){
					float x = p.x;
					float y = p.y;
					float z = p.z;
					float pr = p.r;
					float pg = p.g;
					float pb = p.b;
					float nx = p.normal_x;
					float ny = p.normal_y;
					float nz = p.normal_z;

					pcl::PointXYZRGBNormal p2;
					p2.x = m00*x + m01*y + m02*z + m03;
					p2.y = m10*x + m11*y + m12*z + m13;
					p2.z = m20*x + m21*y + m22*z + m23;
					p2.r = p.r;
					p2.g = p.g;
					p2.b = p.b;
					p2.normal_x = m00*nx + m01*ny + m02*nz;
					p2.normal_y = m10*nx + m11*ny + m12*nz;
					p2.normal_z = m20*nx + m21*ny + m22*nz;
					cloud1->points.push_back(p2);
					weights1.push_back(1.0/(z*z));
				}
			}
		}

		for(unsigned int i = 0; i < n2;i++){
			RGBDFrame * frame	= m2->frames[i];
			cv::Mat		mask	= m2->masks[i]; unsigned char * maskdata = (unsigned char *)mask.data;
			Eigen::Matrix4d	pose = m2->relativeposes[i];

			const double m00 = pose(0,0); const double m01 = pose(0,1); const double m02 = pose(0,2); const double m03 = pose(0,3);
			const double m10 = pose(1,0); const double m11 = pose(1,1); const double m12 = pose(1,2); const double m13 = pose(1,3);
			const double m20 = pose(2,0); const double m21 = pose(2,1); const double m22 = pose(2,2); const double m23 = pose(2,3);


			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_cloud = frame->getPCLcloud();
			unsigned int nr_data = source_cloud->points.size();
			for(unsigned int k = 0; k < nr_data;k++){
				pcl::PointXYZRGBNormal p = source_cloud->points[k];
				if(maskdata[k] == 255 && p.z > 0 && !isnan(p.normal_x)){
					float x = p.x;
					float y = p.y;
					float z = p.z;
					float pr = p.r;
					float pg = p.g;
					float pb = p.b;
					float nx = p.normal_x;
					float ny = p.normal_y;
					float nz = p.normal_z;

					pcl::PointXYZRGBNormal p2;
					p2.x = m00*x + m01*y + m02*z + m03;
					p2.y = m10*x + m11*y + m12*z + m13;
					p2.z = m20*x + m21*y + m22*z + m23;
					p2.r = p.r;
					p2.g = p.g;
					p2.b = p.b;
					p2.normal_x = m00*nx + m01*ny + m02*nz;
					p2.normal_y = m10*nx + m11*ny + m12*nz;
					p2.normal_z = m20*nx + m21*ny + m22*nz;
					cloud2->points.push_back(p2);
					weights2.push_back(1.0/(z*z));
				}
			}
		}

		//Random order
		std::vector<unsigned int> ro1;
		unsigned int nc1 = cloud1->points.size();
		ro1.resize(nc1);
		for(unsigned int i = 0; i < nc1; i++){ro1[i] = i;}
		for(unsigned int i = 0; i < nc1; i++){
			unsigned int rind = rand()%nc1;
			int tmp = ro1[i];
			ro1[i] = ro1[rind];
			ro1[rind] = tmp;
		}

		std::vector<unsigned int> ro2;
		unsigned int nc2 = cloud2->points.size();
		ro2.resize(nc2);
		for(unsigned int i = 0; i < nc2; i++){ro2[i] = i;}
		for(unsigned int i = 0; i < nc2; i++){
			unsigned int rind = rand()%nc2;
			int tmp = ro2[i];
			ro2[i] = ro2[rind];
			ro2[rind] = tmp;
		}

		unsigned int target_points = 500;

		//Build registration input
		unsigned int nr_points1 = std::min(unsigned(cloud1->points.size()),target_points);
		MatrixXd data1 (6,nr_points1);
		MatrixXd data_normals1 (3,nr_points1);
		MatrixXd information1 (3,nr_points1);
		for(unsigned int k = 0; k < nr_points1; k++){
			unsigned int k1 = ro1[k];
			pcl::PointXYZRGBNormal & p	= cloud1->points[k1];
			float w = weights1[k];
			data1(0,k) = p.x;
			data1(1,k) = p.y;
			data1(2,k) = p.z;

			data1(3,k) = p.r;
			data1(4,k) = p.g;
			data1(5,k) = p.b;
			data_normals1(0,k) = p.normal_x;
			data_normals1(1,k) = p.normal_y;
			data_normals1(2,k) = p.normal_z;
			information1(0,k) = w;
			information1(1,k) = w;
			information1(2,k) = w;
		}

		CloudData * cd1				= new CloudData();
		cd1->data					= data1;
		cd1->information			= information1;
		cd1->normals				= data_normals1;

		unsigned int nr_points2 = std::min(unsigned(cloud2->points.size()),target_points);
		MatrixXd data2 (6,nr_points2);
		MatrixXd data_normals2 (3,nr_points2);
		MatrixXd information2 (3,nr_points2);
		for(unsigned int k = 0; k < nr_points1; k++){
			unsigned int k2 = ro2[k];
			pcl::PointXYZRGBNormal & p	= cloud2->points[k2];
			float w = weights2[k];
			data2(0,k) = p.x;
			data2(1,k) = p.y;
			data2(2,k) = p.z;

			data2(3,k) = p.r;
			data2(4,k) = p.g;
			data2(5,k) = p.b;
			data_normals2(0,k) = p.normal_x;
			data_normals2(1,k) = p.normal_y;
			data_normals2(2,k) = p.normal_z;
			information2(0,k) = w;
			information2(1,k) = w;
			information2(2,k) = w;
		}

		CloudData * cd2				= new CloudData();
		cd2->data					= data2;
		cd2->information			= information2;
		cd2->normals				= data_normals2;

		registration->viewer = viewer;
		registration->setSrc(cd1);
		registration->setDst(cd2);
		//FusionResults fr = registration->getTransform(guess).inverse();
		delete cd1;
		delete cd2;
	}

	for(unsigned int i = 0; i < model2->frames.size();i++){
		model->frames.push_back(model2->frames[i]);
		model->masks.push_back(model2->masks[i]);
		model->relativeposes.push_back(guess*model2->relativeposes[i]);
	}
}
void ModelUpdaterBasic::refine(){}//No refinement behaviour added yet

void ModelUpdaterBasic::setRegistration( Registration * registration_){
	if(registration != 0){delete registration;}
	registration = registration;
}

}


