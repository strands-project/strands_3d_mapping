#include "registration/RegistrationSICP.h"
#include "registration/ICP.h"

namespace reglib
{

RegistrationSICP::RegistrationSICP(){}
RegistrationSICP::~RegistrationSICP(){}

FusionResults RegistrationSICP::getTransform(Eigen::MatrixXd guess){
	Eigen::Matrix<double, 3, Eigen::Dynamic> X;// = src->data;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Y;// = dst->data;
	Eigen::Matrix<double, 3, Eigen::Dynamic> N;// = dst->data;
	
	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);

	unsigned int s_nr_data = src->data.cols();
    X.resize(Eigen::NoChange,s_nr_data);
	for(unsigned int i = 0; i < s_nr_data; i++){
		float x = src->data(0,i);
		float y = src->data(1,i);
		float z = src->data(2,i);
		X(0,i) = m00*x + m01*y + m02*z + m03;
		X(1,i) = m10*x + m11*y + m12*z + m13;
		X(2,i) = m20*x + m21*y + m22*z + m23;
	}
	
	unsigned int d_nr_data = dst->data.cols();
	Y.resize(Eigen::NoChange,d_nr_data);
	N.resize(Eigen::NoChange,d_nr_data);
	for(unsigned int i = 0; i < d_nr_data; i++){
		Y(0,i)	= dst->data(0,i);
		Y(1,i)	= dst->data(1,i);
		Y(2,i)	= dst->data(2,i);
		N(0,i)	= dst->normals(0,i);
		N(1,i)	= dst->normals(1,i);
		N(2,i)	= dst->normals(2,i);
	}

	SICP::Parameters pars;
	pars.p			= 0.5;
	pars.max_icp	= 5;
	pars.max_inner	= 1;
	pars.max_outer	= 30;
	pars.stop		= 0.0001;
	pars.print_icpn	= true;
	Eigen::Matrix4d t = SICP::point_to_plane(X,Y,N,pars);
	
	pcl::TransformationFromCorrespondences tfc;
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	Eigen::Matrix4d np = tfc.getTransformation().matrix().cast<double>();
	return FusionResults(np,0);

/*	
	ICP::Parameters ipars;
    ipars.f			= ICP::PNORM;
    ipars.p			= 0.5;
    //ipars.max_icp	= 10;
	//ipars.max_outer	= 100;
	//ipars.stop		= 0.0001;
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	scloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){
		pcl::PointXYZRGBNormal p;p.x = s(0,i);p.y = s(1,i);p.z = s(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);
	}
	dcloud->points.clear();
	for(unsigned int i = 0; i < d_nr_data; i++){
		pcl::PointXYZRGBNormal p;p.x = d(0,i);p.y = d(1,i);p.z = d(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);
	}		
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();

	for(int k = 0;true;k++){
		Eigen::Matrix4d t = SICP::point_to_plane(s,d,dn,pars);
		//Eigen::Matrix4d t = ICP::point_to_plane(s, d, dn,ipars);//, pars);
		scloud->points.clear();
		for(unsigned int i = 0; i < s_nr_data; i++){
			pcl::PointXYZRGBNormal p;p.x = s(0,i);p.y = s(1,i);p.z = s(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);
		}
		dcloud->points.clear();
		for(unsigned int i = 0; i < d_nr_data; i++){
			pcl::PointXYZRGBNormal p;p.x = d(0,i);p.y = d(1,i);p.z = d(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);
		}
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "scloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "dcloud");
		printf("%i\n",k);
		if(k % 3000 == 0){	viewer->spin();}
		else{			viewer->spinOnce();}
		viewer->removeAllPointClouds();
	}


	//SICP::print();

*/
/*
	float m00 = np(0,0); float m01 = np(0,1); float m02 = np(0,2); float m03 = np(0,3);
	float m10 = np(1,0); float m11 = np(1,1); float m12 = np(1,2); float m13 = np(1,3);
	float m20 = np(2,0); float m21 = np(2,1); float m22 = np(2,2); float m23 = np(2,3);

	dcloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){
		pcl::PointXYZRGBNormal p;
		float x = s(0,i);	float y = s(1,i);	float z = s(2,i);
		p.x = x+0.0001;	p.y = y;	p.z = z;
		p.b = 255;		p.g = 255;	p.r = 0;	dcloud->points.push_back(p);
	}

	scloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){
		pcl::PointXYZRGBNormal p;
		float x = src->data(0,i);	float y = src->data(1,i);	float z = src->data(2,i);
		p.x = m00*x + m01*y + m02*z + m03;	p.y = m10*x + m11*y + m12*z + m13;	p.z = m20*x + m21*y + m22*z + m23;
		p.b = 0;	p.g = 0;	p.r = 255;	scloud->points.push_back(p);
	}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();
	
	exit(0);

	return guess;
*/
}

}
