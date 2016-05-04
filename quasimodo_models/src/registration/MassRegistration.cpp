#include "registration/MassRegistration.h"

namespace reglib
{

MassRegistration::MassRegistration(){
	visualizationLvl = 0;
	nomask = true;
	maskstep = 1;
	nomaskstep = 100000;
    timeout = 60;//1 minute timeout
}
MassRegistration::~MassRegistration(){}

void MassRegistration::setData(std::vector<RGBDFrame*> frames_,std::vector<ModelMask *> mmasks_){
	frames = frames_;
	mmasks = mmasks_;
}

void MassRegistration::setData(std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > all_clouds){}

void MassRegistration::setVisualizationLvl(unsigned int lvl){
	visualizationLvl = lvl;
}

MassFusionResults MassRegistration::getTransforms(std::vector<Eigen::Matrix4d> guess){
	return MassFusionResults(guess,0);
}

void MassRegistration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y){

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();

//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);

	scloud->points.clear();
	dcloud->points.clear();

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}
	//printf("nr datas: %i %i\n",scloud->points.size(),dcloud->points.size());

	for(unsigned int i = 0; i < s_nr_data; i++){
		//if(i%100 == 0){printf("x: %f %f %f\n",X(0,i),X(1,i),X(2,i));}
	}

	for(unsigned int i = 0; i < d_nr_data; i++){
		//if(i%100 == 0){printf("y: %f %f %f\n",Y(0,i),Y(1,i),Y(2,i));}
	}

viewer->removeAllPointClouds();
viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
viewer->spin();

//viewer->removeAllPointClouds();

//	viewer->removeAllPointClouds();
//	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
//	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
//	viewer->spin();
//	viewer->removeAllPointClouds();
}

Eigen::MatrixXd MassRegistration::getMat(int rows, int cols, double * datas){
	Eigen::MatrixXd mat (rows,cols);
	for(int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){
			mat(i,j) = i;//datas[cols*i+j];
		}
	}
	return mat;
}


void MassRegistration::show(std::vector<Eigen::MatrixXd> Xv, bool save, std::string filename, bool stop){
	viewer->removeAllPointClouds();

	srand(0);
    for(unsigned int xi = 0; xi < Xv.size(); xi++){
		Eigen::MatrixXd X = Xv[xi];
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		int r = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		int g = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		int b = 256*(1+(rand()%4))/4 - 1;//255*(xi & 1);

		unsigned int nr_data = X.cols();
		cloud->points.clear();
		for(unsigned int i = 0; i < nr_data; i++){
			pcl::PointXYZRGBNormal p;
			p.x = X(0,i);
			p.y = X(1,i);
			p.z = X(2,i);
			p.b = r;
			p.g = g;
			p.r = b;
			cloud->points.push_back(p);
		}
		char buf [1024];
		sprintf(buf,"cloud%i",xi);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf);
	}
	if(!save){
		viewer->spin();
	}else{
		viewer->spinOnce();
	}
	//void pcl::visualization::PCLVisualizerInteractorStyle::saveScreenshot	(	const std::string & 	file	)
	if(save){
		printf("saving: %s\n",filename.c_str());
		viewer->saveScreenshot(filename);
	}
	viewer->removeAllPointClouds();
}
/*
void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y){

	//printf("start show\n");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();
	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}		
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	//printf("pre\n");
    viewer->spin();
	//printf("post\n");
	viewer->removeAllPointClouds();

	//printf("stop show\n");
}

void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Xn, Eigen::MatrixXd Y, Eigen::MatrixXd Yn){

	//printf("start show\n");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::PointCloud<pcl::Normal>::Ptr sNcloud (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr dNcloud (new pcl::PointCloud<pcl::Normal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();
	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();

	sNcloud->points.clear();
	dNcloud->points.clear();

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::Normal p;p.normal_x = Xn(0,i);p.normal_y = Xn(1,i);p.normal_z = Xn(2,i);sNcloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::Normal p;p.normal_x = Yn(0,i);p.normal_y = Yn(1,i);p.normal_z = Yn(2,i);dNcloud->points.push_back(p);}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (scloud, sNcloud, 100, 0.2, "sNcloud");

	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (dcloud, dNcloud, 100, 0.2, "dNcloud");
	//printf("pre\n");
	viewer->spin();
	//printf("post\n");
	viewer->removeAllPointClouds();

	//printf("stop show\n");
}

void Registration::setVisualizationLvl(unsigned int lvl){visualizationLvl = lvl;}

void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y, Eigen::VectorXd W){
	show(X,Y);
	double mw = W.maxCoeff();
	W = W/mw;
	//std::cout << W << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();

	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();
	//for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	//for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}
	for(unsigned int i = 0; i < s_nr_data; i++){
		pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 255*W(i);	p.g = 255*W(i);	p.r = 255*W(i);	scloud->points.push_back(p);
		//if(W(i) > 0.001){	pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;	p.r = 0;	scloud->points.push_back(p);}
		//else{				pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 0;	p.r = 255;	scloud->points.push_back(p);}
	}
	for(unsigned int i = 0; i < d_nr_data; i+=1){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 255;p.g = 0;p.r = 0;dcloud->points.push_back(p);}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	//printf("pre spin\n");
    viewer->spin();
	//printf("post spin\n");
	viewer->removeAllPointClouds();
}
*/
}
