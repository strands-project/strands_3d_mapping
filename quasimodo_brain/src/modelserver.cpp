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

//#include "modelupdater/ModelUpdater.h"
//#include "/home/johane/catkin_ws_dyn/src/quasimodo_models/include/modelupdater/ModelUpdater.h"
#include "../../quasimodo_models/include/modelupdater/ModelUpdater.h"
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

std::map<int , reglib::Camera *>		cameras;
std::map<int , reglib::RGBDFrame *>		frames;

std::map<int , reglib::Model *>			models;
std::map<int , reglib::ModelUpdater *>	updaters;
reglib::RegistrationGOICP *				registration;
ModelDatabase * 						modeldatabase;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

bool myfunction (reglib::Model * i,reglib::Model * j) { return i->frames.size() > j->frames.size(); }

int savecounter = 0;
void show_sorted(){
	std::vector<reglib::Model *> results;
	for(unsigned int i = 0; i < modeldatabase->models.size(); i++){
		results.push_back(modeldatabase->models[i]);
	}
	//for (std::map<int,reglib::Model *>::iterator it=models.begin(); it!=models.end(); ++it){
	//	results.push_back(it->second);
	//}

	std::sort (results.begin(), results.end(), myfunction);

	viewer->removeAllPointClouds();
	printf("number of models: %i\n",results.size());

	float maxx = 0;

	for(unsigned int i = 0; i < results.size(); i++){
		printf("results %i -> %i\n",i,results[i]->frames.size());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = results[i]->getPCLcloud(results[i]->frames.size(), false);
		float meanx = 0;
		float meany = 0;
		float meanz = 0;
		for(int j = 0; j < cloud->points.size(); j++){
			meanx += cloud->points[j].x;
			meany += cloud->points[j].y;
			meanz += cloud->points[j].z;
		}
		meanx /= float(cloud->points.size());
		meany /= float(cloud->points.size());
		meanz /= float(cloud->points.size());

		for(unsigned int j = 0; j < cloud->points.size(); j++){
			cloud->points[j].x -= meanx;
			cloud->points[j].y -= meany;
			cloud->points[j].z -= meanz;
		}

		float minx = 100000000000;

		for(unsigned int j = 0; j < cloud->points.size(); j++){minx = std::min(cloud->points[j].x , minx);}
		for(unsigned int j = 0; j < cloud->points.size(); j++){cloud->points[j].x += maxx-minx + 1.0;}
		for(unsigned int j = 0; j < cloud->points.size(); j++){maxx = std::max(cloud->points[j].x,maxx);}
		char buf [1024];
		sprintf(buf,"cloud%i",i);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), buf);
	}
	//viewer->spin();
	if(savecounter % 5 == 0){viewer->spin();}
	char buf [1024];
	sprintf(buf,"globalimg%i.png",savecounter++);
	viewer->saveScreenshot(std::string(buf));
	viewer->spinOnce();

	//viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf);
}


bool getModel(quasimodo_msgs::get_model::Request  & req, quasimodo_msgs::get_model::Response & res){
	int model_id			= req.model_id;
	reglib::Model * model	= models[model_id];
    printf("getModel\n");

	res.model.model_id = model_id;
	res.model.local_poses.resize(model->relativeposes.size());
	res.model.frames.resize(model->frames.size());
	res.model.masks.resize(model->masks.size());

	for(unsigned int i = 0; i < model->relativeposes.size(); i++){
		geometry_msgs::Pose		pose1;
		tf::poseEigenToMsg (Eigen::Affine3d(model->relativeposes[i]), pose1);
		res.model.local_poses[i] = pose1;


		geometry_msgs::Pose		pose2;
		tf::poseEigenToMsg (Eigen::Affine3d(model->frames[i]->pose), pose2);

		cv_bridge::CvImage rgbBridgeImage;
		rgbBridgeImage.image = model->frames[i]->rgb;
		rgbBridgeImage.encoding = "bgr8";

		cv_bridge::CvImage depthBridgeImage;
		depthBridgeImage.image = model->frames[i]->depth;
		depthBridgeImage.encoding = "mono16";

		res.model.frames[i].capture_time = ros::Time();
		//res.model.frames[i].pose		= pose2;
		res.model.frames[i].frame_id	= model->frames[i]->id;
		res.model.frames[i].rgb			= *(rgbBridgeImage.toImageMsg());
		res.model.frames[i].depth		= *(depthBridgeImage.toImageMsg());

		cv_bridge::CvImage maskBridgeImage;
		maskBridgeImage.image			= model->masks[i];
		maskBridgeImage.encoding		= "mono8";
		res.model.masks[i]				= *(maskBridgeImage.toImageMsg());
	}
	return true;
}

bool indexFrame(quasimodo_msgs::index_frame::Request  & req, quasimodo_msgs::index_frame::Response & res){
    printf("indexFrame\n");
    sensor_msgs::CameraInfo		camera			= req.frame.camera;
	ros::Time					capture_time	= req.frame.capture_time;
	geometry_msgs::Pose			pose			= req.frame.pose;

	cv_bridge::CvImagePtr			rgb_ptr;
	try{							rgb_ptr = cv_bridge::toCvCopy(req.frame.rgb, sensor_msgs::image_encodings::BGR8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}
	cv::Mat rgb = rgb_ptr->image;

	cv_bridge::CvImagePtr			depth_ptr;
	try{							depth_ptr = cv_bridge::toCvCopy(req.frame.depth, sensor_msgs::image_encodings::MONO16);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}
    cv::Mat depth = depth_ptr->image;

	Eigen::Affine3d epose;
	tf::poseMsgToEigen(pose, epose);

	reglib::RGBDFrame * frame = new reglib::RGBDFrame(cameras[0],rgb, depth, double(capture_time.sec)+double(capture_time.nsec)/1000000000.0, epose.matrix());
	frames[frame->id] = frame;
	res.frame_id = frame->id;
	return true;
}


class SearchResult{
	public:
	double score;
	reglib::Model * model;
	Eigen::Affine3d pose;

	SearchResult(double score_, reglib::Model * model_, Eigen::Affine3d pose_){
		score	= score_;
		model	= model_;
		pose	= pose_;
	}
	~SearchResult(){}
};

std::vector<SearchResult *> getSearchResult(reglib::Model * model){
	std::vector<SearchResult *> results;
	//printf("---getSearchResult---\n");
	//reglib::Model * model2 = 0;
	for (std::map<int,reglib::Model *>::iterator it=models.begin(); it!=models.end(); ++it){
		//std::cout << it->first << '\n';
		//printf("key: %i\n",it->first);
		//model2 = it->second;
		//break;
		if((it->second != 0) && (model->id != it->second->id)){
			//printf("found Model2->id %ld to match to\n",it->second->id);
			results.push_back(new SearchResult(1,it->second, Eigen::Affine3d::Identity()));
		}
	}
	//send model to NILS
	//printf("---getSearchResult---\n");
	return results;
}

bool removeModel(int id){
	printf("removeModel: %i\n",id);
	delete models[id];
	models.erase(id);
	delete updaters[id];
	updaters.erase(id);
}

reglib::Model * mod;
std::vector<reglib::Model * > res;
std::vector<reglib::FusionResults > fr_res;

void call_from_thread(int i) {
	reglib::Model * model2 = res[i];

	printf("testreg %i to %i\n",mod->id,model2->id);
	reglib::RegistrationGOICP *	reg		= new reglib::RegistrationGOICP();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);
	mu->viewer							= viewer;
	reg->visualizationLvl				= 1;

	reglib::FusionResults fr = mu->registerModel(mod);

	fr_res[i] = fr;


	delete mu;
	delete reg;
}

void addToDB(ModelDatabase * database, reglib::Model * model, bool add = true){
	if(add){database->add(model);}
	
	printf("added to database");

	//printf("new model created, time to run quasimodo algorithm\n");
	//New data in:
		//while true
		//Search for more of the same <-- Nils stuff, Rares stuff, other?
		//if the same is found
			//Register
			//Fuse
		//else
			//Break
	
	mod = model;
	res = modeldatabase->search(model,15);
	fr_res.resize(res.size());
	printf("nr_matches: %i\n",res.size());

	if(res.size() == 0){return;}
	
	std::vector<reglib::Model * > models2merge;
	std::vector<reglib::FusionResults > fr2merge;

	bool multi_thread = false;
	if(multi_thread){
		const int num_threads = res.size();
		std::thread t[num_threads];

		for (int i = 0; i < num_threads; ++i) {
			t[i] = std::thread(call_from_thread,i);
		}

		std::cout << "Launched from the main\n";

		for (int i = 0; i < num_threads; ++i) {t[i].join();}

		for (int i = 0; i < num_threads; ++i) {
			reglib::Model * model2 = res[i];
			reglib::FusionResults fr = fr_res[i];
			if(fr.score > 800){
				fr.guess = fr.guess.inverse();

				fr2merge.push_back(fr);
				models2merge.push_back(model2);
				printf("%i could be registered\n",i);
			}
		}
	}else{
		for(unsigned int i = 0; i < res.size(); i++){
			reglib::Model * model2 = res[i];

			printf("testreg %i to %i\n",model->id,model2->id);
			reglib::RegistrationGOICP *	reg		= new reglib::RegistrationGOICP();
			reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);
			mu->viewer							= viewer;
			reg->visualizationLvl				= 2;

			reglib::FusionResults fr = mu->registerModel(model);

			if(fr.score > 800){
				fr.guess = fr.guess.inverse();

				fr2merge.push_back(fr);
				models2merge.push_back(model2);
				printf("could be registered\n");
			}

			delete mu;
			delete reg;
		}
	}\

	std::map<int , reglib::Model *>	new_models;
	std::map<int , reglib::Model *>	updated_models;
	
	for(unsigned int i = 0; i < models2merge.size(); i++){
		reglib::Model * model2 = models2merge[i];

		reglib::RegistrationGOICP *	reg		= new reglib::RegistrationGOICP();
		reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);
		mu->viewer							= viewer;
		reg->visualizationLvl				= 1;

		reglib::UpdatedModels ud = mu->fuseData(&(fr2merge[i]), model, model2);

		printf("merge %i to %i\n",model->id,model2->id);
		printf("new_models:     %i\n",ud.new_models.size());
		printf("updated_models: %i\n",ud.updated_models.size());
		printf("deleted_models: %i\n",ud.deleted_models.size());
			
		delete mu;
		delete reg;
		
		for(unsigned int j = 0; j < ud.new_models.size(); j++){
			new_models[ud.new_models[j]->id] = ud.new_models[j];
		}
		
		for(unsigned int j = 0; j < ud.updated_models.size(); j++){
			updated_models[ud.updated_models[j]->id] = ud.updated_models[j];
		}
		
		for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
			database->remove(ud.deleted_models[j]);
			delete ud.deleted_models[j];
		}		
	}
	
	
	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end(); ++it){
		database->remove(it->second);	
	}
	
	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end(); ++it){
		addToDB(database, it->second);
	}
	
	for (std::map<int,reglib::Model *>::iterator it=new_models.begin(); it!=new_models.end(); ++it){
		addToDB(database, it->second);
	}
	
	printf("DONE WITH REGISTER\n");
}

bool modelFromFrame(quasimodo_msgs::model_from_frame::Request  & req, quasimodo_msgs::model_from_frame::Response & res){
	printf("======================================\nmodelFromFrame\n======================================\n");
    uint64 frame_id = req.frame_id;
    cv_bridge::CvImagePtr			mask_ptr;
	try{							mask_ptr = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::MONO8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}
	
	cv::Mat mask					= mask_ptr->image;
	reglib::Model * newmodel		= new reglib::Model(frames[frame_id],mask);
	
    res.model_id					= newmodel->id;
    	
	addToDB(modeldatabase, newmodel);
	show_sorted();
	return true;
/*
    	//Search for more of the same <-- Nils stuff, Rares stuff, other?
    	std::vector<SearchResult *> sr = getSearchResult(newmodel);
    	bool changed = false;

		std::vector<reglib::FusionResults> fr_to_fuse;
		std::vector<SearchResult *> sr_to_fuse;

		while(sr.size() != 0){
			SearchResult * result = sr.back();
			sr.pop_back();
			if(result->score > 0.5){
				reglib::RegistrationGOICP *	reg = new reglib::RegistrationGOICP();
				
				//reglib::ModelUpdaterBasicFuse * muf1 = new reglib::ModelUpdaterBasicFuse( newmodel, reg);
				reglib::ModelUpdaterBasicFuse * muf2 = new reglib::ModelUpdaterBasicFuse( result->model, reg);
				
				reglib::FusionResults fr2 = muf2->registerModel(newmodel);
				
				delete muf2;
				delete reg;
			
				reglib::FusionResults fr = updaters[result->model->id]->registerModel(newmodel);
				if(fr.score > 800){
					fr.guess = fr.guess.inverse();
					fr_to_fuse.push_back(fr);
					sr_to_fuse.push_back(result);
					printf("could be registered\n");
					continue;
				}
			}
			delete result;
		}
		for(unsigned int i = 0; i < fr_to_fuse.size(); i++){

			//((reglib::ModelUpdaterBasicFuse *)updaters[newmodel->id])->graph->merge(fr_to_fuse[i].guess.matrix().cast<double>(), ((reglib::ModelUpdaterBasicFuse *)updaters[sr_to_fuse[i]->model->id])->graph);
			reglib::UpdatedModels ud = updaters[newmodel->id]->fuseData(&(fr_to_fuse[i]), newmodel, sr_to_fuse[i]->model);
			printf("new_models:     %i\n",ud.new_models.size());
			printf("updated_models: %i\n",ud.updated_models.size());
			printf("deleted_models: %i\n",ud.deleted_models.size());
			for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
				removeModel(ud.deleted_models[j]->id);
			}
			//newmodel = result->model;
			//changed = true;
			//break;
		}
		if(fr_to_fuse.size() == 0){
			break;
		}

    }
    */

}

/*
bool modelFromFrame(quasimodo_msgs::model_from_frame::Request  & req, quasimodo_msgs::model_from_frame::Response & res){
	printf("======================================\nmodelFromFrame\n======================================\n");
    uint64 frame_id = req.frame_id;
    cv_bridge::CvImagePtr			mask_ptr;
	try{							mask_ptr = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::MONO8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}
	
	cv::Mat mask					= mask_ptr->image;
	reglib::Model * newmodel		= new reglib::Model(frames[frame_id],mask);
	
	modeldatabase->add(newmodel);
	printf("added to database");
    models[newmodel->id]			= newmodel;

	char buf [1024];
	sprintf(buf,"mask%i.png",frame_id);
	cv::imwrite( buf, mask );
	printf("saving: %s\n",buf);

	reglib::ModelUpdaterBasicFuse * muf = new reglib::ModelUpdaterBasicFuse( newmodel, registration);


	reglib::ModelUpdater * mu       = muf;
	mu->viewer						= viewer;
    updaters[newmodel->id]			= mu;
    
    res.model_id					= newmodel->id;
    

	if(newmodel->id > 10 ){	registration->visualizationLvl = 1;}
	else{					registration->visualizationLvl = 1;}

	//printf("new model created, time to run quasimodo algorithm\n");
    
    //Wait for data...
	//New data in:
		//while true
		//Search for more of the same <-- Nils stuff, Rares stuff, other?
		//if the same is found
			//Register
			//Fuse
		//else
			//Break
    while(true){
    	//Search for more of the same <-- Nils stuff, Rares stuff, other?
    	std::vector<SearchResult *> sr = getSearchResult(newmodel);
    	bool changed = false;

		std::vector<reglib::FusionResults> fr_to_fuse;
		std::vector<SearchResult *> sr_to_fuse;

		while(sr.size() != 0){
			SearchResult * result = sr.back();
			sr.pop_back();
			if(result->score > 0.5){
				reglib::RegistrationGOICP *	reg = new reglib::RegistrationGOICP();
				
				//reglib::ModelUpdaterBasicFuse * muf1 = new reglib::ModelUpdaterBasicFuse( newmodel, reg);
				reglib::ModelUpdaterBasicFuse * muf2 = new reglib::ModelUpdaterBasicFuse( result->model, reg);
				
				reglib::FusionResults fr2 = muf2->registerModel(newmodel);
				
				delete muf2;
				delete reg;
			
				reglib::FusionResults fr = updaters[result->model->id]->registerModel(newmodel);
				if(fr.score > 800){
					fr.guess = fr.guess.inverse();
					fr_to_fuse.push_back(fr);
					sr_to_fuse.push_back(result);
					printf("could be registered\n");
					continue;
				}
			}
			delete result;
		}
		for(unsigned int i = 0; i < fr_to_fuse.size(); i++){

			//((reglib::ModelUpdaterBasicFuse *)updaters[newmodel->id])->graph->merge(fr_to_fuse[i].guess.matrix().cast<double>(), ((reglib::ModelUpdaterBasicFuse *)updaters[sr_to_fuse[i]->model->id])->graph);
			reglib::UpdatedModels ud = updaters[newmodel->id]->fuseData(&(fr_to_fuse[i]), newmodel, sr_to_fuse[i]->model);
			printf("new_models:     %i\n",ud.new_models.size());
			printf("updated_models: %i\n",ud.updated_models.size());
			printf("deleted_models: %i\n",ud.deleted_models.size());
			for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
				removeModel(ud.deleted_models[j]->id);
			}
			//newmodel = result->model;
			//changed = true;
			//break;
		}
		if(fr_to_fuse.size() == 0){
			break;
		}
    }

	//show_sorted();
	printf("DONE WITH REGISTER\n");
	return true;
}
*/
bool fuseModels(quasimodo_msgs::fuse_models::Request  & req, quasimodo_msgs::fuse_models::Response & res){}


int main(int argc, char **argv){
	ros::init(argc, argv, "quasimodo_model_server");
	ros::NodeHandle n;

	cameras[0]		= new reglib::Camera();
	registration	= new reglib::RegistrationGOICP();
	modeldatabase	= new ModelDatabaseBasic();

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);

	ros::ServiceServer service1 = n.advertiseService("model_from_frame", modelFromFrame);
	ROS_INFO("Ready to add use model_from_frame.");

	ros::ServiceServer service2 = n.advertiseService("index_frame", indexFrame);
	ROS_INFO("Ready to add use index_frame.");

	ros::ServiceServer service3 = n.advertiseService("fuse_models", fuseModels);
	ROS_INFO("Ready to add use fuse_models.");

	ros::ServiceServer service4 = n.advertiseService("get_model", getModel);
	ROS_INFO("Ready to add use get_model.");
	ros::spin();

	return 0;
}
