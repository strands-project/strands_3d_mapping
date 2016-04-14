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

#include <sys/time.h>

bool visualization = false;


std::map<int , reglib::Camera *>		cameras;
std::map<int , reglib::RGBDFrame *>		frames;

std::map<int , reglib::Model *>			models;
std::map<int , reglib::ModelUpdater *>	updaters;
reglib::RegistrationGOICP *				registration;
ModelDatabase * 						modeldatabase;

std::string								savepath = ".";

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

ros::Publisher models_new_pub;
ros::Publisher models_updated_pub;
ros::Publisher models_deleted_pub;

ros::ServiceClient soma2add;

double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

quasimodo_msgs::retrieval_result sresult;
bool new_search_result = false;
double last_search_result_time = 0;

void dumpDatabase(std::string path = "."){
	char command [1024];
	sprintf(command,"rm -r %s/model*",path.c_str());
	printf("%s\n",command);
	system(command);

	sprintf(command,"rm %s/camera*",path.c_str());
	printf("%s\n",command);
	system(command);

	cameras[0]->save(path+"/camera0");
	for(unsigned int m = 0; m < modeldatabase->models.size(); m++){
		char buf [1024];
		sprintf(buf,"%s/model%i",path.c_str(),m);
		sprintf(command,"mkdir -p %s",buf);
		system(command);
		modeldatabase->models[m]->save(std::string(buf));
	}
}

void retrievalCallback(const quasimodo_msgs::retrieval_query_result & qr){
	printf("retrievalCallback\n");
	sresult = qr.result;
	new_search_result = true;
	last_search_result_time = getTime();
}


bool myfunction (reglib::Model * i,reglib::Model * j) { return i->frames.size() > j->frames.size(); }

int savecounter = 0;
void show_sorted(){
    if(!visualization){return;}
	std::vector<reglib::Model *> results;
	for(unsigned int i = 0; i < modeldatabase->models.size(); i++){results.push_back(modeldatabase->models[i]);}
	std::sort (results.begin(), results.end(), myfunction);
	viewer->removeAllPointClouds();
	float maxx = 0;
	for(unsigned int i = 0; i < results.size(); i++){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = results[i]->getPCLcloud(1, false);
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
		for(unsigned int j = 0; j < cloud->points.size(); j++){cloud->points[j].x += maxx-minx + 0.15;}
		for(unsigned int j = 0; j < cloud->points.size(); j++){maxx = std::max(cloud->points[j].x,maxx);}
		char buf [1024];
		sprintf(buf,"cloud%i",i);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), buf);
	}
	viewer->spin();
	//char buf [1024];
	//sprintf(buf,"globalimg%i.png",savecounter++);
	//viewer->saveScreenshot(std::string(buf));
	//viewer->spinOnce();
}

quasimodo_msgs::model getModelMSG(reglib::Model * model){
	quasimodo_msgs::model msg;
	msg.model_id = model->id;
	msg.local_poses.resize(model->relativeposes.size());
	msg.frames.resize(model->frames.size());
	msg.masks.resize(model->modelmasks.size());
	for(unsigned int i = 0; i < model->relativeposes.size(); i++){
		geometry_msgs::Pose		pose1;
		tf::poseEigenToMsg (Eigen::Affine3d(model->relativeposes[i]), pose1);
		geometry_msgs::Pose		pose2;
		tf::poseEigenToMsg (Eigen::Affine3d(model->frames[i]->pose), pose2);
		cv_bridge::CvImage rgbBridgeImage;
		rgbBridgeImage.image = model->frames[i]->rgb;
		rgbBridgeImage.encoding = "bgr8";
		cv_bridge::CvImage depthBridgeImage;
		depthBridgeImage.image = model->frames[i]->depth;
		depthBridgeImage.encoding = "mono16";
		cv_bridge::CvImage maskBridgeImage;
		maskBridgeImage.image			= model->modelmasks[i]->getMask();
		maskBridgeImage.encoding		= "mono8";
		msg.local_poses[i]			= pose1;
		msg.frames[i].capture_time	= ros::Time();
		msg.frames[i].pose			= pose2;
		msg.frames[i].frame_id		= model->frames[i]->id;
		msg.frames[i].rgb			= *(rgbBridgeImage.toImageMsg());
		msg.frames[i].depth			= *(depthBridgeImage.toImageMsg());
		msg.masks[i]				= *(maskBridgeImage.toImageMsg());//getMask()
	}
	return msg;
}

std::vector<soma2_msgs::SOMA2Object> getSOMA2ObjectMSGs(reglib::Model * model){
	std::vector<soma2_msgs::SOMA2Object> msgs;
	for(int i = 0; i < model->frames.size(); i++){
		soma2_msgs::SOMA2Object msg;
		char buf [1024];
		sprintf(buf,"id_%i_%i",model->id,model->frames[i]->id);
		msg.id				= std::string(buf);
		msg.map_name		= "whatevermapname";				//	#### the global map that the object belongs. Automatically filled by SOMA2 insert service
		msg.map_unique_id	= "";								// 	#### the unique db id of the map. Automatically filled by SOMA2 insert service
		msg.config			= "configid";						//	#### the corresponding world configuration. This could be incremented as config1, config2 at each meta-room observation
		msg.mesh			= "";								//	#### mesh model of the object. Could be left blank
		sprintf(buf,"type_%i",model->id);
		msg.type			= std::string(buf);					//	#### type of the object. For higher level objects, this could chair1, chair2. For middle or lower level segments it could be segment1101, segment1102, etc.
//		msg.waypoint		= "";								//	#### the waypoint id. Could be left blank
		msg.timestep 		= 0;								//	#### this is the discrete observation instance. This could be incremented at each meta-room observation as 1,2,3,etc...
		msg.logtimestamp	= 1000000.0 * double(model->frames[i]->capturetime);	//	#### this is the unix timestamp information that holds the time when the object is logged. SOMA2 uses UTC time values.

		Eigen::Affine3f transform = Eigen::Affine3f(model->frames[i]->pose.cast<float>());//Eigen::Affine3f::Identity();

		bool * maskdata = model->modelmasks[i]->maskvec;
		unsigned char * rgbdata = (unsigned char *) model->frames[i]->rgb.data;
		unsigned short * depthdata = (unsigned short *) model->frames[i]->depth.data;

		const unsigned int width	= model->frames[i]->camera->width; const unsigned int height	= model->frames[i]->camera->height;
		const double idepth			= model->frames[i]->camera->idepth_scale;
		const double cx				= model->frames[i]->camera->cx;		const double cy				= model->frames[i]->camera->cy;
		const double ifx			= 1.0/model->frames[i]->camera->fx;	const double ify			= 1.0/model->frames[i]->camera->fy;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr	cloud	(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr	transformed_cloud	(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(unsigned int w = 0; w < width; w++){
			for(unsigned int h = 0; h < height;h++){
				int ind = h*width+w;
				double z = idepth*double(depthdata[ind]);
				if(z > 0 && maskdata[ind]){
					pcl::PointXYZRGB p;
					p.x = (double(w) - cx) * z * ifx;
					p.y = (double(h) - cy) * z * ify;
					p.z = z;
					p.b = rgbdata[3*ind+0];
					p.g = rgbdata[3*ind+1];
					p.r = rgbdata[3*ind+2];
					cloud->points.push_back(p);
				}
			}
		}

		pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
		sensor_msgs::PointCloud2ConstPtr input;
		pcl::fromROSMsg (*input, *transformed_cloud);


		msg.cloud = *input; //#### The 3D cloud of the object		(with respect to /map frame)

		Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Affine3d(model->frames[i]->pose).rotation());

		geometry_msgs::Pose		pose;
		pose.orientation.x	= q.x();
		pose.orientation.y	= q.y();
		pose.orientation.z	= q.z();
		pose.orientation.w	= q.w();
		pose.position.x		= model->frames[i]->pose(0,3);
		pose.position.y		= model->frames[i]->pose(1,3);
		pose.position.z		= model->frames[i]->pose(2,3);
		msg.pose = pose;		//	#### Object pose in 3D			(with respect to /map frame)
//		geometry_msgs/Pose sweepCenter 	#### The pose of the robot during sweep (with respect to /map frame)
		msgs.push_back(msg);
	}

//	msg.model_id = model->id;
//	msg.local_poses.resize(model->relativeposes.size());
//	msg.frames.resize(model->frames.size());
//	msg.masks.resize(model->modelmasks.size());
//	for(unsigned int i = 0; i < model->relativeposes.size(); i++){
//		geometry_msgs::Pose		pose1;
//		tf::poseEigenToMsg (Eigen::Affine3d(model->relativeposes[i]), pose1);
//		geometry_msgs::Pose		pose2;
//		tf::poseEigenToMsg (Eigen::Affine3d(model->frames[i]->pose), pose2);
//		cv_bridge::CvImage rgbBridgeImage;
//		rgbBridgeImage.image = model->frames[i]->rgb;
//		rgbBridgeImage.encoding = "bgr8";
//		cv_bridge::CvImage depthBridgeImage;
//		depthBridgeImage.image = model->frames[i]->depth;
//		depthBridgeImage.encoding = "mono16";
//		cv_bridge::CvImage maskBridgeImage;
//		maskBridgeImage.image			= model->modelmasks[i]->getMask();
//		maskBridgeImage.encoding		= "mono8";
//		msg.local_poses[i]			= pose1;
//		msg.frames[i].capture_time	= ros::Time();
//		msg.frames[i].pose			= pose2;
//		msg.frames[i].frame_id		= model->frames[i]->id;
//		msg.frames[i].rgb			= *(rgbBridgeImage.toImageMsg());
//		msg.frames[i].depth			= *(depthBridgeImage.toImageMsg());
//		msg.masks[i]				= *(maskBridgeImage.toImageMsg());//getMask()
//	}
	return msgs;
}

void publishModel(reglib::Model * model){
//	std::vector<soma2_msgs::SOMA2Object> somamsgs = getSOMA2ObjectMSGs(model);
	soma_manager::SOMA2InsertObjs objs;
	objs.request.objects = getSOMA2ObjectMSGs(model);
	if (soma2add.call(objs)){//Add frame to model server
	////			int frame_id = ifsrv.response.frame_id;
	}else{ROS_ERROR("Failed to call service soma2add");}

//	for(int i = 0; i < somamsgs.size(); i++){
//		if (soma2add.call(somamsgs[i])){//Add frame to model server
////			int frame_id = ifsrv.response.frame_id;
//		 }else{ROS_ERROR("Failed to call service soma2add");}
//	 }
}

bool getModel(quasimodo_msgs::get_model::Request  & req, quasimodo_msgs::get_model::Response & res){
	int model_id			= req.model_id;
	reglib::Model * model	= models[model_id];
	res.model = getModelMSG(model);
	return true;
}

bool indexFrame(quasimodo_msgs::index_frame::Request  & req, quasimodo_msgs::index_frame::Response & res){
	//printf("indexFrame\n");
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

bool removeModel(int id){
	//printf("removeModel: %i\n",id);
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
	reglib::RegistrationRandom *	reg		= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);
	mu->viewer							= viewer;
	reg->visualizationLvl				= 0;

	reglib::FusionResults fr = mu->registerModel(mod);
	fr_res[i] = fr;

	delete mu;
	delete reg;
}


int current_model_update = 0;
void addToDB(ModelDatabase * database, reglib::Model * model, bool add = true, bool deleteIfFail = false){
	printf("addToDB %i %i\n",add,deleteIfFail);
	if(add){

		if(model->frames.size() > 2){
			reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
			reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model, 0);
			mu->viewer							= viewer;
			reg->visualizationLvl				= 0;
			mu->refine(0.001,true);
			delete mu;
			delete reg;
		}
		database->add(model);
		//models_new_pub.publish(getModelMSG(model));
		model->last_changed = ++current_model_update;
//		show_sorted();
	}

	mod = model;
	res = modeldatabase->search(model,150);
	fr_res.resize(res.size());

	if(res.size() == 0){
		printf("no candidates found in database!\n");
		return;
	}

	std::vector<reglib::Model * > models2merge;
	std::vector<reglib::FusionResults > fr2merge;
	bool multi_thread = true;
	if(multi_thread){
		const int num_threads = res.size();
		std::thread t[num_threads];
		for (int i = 0; i < num_threads; ++i) {t[i] = std::thread(call_from_thread,i);}
		for (int i = 0; i < num_threads; ++i) {t[i].join();}
		for (int i = 0; i < num_threads; ++i) {
			reglib::Model * model2 = res[i];
			reglib::FusionResults fr = fr_res[i];
			if(fr.score > 100){
				fr.guess = fr.guess.inverse();
				fr2merge.push_back(fr);
				models2merge.push_back(model2);
				printf("%i could be registered\n",i);
			}
		}
	}else{
		for(unsigned int i = 0; i < res.size(); i++){
			reglib::Model * model2 = res[i];
			reglib::RegistrationRandom *	reg		= new reglib::RegistrationRandom();
			reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);

			mu->viewer							= viewer;
			reg->visualizationLvl				= 0;
			reglib::FusionResults fr = mu->registerModel(model);

			if(fr.score > 100){
				fr.guess = fr.guess.inverse();
				fr2merge.push_back(fr);
				models2merge.push_back(model2);
			}
			delete mu;
			delete reg;
		}
	}

	bool changed = false;
	std::map<int , reglib::Model *>	new_models;
	std::map<int , reglib::Model *>	updated_models;
	for(unsigned int i = 0; i < models2merge.size(); i++){
		reglib::Model * model2 = models2merge[i];

		reglib::RegistrationRandom *	reg		= new reglib::RegistrationRandom();
		reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2, reg);
		mu->viewer							= viewer;
		reg->visualizationLvl				= 0;

		reglib::UpdatedModels ud = mu->fuseData(&(fr2merge[i]), model, model2);

		printf("merge %i to %i\n",model->id,model2->id);
		printf("new_models:     %i\n",ud.new_models.size());
		printf("updated_models: %i\n",ud.updated_models.size());
		printf("deleted_models: %i\n",ud.deleted_models.size());

		delete mu;
		delete reg;

		for(unsigned int j = 0; j < ud.new_models.size(); j++){		new_models[ud.new_models[j]->id]			= ud.new_models[j];}
		for(unsigned int j = 0; j < ud.updated_models.size(); j++){	updated_models[ud.updated_models[j]->id]	= ud.updated_models[j];}

		for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
			database->remove(ud.deleted_models[j]);
			delete ud.deleted_models[j];
		}
		if(ud.deleted_models.size() > 0){changed = true; break;}
	}

	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end();	++it){	database->remove(it->second); 		models_deleted_pub.publish(getModelMSG(it->second));}
	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end();	++it){	addToDB(database, it->second);}
	for (std::map<int,reglib::Model *>::iterator it=new_models.begin();		it!=new_models.end();		++it){	addToDB(database, it->second);}

	printf("end of addToDB: %i %i",add,deleteIfFail);
	if(deleteIfFail){
		if(!changed){
			printf("didnt manage to integrate searchresult\n");
			database->remove(model);
			for(int i = 0; i < model->frames.size(); i++){
				delete model->frames[i];
				delete model->modelmasks[i];
			}
			delete model;
		}else{
			printf("integrateing searchresult\n");
		}
	}
}

bool nextNew = true;
reglib::Model * newmodel = 0;

int sweepid_counter = 0;

//std::vector<cv::Mat> newmasks;
std::vector<cv::Mat> allmasks;

int modaddcount = 0;
bool modelFromFrame(quasimodo_msgs::model_from_frame::Request  & req, quasimodo_msgs::model_from_frame::Response & res){
	printf("======================================\nmodelFromFrame\n======================================\n");
	uint64 frame_id = req.frame_id;
	uint64 isnewmodel = req.isnewmodel;

	printf("%i and %i\n",long(frame_id),long(isnewmodel));
	cv_bridge::CvImagePtr			mask_ptr;
	try{							mask_ptr = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::MONO8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}

	cv::Mat mask					= mask_ptr->image;
	allmasks.push_back(mask);
	std::cout << frames[frame_id]->pose << std::endl << std::endl;

	cv::Mat fullmask;
	fullmask.create(480,640,CV_8UC1);
	unsigned char * maskdata = (unsigned char *)fullmask.data;
	for(unsigned int j = 0; j < 640*480; j++){maskdata[j] = 255;}
	if(newmodel == 0){
		newmodel					= new reglib::Model(frames[frame_id],mask);
		modeldatabase->add(newmodel);
//		newmodel->masks.back() = fullmask;
//		newmodel->recomputeModelPoints();
	}else{
//		reglib::Model * inputmodel					= new reglib::Model(frames[frame_id],mask);
//		inputmodel->masks.back() = fullmask;
//		inputmodel->recomputeModelPoints();

//		reglib::RegistrationRefinement *	reg		= new reglib::RegistrationRefinement();
//		//RegistrationRefinement
//		reg->target_points							= 3000;
//		reglib::ModelUpdaterBasicFuse * mu			= new reglib::ModelUpdaterBasicFuse( inputmodel, reg);
//		mu->viewer									= viewer;
//		reg->visualizationLvl						= 4;
//		reglib::FusionResults fr = mu->registerModel(newmodel,(newmodel->frames.front()->pose.inverse() * frames[frame_id]->pose).inverse());
//		fr.guess = fr.guess.inverse();
//		delete inputmodel;
//		delete mu;
//		delete reg;

		newmodel->frames.push_back(frames[frame_id]);
		//newmodel->masks.push_back(mask);
		newmodel->relativeposes.push_back(newmodel->frames.front()->pose.inverse() * frames[frame_id]->pose);
		//newmodel->relativeposes.push_back(frames[frame_id]->pose);
		//newmodel->relativeposes.push_back(newmodel->frames.front()->pose.inverse() * fr.guess);
		newmodel->modelmasks.push_back(new reglib::ModelMask(mask));
		newmodel->recomputeModelPoints();

	}
	newmodel->modelmasks.back()->sweepid = sweepid_counter;
//printf("LINE: %i\n",__LINE__);

	//show_sorted();

	res.model_id					= newmodel->id;

	if(isnewmodel != 0){
		//newmodel->masks = newmasks;
		//newmasks.clear();
		newmodel->recomputeModelPoints();
		//show_sorted();
		reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
		reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( newmodel, reg);
		mu->viewer							= viewer;
//		reg->visualizationLvl				= 0;
		mu->refine(0.01,true);
		mu->recomputeScores();
		delete mu;
		delete reg;
		int current_model_update_before = current_model_update;
		newmodel->recomputeModelPoints();
		newmodel->last_changed = ++current_model_update;
		newmodel->print();
		addToDB(modeldatabase, newmodel,false);
		//if(modaddcount % 1 == 0){show_sorted();}
		modaddcount++;
		dumpDatabase(savepath);

//		cameras[0]->save("./camera0");
//		for(unsigned int m = 0; m < modeldatabase->models.size(); m++){
//			char buf [1024];
//			sprintf(buf,"./model%i",m);
//			char command [1024];
//			sprintf(command,"mkdir -p %s",buf);
//			system(command);
//			modeldatabase->models[m]->save(std::string(buf));
//		}
//exit(0);
		bool run_search = false;
		for(unsigned int m = 0; run_search && m < modeldatabase->models.size(); m++){
			printf("looking at: %i\n",modeldatabase->models[m]->last_changed);
			reglib::Model * currentTest = modeldatabase->models[m];
			if(currentTest->last_changed > current_model_update_before){
				printf("changed: %i\n",m);

				double start = getTime();
                double timelimit = 1;//30;

				new_search_result = false;
				models_new_pub.publish(getModelMSG(currentTest));

				while(getTime()-start < timelimit){
                    ros::spinOnce();
					if(new_search_result){

						for(int i = 0; i < sresult.retrieved_images.size(); i++){
							for(int j = 0; j < 1 && j < sresult.retrieved_images[i].images.size(); j++){

								cv_bridge::CvImagePtr ret_image_ptr;
								try {ret_image_ptr = cv_bridge::toCvCopy(sresult.retrieved_images[i].images[j], sensor_msgs::image_encodings::BGR8);}
								catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

								cv_bridge::CvImagePtr ret_mask_ptr;
								try {ret_mask_ptr = cv_bridge::toCvCopy(sresult.retrieved_masks[i].images[j], sensor_msgs::image_encodings::MONO8);}
								catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

								cv_bridge::CvImagePtr ret_depth_ptr;
								try {ret_depth_ptr = cv_bridge::toCvCopy(sresult.retrieved_depths[i].images[j], sensor_msgs::image_encodings::MONO16);}
								catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

								cv::Mat rgbimage	= ret_image_ptr->image;
                                cv::Mat maskimage	= ret_mask_ptr->image;
                                cv::Mat depthimage	= ret_depth_ptr->image;
                                for (int ii = 0; ii < depthimage.rows; ++ii) {
                                    for (int jj = 0; jj < depthimage.cols; ++jj) {
                                        depthimage.at<uint16_t>(ii, jj) /= 2;
                                    }
                                }



                                // Remove this when this is correct:

//                                cv::Mat masked_rgb = depthimage.clone();
//                                masked_rgb.setTo(cv::Scalar(0, 0, 0), maskimage == 0);
//								cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
//								cv::imshow(		"maskimage",	maskimage);
//								cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
//								cv::imshow(		"rgbimage",		rgbimage );
//								cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
//								cv::imshow(		"depthimage",	depthimage );
//                                cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
//                                cv::imshow(		"mask",	masked_rgb);
//                                cv::waitKey(0);


								//printf("indexFrame\n");
								//sensor_msgs::CameraInfo		camera			= req.frame.camera;
								//ros::Time					capture_time	= req.frame.capture_time;
								//geometry_msgs::Pose			pose			= req.frame.pose;

								Eigen::Affine3d epose = Eigen::Affine3d::Identity();
								//tf::poseMsgToEigen(pose, epose);

								reglib::RGBDFrame * frame = new reglib::RGBDFrame(cameras[0],rgbimage, depthimage, 0, epose.matrix());
								reglib::Model * searchmodel = new reglib::Model(frame,maskimage);

								printf("--- trying to add serach results, if more then one addToDB: results added-----\n");
								addToDB(modeldatabase, newmodel,false,true);

//								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = searchmodel->getPCLcloud(1, false);
//								viewer->removeAllPointClouds();
//								viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
//								viewer->spin();

//								reglib::RegistrationRandom *	reg		= new reglib::RegistrationRandom();
//								reglib::ModelUpdaterBasicFuse * mu		= new reglib::ModelUpdaterBasicFuse( searchmodel, reg);
//								mu->viewer								= viewer;
//                                reg->visualizationLvl					= 0;

//								reglib::FusionResults fr = mu->registerModel(currentTest);
//								reglib::UpdatedModels ud = mu->fuseData(&(fr), currentTest, searchmodel);

//								printf("merge %i to %i\n",		currentTest->id,searchmodel->id);
//								printf("new_models:     %i\n",ud.new_models.size());
//								printf("updated_models: %i\n",ud.updated_models.size());
//								printf("deleted_models: %i\n",ud.deleted_models.size());

//								for(unsigned int j = 0; j < ud.new_models.size(); j++){
//									modeldatabase->add(ud.new_models[j]);
//								}

//								for(unsigned int j = 0; j < ud.updated_models.size(); j++){
//									modeldatabase->remove(ud.updated_models[j]);
//									modeldatabase->add(ud.updated_models[j]);
//								}

//								bool searchmodel_merged = false;
//								for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
//									if(ud.deleted_models[j] == searchmodel){
//										searchmodel_merged = true;
//									}else{
//										modeldatabase->remove(ud.deleted_models[j]);
//									}
//									delete ud.deleted_models[j];
//								}

//								if(searchmodel_merged){
//									frames[frame->id] = frame;
//								}

//								delete mu;
//								delete reg;
							}
						}

						break;
					}else{
						printf("searching... timeout in %3.3f\n", +start +timelimit - getTime());
						usleep(100000);
					}
				}
			}
			exit(0);
		}
		newmodel = 0;
		sweepid_counter++;
	}
	return true;
}

bool fuseModels(quasimodo_msgs::fuse_models::Request  & req, quasimodo_msgs::fuse_models::Response & res){}

int main(int argc, char **argv){
	ros::init(argc, argv, "quasimodo_model_server");
	ros::NodeHandle n;

	cameras[0]		= new reglib::Camera();
	registration	= new reglib::RegistrationGOICP();
	modeldatabase	= new ModelDatabaseBasic();

    if(visualization){
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
        viewer->addCoordinateSystem(0.1);
        viewer->setBackgroundColor(0.9,0.9,0.9);
    }
	models_new_pub		= n.advertise<quasimodo_msgs::model>("/models/new",		1000);
	models_updated_pub	= n.advertise<quasimodo_msgs::model>("/models/updated", 1000);
	models_deleted_pub	= n.advertise<quasimodo_msgs::model>("/models/deleted", 1000);

	ros::ServiceServer service1 = n.advertiseService("model_from_frame", modelFromFrame);
	ROS_INFO("Ready to add use model_from_frame.");

	ros::ServiceServer service2 = n.advertiseService("index_frame", indexFrame);
	ROS_INFO("Ready to add use index_frame.");

	ros::ServiceServer service3 = n.advertiseService("fuse_models", fuseModels);
	ROS_INFO("Ready to add use fuse_models.");

	ros::ServiceServer service4 = n.advertiseService("get_model", getModel);
	ROS_INFO("Ready to add use get_model.");

	soma2add					= n.serviceClient<soma_manager::SOMA2InsertObjs>(		"soma2/insert_objs");
	ROS_INFO("Ready to add use soma2add.");

	ros::Subscriber sub = n.subscribe("/retrieval_result", 1, retrievalCallback);
	ROS_INFO("Ready to add recieve search results.");

	int inputstate = -1;
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		if(		std::string(argv[i]).compare("-c") == 0){	printf("camera input state\n"); inputstate = 1;}
		else if(std::string(argv[i]).compare("-m") == 0){	printf("model input state\n");	inputstate = 2;}
		else if(std::string(argv[i]).compare("-p") == 0){	printf("path input state\n");	inputstate = 3;}
		else if(inputstate == 1){
			//reglib::Camera * cam = reglib::Camera::load(std::string(argv[i]));
			//delete cameras[0];
			//cameras[0] = cam;
		}else if(inputstate == 2){
			//reglib::Model * model = reglib::Model::load(cameras[0],std::string(argv[i]));
			//sweepid_counter = std::max(int(model->modelmasks[0]->sweepid + 1), sweepid_counter);
			//modeldatabase->add(model);
			//addToDB(modeldatabase, model,false);
			//model->last_changed = ++current_model_update;
			//show_sorted();
		}else if(inputstate == 3){
			savepath = std::string(argv[i]);
		}
	}
//	exit(0);

    ros::Duration(1.0).sleep();
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("modelserver", 1000);
	sleep(1);
	std_msgs::String msg;
	msg.data = "starting";
	chatter_pub.publish(msg);
	ros::spinOnce();
	sleep(1);

	ros::spin();
	return 0;
}
