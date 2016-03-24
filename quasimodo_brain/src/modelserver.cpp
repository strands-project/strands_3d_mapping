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




std::map<int , reglib::Camera *>		cameras;
std::map<int , reglib::RGBDFrame *>		frames;

std::map<int , reglib::Model *>			models;
std::map<int , reglib::ModelUpdater *>	updaters;
reglib::RegistrationGOICP *				registration;
ModelDatabase * 						modeldatabase;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

ros::Publisher models_new_pub;
ros::Publisher models_updated_pub;
ros::Publisher models_deleted_pub;



double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

quasimodo_msgs::retrieval_result sresult;
bool new_search_result = false;
double last_search_result_time = 0;

void retrievalCallback(const quasimodo_msgs::retrieval_query_result & qr){
	printf("retrievalCallback\n");
	sresult = qr.result;
	new_search_result = true;
	last_search_result_time = getTime();
}

bool myfunction (reglib::Model * i,reglib::Model * j) { return i->frames.size() > j->frames.size(); }

int savecounter = 0;
void show_sorted(){
	std::vector<reglib::Model *> results;
	for(unsigned int i = 0; i < modeldatabase->models.size(); i++){results.push_back(modeldatabase->models[i]);}

	std::sort (results.begin(), results.end(), myfunction);

	viewer->removeAllPointClouds();
	printf("number of models: %i\n",results.size());

	float maxx = 0;
	for(unsigned int i = 0; i < results.size(); i++){
		printf("results %i -> %i\n",i,results[i]->frames.size());

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = results[i]->getPCLcloud(results[i]->frames.size(), false);
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
		for(unsigned int j = 0; j < cloud->points.size(); j++){cloud->points[j].x += maxx-minx + 1.0;}
		for(unsigned int j = 0; j < cloud->points.size(); j++){maxx = std::max(cloud->points[j].x,maxx);}
		char buf [1024];
		sprintf(buf,"cloud%i",i);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), buf);
	}
	viewer->spin();
	//if(savecounter % 5 == 0){viewer->spin();}
	char buf [1024];
	sprintf(buf,"globalimg%i.png",savecounter++);
	viewer->saveScreenshot(std::string(buf));
	viewer->spinOnce();

	//viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf);
}

quasimodo_msgs::model getModelMSG(reglib::Model * model){
	quasimodo_msgs::model msg;

	msg.model_id = model->id;
	msg.local_poses.resize(model->relativeposes.size());
	msg.frames.resize(model->frames.size());
	msg.masks.resize(model->masks.size());

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
		maskBridgeImage.image			= model->masks[i];
		maskBridgeImage.encoding		= "mono8";

		msg.local_poses[i]			= pose1;
		msg.frames[i].capture_time	= ros::Time();
		msg.frames[i].pose			= pose2;
		msg.frames[i].frame_id		= model->frames[i]->id;
		msg.frames[i].rgb			= *(rgbBridgeImage.toImageMsg());
		msg.frames[i].depth			= *(depthBridgeImage.toImageMsg());
		msg.masks[i]				= *(maskBridgeImage.toImageMsg());
	}
	return msg;
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
void addToDB(ModelDatabase * database, reglib::Model * model, bool add = true){
	if(add){
		database->add(model);
		//models_new_pub.publish(getModelMSG(model));
		model->last_changed = ++current_model_update;
	}
	printf("add to db\n");

	mod = model;
	res = modeldatabase->search(model,15);
	fr_res.resize(res.size());

	if(res.size() == 0){return;}

	std::vector<reglib::Model * > models2merge;
	std::vector<reglib::FusionResults > fr2merge;

	bool multi_thread = false;
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
	}

	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end();	++it){	database->remove(it->second); 		models_deleted_pub.publish(getModelMSG(it->second));}
	for (std::map<int,reglib::Model *>::iterator it=updated_models.begin(); it!=updated_models.end();	++it){	addToDB(database, it->second);}
	for (std::map<int,reglib::Model *>::iterator it=new_models.begin();		it!=new_models.end();		++it){	addToDB(database, it->second);}

	printf("DONE WITH REGISTER\n");
}

bool nextNew = true;
reglib::Model * newmodel = 0;

int sweepid_counter = 0;



bool modelFromFrame(quasimodo_msgs::model_from_frame::Request  & req, quasimodo_msgs::model_from_frame::Response & res){
	printf("======================================\nmodelFromFrame\n======================================\n");
	uint64 frame_id = req.frame_id;
	uint64 isnewmodel = req.isnewmodel;

	printf("%i and %i\n",long(frame_id),long(isnewmodel));
	cv_bridge::CvImagePtr			mask_ptr;
	try{							mask_ptr = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::MONO8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return false;}

	cv::Mat mask					= mask_ptr->image;
	if(newmodel == 0){
		newmodel					= new reglib::Model(frames[frame_id],mask);
		modeldatabase->add(newmodel);
	}else{
		newmodel->frames.push_back(frames[frame_id]);
		newmodel->masks.push_back(mask);
		newmodel->relativeposes.push_back(newmodel->frames.front()->pose.inverse() * frames[frame_id]->pose);
		newmodel->modelmasks.push_back(new reglib::ModelMask(mask));
		newmodel->recomputeModelPoints();
	}
	newmodel->modelmasks.back()->sweepid = sweepid_counter;

	res.model_id					= newmodel->id;

	if(isnewmodel != 0){

		newmodel->recomputeModelPoints();
		//show_sorted();

		reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
		reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( newmodel, reg);
		mu->viewer							= viewer;
		reg->visualizationLvl				= 0;

		mu->refine(0.05,true);

		delete mu;
		delete reg;

		int current_model_update_before = current_model_update;
		newmodel->recomputeModelPoints();
		//models_new_pub.publish(getModelMSG(newmodel));
		newmodel->last_changed = ++current_model_update;

		show_sorted();

		addToDB(modeldatabase, newmodel,false);

        for(unsigned int m = 0; m < modeldatabase->models.size(); m++){
			printf("looking at: %i\n",modeldatabase->models[m]->last_changed);
			reglib::Model * currentTest = modeldatabase->models[m];
			if(currentTest->last_changed > current_model_update_before){
				printf("changed: %i\n",m);

				double start = getTime();
				double timelimit = 30;

				new_search_result = false;
				models_new_pub.publish(getModelMSG(currentTest));

				while(getTime()-start < timelimit){
                    ros::spinOnce();
					if(new_search_result){

						for(int i = 0; i < sresult.retrieved_images.size(); i++){
							printf("i: %i\n",i);
							for(int j = 0; j < 1 && j < sresult.retrieved_images[i].images.size(); j++){
								printf("i: %i  j: %i\n",i,j);

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

                                printf("res rgb: %i %i\n",rgbimage.rows, rgbimage.cols);
                                printf("res mask: %i %i\n",maskimage.rows, maskimage.cols);
                                printf("res depth: %i %i\n",depthimage.rows, depthimage.cols);

                                // Remove this when this is correct:
                                cv::Mat masked_rgb = depthimage.clone();
                                masked_rgb.setTo(cv::Scalar(0, 0, 0), maskimage == 0);
								cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
								cv::imshow(		"maskimage",	maskimage);
								cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
								cv::imshow(		"rgbimage",		rgbimage );
								cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
								cv::imshow(		"depthimage",	depthimage );
                                cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
                                cv::imshow(		"mask",	masked_rgb);
                                cv::waitKey(0);

								printf("indexFrame\n");
								//sensor_msgs::CameraInfo		camera			= req.frame.camera;
								//ros::Time					capture_time	= req.frame.capture_time;
								//geometry_msgs::Pose			pose			= req.frame.pose;

								Eigen::Affine3d epose = Eigen::Affine3d::Identity();
								//tf::poseMsgToEigen(pose, epose);

								reglib::RGBDFrame * frame = new reglib::RGBDFrame(cameras[0],rgbimage, depthimage, 0, epose.matrix());
								reglib::Model * searchmodel = new reglib::Model(frame,mask);



								pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = searchmodel->getPCLcloud(1, false);
								viewer->removeAllPointClouds();
								viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
								viewer->spin();

								reglib::RegistrationRandom *	reg		= new reglib::RegistrationRandom();
								reglib::ModelUpdaterBasicFuse * mu		= new reglib::ModelUpdaterBasicFuse( searchmodel, reg);
								mu->viewer								= viewer;
                                reg->visualizationLvl					= 0;

								reglib::FusionResults fr = mu->registerModel(currentTest);
								reglib::UpdatedModels ud = mu->fuseData(&(fr), currentTest, searchmodel);

								printf("merge %i to %i\n",		currentTest->id,searchmodel->id);
								printf("new_models:     %i\n",ud.new_models.size());
								printf("updated_models: %i\n",ud.updated_models.size());
								printf("deleted_models: %i\n",ud.deleted_models.size());

								for(unsigned int j = 0; j < ud.new_models.size(); j++){
									modeldatabase->add(ud.new_models[j]);
								}

								for(unsigned int j = 0; j < ud.updated_models.size(); j++){
									modeldatabase->remove(ud.updated_models[j]);
									modeldatabase->add(ud.updated_models[j]);
								}

								bool searchmodel_merged = false;
								for(unsigned int j = 0; j < ud.deleted_models.size(); j++){
									if(ud.deleted_models[j] == searchmodel){
										searchmodel_merged = true;
									}else{
										modeldatabase->remove(ud.deleted_models[j]);
									}
									delete ud.deleted_models[j];
								}

								if(searchmodel_merged){
									frames[frame->id] = frame;
								}

								delete mu;
								delete reg;
							}
						}

						break;
					}else{
						printf("searching... timeout in %3.3f\n", +start +timelimit - getTime());
						usleep(100000);
					}
				}
			}
		}
//		exit(0);

		newmodel = 0;
		sweepid_counter++;

		//The model is added and refined in the database... its time to search in the full dabase for other similar stuff
		// TEMPORARY us fuse once
		// Get search results
		// if search fits...
		// add to db
		// else move on with our lives...
		//
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

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(0.9,0.9,0.9);

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

	ros::Subscriber sub = n.subscribe("/retrieval_result", 1, retrievalCallback);
	ROS_INFO("Ready to add recieve search results.");
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
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
