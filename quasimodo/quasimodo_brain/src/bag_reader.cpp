#include "ros/ros.h"
//#include <dynamic_object_retrieval/dynamic_retrieval.h>
//#include <dynamic_object_retrieval/visualize.h>
//#include <object_3d_benchmark/benchmark_retrieval.h>
//#include <object_3d_benchmark/benchmark_visualization.h>
#include "quasimodo_msgs/query_cloud.h"
#include "quasimodo_msgs/visualize_query.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;

class bag_server {
public:
    ros::NodeHandle n;
    ros::ServiceServer service;
    ros::Publisher image_pub;
    ros::Subscriber sub;
    string image_output;
    string topic_input;

	ros::ServiceClient model_from_frame_client;
	ros::ServiceClient fuse_models_client;
	ros::ServiceClient get_model_client;
	ros::ServiceClient index_frame_client;

	bag_server(const std::string& name){
        ros::NodeHandle pn("~");
        pn.param<std::string>("image_output", image_output, std::string("visualization_image"));
        pn.param<std::string>("topic_input", topic_input, std::string("/retrieval_result"));

        image_pub = n.advertise<sensor_msgs::Image>(image_output, 1);
		service = n.advertiseService(name, &bag_server::service_callback, this);
		sub = n.subscribe(topic_input, 100000, &bag_server::callback, this);

		model_from_frame_client	= n.serviceClient<quasimodo_msgs::model_from_frame>("model_from_frame");
		fuse_models_client		= n.serviceClient<quasimodo_msgs::fuse_models>(		"fuse_models");
		get_model_client		= n.serviceClient<quasimodo_msgs::get_model>(		"get_model");
		index_frame_client		= n.serviceClient<quasimodo_msgs::index_frame>(		"index_frame");
    }

    //sensor_msgs/PointCloud2 cloud
    //sensor_msgs/Image image
    //sensor_msgs/Image depth
    //sensor_msgs/Image mask
    //sensor_msgs/CameraInfo camera
    //int32 number_query


    //sensor_msgs/Image image
    //sensor_msgs/CameraInfo camera
    //geometry_msgs/Transform room_transform
    sensor_msgs::Image vis_img_from_msgs(const quasimodo_msgs::retrieval_query& query,
                                         const quasimodo_msgs::retrieval_result& result){
		cv_bridge::CvImagePtr cv_image_ptr;
		try {cv_image_ptr = cv_bridge::toCvCopy(query.image, sensor_msgs::image_encodings::BGR8);}
        catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

        cv_bridge::CvImagePtr cv_mask_ptr;
        try {cv_mask_ptr = cv_bridge::toCvCopy(query.mask, sensor_msgs::image_encodings::MONO8);}
        catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

		cv_bridge::CvImagePtr cv_depth_ptr;
		try {cv_depth_ptr = cv_bridge::toCvCopy(query.depth, sensor_msgs::image_encodings::MONO16);}
		catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

		cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
		cv::imshow(		"maskimage",	cv_mask_ptr->image);
		cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
		cv::imshow(		"rgbimage",		cv_image_ptr->image );
		cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
		cv::imshow(		"depthimage",	cv_depth_ptr->image );

        Eigen::Affine3d e;
        tf::transformMsgToEigen(query.room_transform, e);
        Eigen::Matrix4f T = e.matrix().cast<float>();
        T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;

        string query_label = "Query Image";
        vector<string> dummy_labels;
        for (int i = 0; i < result.retrieved_clouds.size(); ++i) {
            dummy_labels.push_back(string("result") + to_string(i));
        }

		char buff [1024];
		sprintf(buff,"rgbimage");
		cv::namedWindow(buff,		cv::WINDOW_AUTOSIZE );
		cv::imshow(		buff,		cv_image_ptr->image );
		unsigned char retval = cv::waitKey(0);
		if(retval != 27){
			printf("Keep\n");

			cv::Mat rgbimage_flip;
			cv::Mat depthimage_flip;
			cv::Mat maskimage_flip;


			std::vector<cv::Mat> rgbs;
			std::vector<cv::Mat> depths;
			std::vector<cv::Mat> masks;

			rgbs.push_back(cv_image_ptr->image);
			masks.push_back(cv_mask_ptr->image);
			depths.push_back(cv_depth_ptr->image);
/*
			cv::flip(cv_image_ptr->image, rgbimage_flip, 1);
			cv::flip(cv_depth_ptr->image, depthimage_flip, 1);
			cv::flip(cv_mask_ptr->image, maskimage_flip, 1);
			rgbs.push_back(rgbimage_flip);
			depths.push_back(depthimage_flip);
			masks.push_back(maskimage_flip);
*/
			for(int i = 0; i < result.retrieved_images.size(); i++){
				printf("i: %i\n",i);
				for(int j = 0; j < 2 && j < result.retrieved_images[i].images.size(); j++){
					printf("i: %i  j: %i\n",i,j);

					cv_bridge::CvImagePtr ret_image_ptr;
					try {ret_image_ptr = cv_bridge::toCvCopy(result.retrieved_images[i].images[j], sensor_msgs::image_encodings::BGR8);}
					catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

					cv_bridge::CvImagePtr ret_mask_ptr;
					try {ret_mask_ptr = cv_bridge::toCvCopy(result.retrieved_masks[i].images[j], sensor_msgs::image_encodings::MONO8);}
					catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

					cv_bridge::CvImagePtr ret_depth_ptr;
					try {ret_depth_ptr = cv_bridge::toCvCopy(result.retrieved_depths[i].images[j], sensor_msgs::image_encodings::MONO16);}
					catch (cv_bridge::Exception& e) {ROS_ERROR("cv_bridge exception: %s", e.what());exit(-1);}

					rgbs.push_back(ret_image_ptr->image);
					masks.push_back(ret_mask_ptr->image);
					depths.push_back(ret_depth_ptr->image);

					cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
					cv::imshow(		"maskimage",	masks.back());
					cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
					cv::imshow(		"rgbimage",		rgbs.back() );
					cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
					cv::imshow(		"depthimage",	depths.back() );
					cv::waitKey(0);


					cv::flip(ret_image_ptr->image, rgbimage_flip, 1);
					cv::flip(ret_depth_ptr->image, depthimage_flip, 1);
					cv::flip(ret_mask_ptr->image, maskimage_flip, 1);
					rgbs.push_back(rgbimage_flip);
					depths.push_back(depthimage_flip);
					masks.push_back(maskimage_flip);

				}
			}


			while(true){
				char str [80];
				printf ("build model?");
				scanf ("%79s",str);
				int nr_todo = atoi(str);
				if(str[0] == 'q'){exit(0);}

				std::vector<cv::Mat> current_rgbs;
				std::vector<cv::Mat> current_depths;
				std::vector<cv::Mat> current_masks;

				for (unsigned int i = 0; i < nr_todo && i < rgbs.size(); i++){
					printf("adding frame %i\n",i);

					//int id = rand()%rgbs.size();
					int id = (i)%rgbs.size();

					cv::Mat maskimage	= masks[id];
					cv::Mat rgbimage	= rgbs[id];
					cv::Mat depthimage	= depths[id];


					cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
					cv::imshow(		"maskimage",	maskimage);
					cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
					cv::imshow(		"rgbimage",		rgbimage );
					cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
					cv::imshow(		"depthimage",	depthimage );
					unsigned char retval = cv::waitKey(0);
					printf("retval: %i\n",retval);
					if(retval != 27){
						current_rgbs.push_back(rgbimage);
						current_depths.push_back(depthimage);
						current_masks.push_back(maskimage);
					}
				}


				for (unsigned int i = 0; i < current_rgbs.size(); i++){
					printf("start adding frame %i\n",i);

					cv::Mat maskimage	= current_masks[i];
					cv::Mat rgbimage	= current_rgbs[i];
					cv::Mat depthimage	= current_depths[i];

					geometry_msgs::Pose		pose;
					tf::poseEigenToMsg (Eigen::Affine3d::Identity(), pose);

					cv_bridge::CvImage rgbBridgeImage;
					rgbBridgeImage.image = rgbimage;
					rgbBridgeImage.encoding = "bgr8";

					cv_bridge::CvImage depthBridgeImage;
					depthBridgeImage.image = depthimage;
					depthBridgeImage.encoding = "mono16";

					quasimodo_msgs::index_frame ifsrv;
					ifsrv.request.frame.capture_time = ros::Time();
					ifsrv.request.frame.pose		= pose;
					ifsrv.request.frame.frame_id	= -1;
					ifsrv.request.frame.rgb			= *(rgbBridgeImage.toImageMsg());
					ifsrv.request.frame.depth		= *(depthBridgeImage.toImageMsg());

					if (index_frame_client.call(ifsrv)){//Add frame to model server
						int frame_id = ifsrv.response.frame_id;
						ROS_INFO("frame_id%i", frame_id );
					 }else{ROS_ERROR("Failed to call service index_frame");}

					printf("stop adding frame %i\n",i);
				}

				for (unsigned int i = 0; i < nr_todo && i < rgbs.size(); i++){
					printf("start adding mask %i\n",i);
					cv::Mat mask	= current_masks[i];

					cv_bridge::CvImage maskBridgeImage;
					maskBridgeImage.image = mask;
					maskBridgeImage.encoding = "mono8";

					quasimodo_msgs::model_from_frame mff;
					mff.request.mask = *(maskBridgeImage.toImageMsg());
					mff.request.frame_id = i;

					if (model_from_frame_client.call(mff)){//Build model from frame
						int model_id = mff.response.model_id;
						if(model_id > 0){
							ROS_INFO("model_id%i", model_id );
						}
					}else{ROS_ERROR("Failed to call service index_frame");}

					printf("stop adding mask %i\n",i);
				}
			}
			//exit(0);
			/*
			current_rgbs.push_back(rgbimage);
			current_depths.push_back(depthimage);
			current_masks.push_back(maskimage);
			*/

			while(true){
				char str [80];
				printf ("build model?");
				scanf ("%79s",str);
				int nr_todo = atoi(str);
				if(str[0] == 'q'){exit(0);}

				std::vector<cv::Mat> current_rgbs;
				std::vector<cv::Mat> current_depths;
				std::vector<cv::Mat> current_masks;

				for (unsigned int i = 0; i < nr_todo && i < rgbs.size(); i++){
					printf("adding frame %i\n",i);

					int id = (i)%rgbs.size();

					cv::Mat maskimage	= masks[id];
					cv::Mat rgbimage	= rgbs[id];
					cv::Mat depthimage	= depths[id];


					char buff [1024];
					sprintf(buff,"rgbimage");

					//cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
					//cv::imshow(		"maskimage",	maskimage);
					cv::namedWindow(buff,		cv::WINDOW_AUTOSIZE );
					cv::imshow(		buff,		rgbimage );
					//cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
					//cv::imshow(		"depthimage",	depthimage );
					unsigned char retval = cv::waitKey(0);
					printf("retval: %i\n",retval);
					if(retval != 27){
						current_rgbs.push_back(rgbimage);
						current_depths.push_back(depthimage);
						current_masks.push_back(maskimage);
					}
				}


				for (unsigned int i = 0; i < current_rgbs.size(); i++){
					printf("start adding frame %i\n",i);

					cv::Mat maskimage	= current_masks[i];
					cv::Mat rgbimage	= current_rgbs[i];
					cv::Mat depthimage	= current_depths[i];
					/*
					cv::namedWindow("maskimage",	cv::WINDOW_AUTOSIZE);
					cv::imshow(		"maskimage",	maskimage);
					cv::namedWindow("rgbimage",		cv::WINDOW_AUTOSIZE );
					cv::imshow(		"rgbimage",		rgbimage );
					cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE );
					cv::imshow(		"depthimage",	depthimage );
					cv::waitKey(0);
					*/
					geometry_msgs::Pose		pose;
					tf::poseEigenToMsg (Eigen::Affine3d::Identity(), pose);

					cv_bridge::CvImage rgbBridgeImage;
					rgbBridgeImage.image = rgbimage;
					rgbBridgeImage.encoding = "bgr8";

					cv_bridge::CvImage depthBridgeImage;
					depthBridgeImage.image = depthimage;
					depthBridgeImage.encoding = "mono16";

					quasimodo_msgs::index_frame ifsrv;
					ifsrv.request.frame.capture_time = ros::Time();
					ifsrv.request.frame.pose		= pose;
					ifsrv.request.frame.frame_id	= -1;
					ifsrv.request.frame.rgb			= *(rgbBridgeImage.toImageMsg());
					ifsrv.request.frame.depth		= *(depthBridgeImage.toImageMsg());

					if (index_frame_client.call(ifsrv)){//Add frame to model server
						int frame_id = ifsrv.response.frame_id;
						ROS_INFO("frame_id%i", frame_id );
					 }else{ROS_ERROR("Failed to call service index_frame");}

					printf("stop adding frame %i\n",i);
				}

				for (unsigned int i = 0; i < nr_todo && i < rgbs.size(); i++){
					printf("start adding mask %i\n",i);
					cv::Mat mask	= current_masks[i];

					cv_bridge::CvImage maskBridgeImage;
					maskBridgeImage.image = mask;
					maskBridgeImage.encoding = "mono8";

					quasimodo_msgs::model_from_frame mff;
					mff.request.mask = *(maskBridgeImage.toImageMsg());
					mff.request.frame_id = i;

					if (model_from_frame_client.call(mff)){//Build model from frame
						int model_id = mff.response.model_id;
						if(model_id > 0){
							ROS_INFO("model_id%i", model_id );
						}
					}else{ROS_ERROR("Failed to call service index_frame");}

					printf("stop adding mask %i\n",i);
				}

			}

		}else{
			printf("throw\n");
		}


		//cv::Mat visualization = benchmark_retrieval::make_visualization_image(cv_ptr->image, query_label, retrieved_clouds, dummy_labels, T);
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
		//cv_pub_ptr->image = visualization;
        cv_pub_ptr->encoding = "bgr8";

        return *cv_pub_ptr->toImageMsg();
    }

    bool service_callback(quasimodo_msgs::visualize_query::Request& req,
                          quasimodo_msgs::visualize_query::Response& res)
    {
        res.image = vis_img_from_msgs(req.query, req.result);

        image_pub.publish(res.image);

        return true;
    }

    void callback(const quasimodo_msgs::retrieval_query_result& res)
    {
        sensor_msgs::Image image = vis_img_from_msgs(res.query, res.result);

        image_pub.publish(image);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_visualization_service");

	bag_server vs(ros::this_node::getName());

    ros::spin();

    return 0;
}
