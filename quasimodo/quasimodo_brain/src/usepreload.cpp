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

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "use_rares_client");
    ros::NodeHandle n;

    ros::ServiceClient model_from_frame_client = n.serviceClient<quasimodo_msgs::model_from_frame>("model_from_frame");
    ros::ServiceClient fuse_models_client = n.serviceClient<quasimodo_msgs::fuse_models>("fuse_models");
	ros::ServiceClient get_model_client = n.serviceClient<quasimodo_msgs::get_model>("get_model");

    while(true){
        char str [80];
        printf ("build model?");
        scanf ("%79s",str);
        for (unsigned int i = 0; i < 10; i++){
            cv::Mat mask;
            mask.create(480,640,CV_8UC1);
            unsigned char * maskdata = (unsigned char *)mask.data;
            for(unsigned int i = 0; i < 640*480; i++){maskdata[i] = 255;}

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
                    //Fuse to first model

                    Eigen::Affine3d mat2 = Eigen::Affine3d::Identity();
                    geometry_msgs::Pose		pose2;
                    tf::poseEigenToMsg (mat2, pose2);

                    quasimodo_msgs::fuse_models fm;
                    fm.request.model1 = 0;
                    fm.request.model2 = model_id;
                    fm.request.pose		= pose2;

                    if (fuse_models_client.call(fm)){//Fuse do first model
                        int new_model_id = fm.response.fused_model;
                        printf("new model id: %i\n",new_model_id);

						quasimodo_msgs::get_model gm;
						gm.request.model_id = fm.response.fused_model;

						if (get_model_client.call(gm)){//Get the model to do something with...
							printf("got the model back\n");
						}
                    }
                }
            }else{ROS_ERROR("Failed to call service index_frame");}
        }
    }
}
