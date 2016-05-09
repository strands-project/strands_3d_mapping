/*
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

#include "metaroom_xml_parser/load_utilities.h"
//#include "metaroom_xml_parser/simple_summary_parser.h"



#include <dirent.h>


using namespace std;

using PointT = pcl::PointXYZRGB;missunderstoo
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "ros/ros.h"
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

using pcl::visualization::PointCloudColorHandlerCustom;

ros::ServiceClient model_from_frame_client;
ros::ServiceClient fuse_models_client;
ros::ServiceClient get_model_client;
ros::ServiceClient index_frame_client;

std::vector< std::vector<cv::Mat> > rgbs;
std::vector< std::vector<cv::Mat> > depths;
std::vector< std::vector<cv::Mat> > masks;
std::vector< std::vector<tf::StampedTransform > >tfs;
std::vector< std::vector<Eigen::Matrix4f> > initposes;

std::vector<Eigen::Matrix4f> getRegisteredViewPoses(const std::string& poses_file, const int& no_transforms){
    std::vector<Eigen::Matrix4f> toRet;
    ifstream in(poses_file);
    if (!in.is_open()){
        cout<<"ERROR: cannot find poses file "<<poses_file<<endl;
        return toRet;
    }
    cout<<"Loading additional view registered poses from "<<poses_file<<endl;

    for (int i=0; i<no_transforms+1; i++){
        Eigen::Matrix4f transform;
        float temp;
        for (size_t j=0; j<4; j++){
            for (size_t k=0; k<4; k++){
                in >> temp;
                transform(j,k) = temp;
            }
        }
        toRet.push_back(transform);
    }
    return toRet;
}

void load(std::string path);
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	load(msg->data.c_str());
}

void load(std::string path)
{
	ObjectData object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(path);

    printf("number of inds: %i",int(object.objectScanIndices.size()));
    printf("AVs: %i\n",int(object.vAdditionalViews.size()));
    if (!object.vAdditionalViews.size()){return;}



    std::vector<cv::Mat> viewrgbs;
    std::vector<cv::Mat> viewdepths;
    std::vector<cv::Mat> viewmasks;
    std::vector<tf::StampedTransform > viewtfs;
    std::vector<Eigen::Matrix4f> viewposes;

    int step = std::max(1,int(0.5+double(object.vAdditionalViews.size())/10.0));

    for (unsigned int i=0; i < 2000 && i<object.vAdditionalViews.size(); i+=step){
        CloudPtr cloud = object.vAdditionalViews[i];

        cv::Mat mask;
        mask.create(cloud->height,cloud->width,CV_8UC1);
        unsigned char * maskdata = (unsigned char *)mask.data;

        cv::Mat rgb;
        rgb.create(cloud->height,cloud->width,CV_8UC3);
        unsigned char * rgbdata = (unsigned char *)rgb.data;

        cv::Mat depth;
        depth.create(cloud->height,cloud->width,CV_16UC1);
        unsigned short * depthdata = (unsigned short *)depth.data;

        unsigned int nr_data = cloud->height * cloud->width;
        for(unsigned int j = 0; j < nr_data; j++){
            maskdata[j] = 0;
            PointType p = cloud->points[j];
            rgbdata[3*j+0]	= p.b;
            rgbdata[3*j+1]	= p.g;
            rgbdata[3*j+2]	= p.r;
            depthdata[j]	= short(5000.0 * p.z);
        }

        std::vector<int> & inds = object.vAdditionalViewMaskIndices[i];

        for(unsigned int j = 0; j < inds.size(); j++){
            maskdata[inds[j]] = 255;
        }

        cout<<"Processing AV "<<i<<endl;

        viewrgbs.push_back(rgb);
        viewdepths.push_back(depth);
        viewmasks.push_back(mask);
        viewtfs.push_back(object.vAdditionalViewsTransformsRegistered[i]);


//        cv::namedWindow("rgbimage",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"rgbimage",	rgb);
//        cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"depthimage",	depth);
//        cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
//        cv::imshow(		"mask",	mask);
//        cv::imshow(		"raresmask",object.vAdditionalViewMaskImages[i]);
//        cv::waitKey(0);
    }


    if(viewrgbs.size() > 0){
        std::vector<int> fid;
        std::vector<int> fadded;
        for(unsigned int j = 0; j < viewrgbs.size(); j++){
            cv::Mat rgbimage		= viewrgbs[j];
            cv::Mat depthimage		= viewdepths[j];
            tf::StampedTransform tf	= viewtfs[j];
            //            Eigen::Matrix4f m		= viewposes[j];
            geometry_msgs::TransformStamped tfstmsg;
            tf::transformStampedTFToMsg (tf, tfstmsg);

            geometry_msgs::Transform tfmsg = tfstmsg.transform;

            printf("start adding frame %i\n",j);

            //            Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::Affine3f(m).rotation());

            geometry_msgs::Pose		pose;
            pose.orientation		= tfmsg.rotation;
            pose.position.x		= tfmsg.translation.x;
            pose.position.y		= tfmsg.translation.y;
            pose.position.z		= tfmsg.translation.z;
            //            pose.orientation.x	= q.x();
            //            pose.orientation.y	= q.y();
            //            pose.orientation.z	= q.z();
            //            pose.orientation.w	= q.w();
            //            pose.position.x		= m(0,3);
            //            pose.position.y		= m(1,3);
            //            pose.position.z		= m(2,3);


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
                fadded.push_back(j);
                fid.push_back(frame_id);
                ROS_INFO("frame_id%i", frame_id );
            }else{ROS_ERROR("Failed to call service index_frame");}
        }

        for(unsigned int j = 0; j < fadded.size(); j++){
            cv::Mat mask	= viewmasks[fadded[j]];

            cv_bridge::CvImage maskBridgeImage;
            maskBridgeImage.image = mask;
            maskBridgeImage.encoding = "mono8";

            quasimodo_msgs::model_from_frame mff;
            mff.request.mask		= *(maskBridgeImage.toImageMsg());
            mff.request.isnewmodel	= (j == (fadded.size()-1));
            mff.request.frame_id	= fid[j];

            if (model_from_frame_client.call(mff)){//Build model from frame
                int model_id = mff.response.model_id;
                if(model_id > 0){
                    ROS_INFO("model_id%i", model_id );
                }
            }else{ROS_ERROR("Failed to call service index_frame");}
        }
    }

}

void chatterCallback2(const std_msgs::String::ConstPtr& msg)
{

    ROS_INFO("I heard: [%s]", msg->data.c_str());

    vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(msg->data.c_str());

    for (auto sweep_xml : sweep_xmls) {
        int slash_pos = sweep_xml.find_last_of("/");
        std::string sweep_folder = sweep_xml.substr(0, slash_pos) + "/";
        printf("folder: %s\n",sweep_folder.c_str());

        QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
        vector<ObjectData> objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(sweep_folder+"room.xml");

        int object_id;
        for (unsigned int i=0; i< objects.size(); i++){
            if (objects[i].objectScanIndices.size() != 0){object_id = i;}
        }
        int index = objectFiles[object_id].toStdString().find_last_of(".");
        string object_root_name = objectFiles[object_id].toStdString().substr(0,index);

        for (auto object : objects){
            if (!object.objectScanIndices.size()){continue;}

            printf("%i\n",int(object.vAdditionalViews.size()));

            std::vector<cv::Mat> viewrgbs;
            std::vector<cv::Mat> viewdepths;
            std::vector<cv::Mat> viewmasks;
            std::vector<tf::StampedTransform > viewtfs;
            std::vector<Eigen::Matrix4f> viewposes;
            //std::stringstream ss_pose_file; ss_pose_file<<sweep_folder<<"/object_"<<object_id<<"/";
            char buf [1024];
            sprintf(buf,"%s/object_%i/poses.txt",sweep_folder.c_str(),object_id);
            std::vector<Eigen::Matrix4f> poses = getRegisteredViewPoses(std::string(buf), object.vAdditionalViews.size());
            cout<<"Loaded "<<poses.size()<<" registered poses."<<endl;

            int step = std::max(1,int(0.5+double(object.vAdditionalViews.size())/10.0));
            for (unsigned int i=0; i < 2000 && i<object.vAdditionalViews.size(); i+=step){
                CloudPtr cloud = object.vAdditionalViews[i];

                cv::Mat mask;
                mask.create(cloud->height,cloud->width,CV_8UC1);
                unsigned char * maskdata = (unsigned char *)mask.data;

                cv::Mat rgb;
                rgb.create(cloud->height,cloud->width,CV_8UC3);
                unsigned char * rgbdata = (unsigned char *)rgb.data;

                cv::Mat depth;
                depth.create(cloud->height,cloud->width,CV_16UC1);
                unsigned short * depthdata = (unsigned short *)depth.data;

                unsigned int nr_data = cloud->height * cloud->width;
                for(unsigned int j = 0; j < nr_data; j++){
                    maskdata[j] = 0;
                    PointType p = cloud->points[j];
                    rgbdata[3*j+0]	= p.b;
                    rgbdata[3*j+1]	= p.g;
                    rgbdata[3*j+2]	= p.r;
                    depthdata[j]	= short(5000.0 * p.z);
                }

                cout<<"Processing AV "<<i<<endl;

                stringstream av_ss; av_ss<<object_root_name; av_ss<<"_additional_view_"; av_ss<<i; av_ss<<".txt";
                string complete_av_mask_name = sweep_folder + av_ss.str();
                ifstream av_mask_in(complete_av_mask_name);
                if (!av_mask_in.is_open()){
                    cout<<"COULD NOT FIND AV MASK FILE "<<complete_av_mask_name<<endl;
                    continue;
                }

                CloudPtr av_mask(new Cloud);

                int av_index;
                while (av_mask_in.is_open() && !av_mask_in.eof()){
                    av_mask_in>>av_index;
                    av_mask->push_back(object.vAdditionalViews[i]->points[av_index]);
                    maskdata[av_index] = 255;
                }

                viewrgbs.push_back(rgb);
                viewdepths.push_back(depth);
                viewmasks.push_back(mask);
                viewtfs.push_back(object.vAdditionalViewsTransforms[i]);
                viewposes.push_back(poses[i+1]);

                cv::namedWindow("rgbimage",	cv::WINDOW_AUTOSIZE);
                cv::imshow(		"rgbimage",	rgb);
                cv::namedWindow("depthimage",	cv::WINDOW_AUTOSIZE);
                cv::imshow(		"depthimage",	depth);
                cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
                cv::imshow(		"mask",	mask);
                cv::waitKey(30);
            }

            if(viewrgbs.size() > 0){
                std::vector<int> fid;
                std::vector<int> fadded;
                for(unsigned int j = 0; j < viewrgbs.size(); j++){
                    cv::Mat rgbimage		= viewrgbs[j];
                    cv::Mat depthimage		= viewdepths[j];
                    tf::StampedTransform tf	= viewtfs[j];
                    Eigen::Matrix4f m		= viewposes[j];
                    geometry_msgs::TransformStamped tfstmsg;
                    tf::transformStampedTFToMsg (tf, tfstmsg);

                    geometry_msgs::Transform tfmsg = tfstmsg.transform;

                    printf("start adding frame %i\n",j);

                    Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::Affine3f(m).rotation());

                    geometry_msgs::Pose		pose;
                    //		  pose.orientation		= tfmsg.rotation;
                    //		  pose.position.x		= tfmsg.translation.x;
                    //		  pose.position.y		= tfmsg.translation.y;
                    //		  pose.position.z		= tfmsg.translation.z;
                    pose.orientation.x	= q.x();
                    pose.orientation.y	= q.y();
                    pose.orientation.z	= q.z();
                    pose.orientation.w	= q.w();
                    pose.position.x		= m(0,3);
                    pose.position.y		= m(1,3);
                    pose.position.z		= m(2,3);


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
                        fadded.push_back(j);
                        fid.push_back(frame_id);
                        ROS_INFO("frame_id%i", frame_id );
                    }else{ROS_ERROR("Failed to call service index_frame");}
                }

                for(unsigned int j = 0; j < fadded.size(); j++){
                    cv::Mat mask	= viewmasks[fadded[j]];

                    cv_bridge::CvImage maskBridgeImage;
                    maskBridgeImage.image = mask;
                    maskBridgeImage.encoding = "mono8";

                    quasimodo_msgs::model_from_frame mff;
                    mff.request.mask		= *(maskBridgeImage.toImageMsg());
                    mff.request.isnewmodel	= (j == (fadded.size()-1));
                    mff.request.frame_id	= fid[j];

                    if (model_from_frame_client.call(mff)){//Build model from frame
                        int model_id = mff.response.model_id;
                        if(model_id > 0){
                            ROS_INFO("model_id%i", model_id );
                        }
                    }else{ROS_ERROR("Failed to call service index_frame");}
                }
            }
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "use_rares_client");
    ros::NodeHandle n;

    std::string listentopic = "/some/stupid/topic";
    if(argc == 2){listentopic = std::string(argv[1]);}
    else{printf("robot listener: incorrect number of args. Should take a single arguement witha topicname!\n");}

    model_from_frame_client	= n.serviceClient<quasimodo_msgs::model_from_frame>("model_from_frame");
    fuse_models_client		= n.serviceClient<quasimodo_msgs::fuse_models>(		"fuse_models");
	get_model_client		= n.serviceClient<quasimodo_msgs::get_model>(	"get_model");
    index_frame_client		= n.serviceClient<quasimodo_msgs::index_frame>(		"index_frame");

    ros::Subscriber sub = n.subscribe(listentopic, 1000, chatterCallback);

    ros::spin();
}
