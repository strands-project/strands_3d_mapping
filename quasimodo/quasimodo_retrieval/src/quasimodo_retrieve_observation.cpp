#include <ros/ros.h>
#include <quasimodo_msgs/model.h>
#include <std_msgs/String.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

using PointT = pcl::PointXYZRGB;

using namespace std;

class retrieve_observation_server {
public:

    using object_data = semantic_map_load_utilties::DynamicObjectData<PointT>;

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    retrieve_observation_server(const std::string& name)
    {
        ros::NodeHandle pn("~");

        string  input;
        pn.param<std::string>("input", input, std::string("/object_learning/learned_object_xml"));
        string output;
        pn.param<std::string>("output", output, std::string("/models/new"));

        pub = n.advertise<quasimodo_msgs::model>(output, 1);
        sub = n.subscribe(input, 1, &retrieve_observation_server::callback, this);
    }

    void convert_to_img_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = cv_image;
        cv_pub_ptr->encoding = "bgr8";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    void convert_to_depth_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = 5*cv_image;
        cv_pub_ptr->encoding = "mono16";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    void convert_to_mask_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = cv_image;
        cv_pub_ptr->encoding = "mono8";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    quasimodo_msgs::model dynamic_object_to_quasimodo_model(object_data& data)
    {
        /*
        struct DynamicObjectData
        {
            tf::StampedTransform                                        transformToGlobal; // camera frame to map frame
            tf::StampedTransform                                        calibratedTransform; // registration transform for the intermediate cloud in which this object lies
            boost::shared_ptr<pcl::PointCloud<PointType>>               intermediateCloud; // this is the intermediate cloud where the object can be found (corresponding to the objectScanIndices mask)
            boost::shared_ptr<pcl::PointCloud<PointType>>               objectCloud;
            cv::Mat                                                     objectRGBImage;
            cv::Mat                                                     objectDepthImage;
            std::string                                                 objectLabel;
            std::vector<int>                                            objectScanIndices;
            boost::posix_time::ptime                                    time;
            std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  vAdditionalViews;
            std::vector<tf::StampedTransform>                           vAdditionalViewsTransforms;
            std::vector<tf::StampedTransform>                           vAdditionalViewsTransformsRegistered;
            tf::StampedTransform                                        additionalViewsTransformToObservation;
            std::vector<std::vector<int>>                               vAdditionalViewMaskIndices;
            std::vector<cv::Mat>                                        vAdditionalViewMaskImages;
        };

        --->

        model.msg
        uint64 model_id
        geometry_msgs/Pose[] local_poses
        rgbd_frame[] frames
        sensor_msgs/Image[] masks

        rgbd_frame.msg
        sensor_msgs/CameraInfo camera
        time capture_time
        geometry_msgs/Pose pose
        sensor_msgs/Image rgb
        sensor_msgs/Image depth
        uint64 frame_id
        */

        size_t N = data.vAdditionalViews.size();

        quasimodo_msgs::model model;
        model.model_id = 0; // random number!
        model.local_poses.resize(N);
        model.frames.resize(N);
        model.masks.resize(N);

        for (size_t i = 0; i < N; ++i) {
            pair<cv::Mat, cv::Mat> images = SimpleXMLParser<PointT>::createRGBandDepthFromPC(data.vAdditionalViews[i]);
            convert_to_img_msg(images.first, model.frames[i].rgb);
            convert_to_depth_msg(images.second, model.frames[i].depth);

            cv::Mat mask = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
            for (int index : data.vAdditionalViewMaskIndices[i]) {
                int y = index / 640;
                int x = index - y*640;
                mask.at<uchar>(y, x) = 255;
            }
            //convert_to_mask_msg(data.vAdditionalViewMaskImages[i], model.masks[i]);
            convert_to_mask_msg(mask, model.masks[i]);
            model.frames[i].capture_time = ros::Time::now();
            //tf::poseTFToMsg(data.vAdditionalViewsTransformsRegistered[i], model.frames[i].pose);
            tf::poseTFToMsg(data.vAdditionalViewsTransformsRegistered[i], model.local_poses[i]);
            model.frames[i].frame_id = 0; // random number!
        }

        return model;
    }

    void callback(const std_msgs::String::ConstPtr& model)
    {
        string object_xml_path = model->data;
        object_data data = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointT>(object_xml_path);
        quasimodo_msgs::model model_msg = dynamic_object_to_quasimodo_model(data);
        pub.publish(model_msg);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_retrieve_observation");

    retrieve_observation_server ro(ros::this_node::getName());

    ros::spin();

    return 0;
}
