#include "ros/ros.h"
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_visualization.h>
#include "quasimodo_msgs/query_cloud.h"
#include "quasimodo_msgs/visualize_query.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/string_array.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// for logging to soma2 in mongodb
#include <soma2_msgs/SOMA2Object.h>
#include <soma_manager/SOMA2InsertObjs.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;

class logging_server {
public:
    ros::NodeHandle n;
    ros::Subscriber sub;
    string topic_input;

    logging_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("topic_input", topic_input, std::string("/retrieval_result"));
        sub = n.subscribe(topic_input, 1, &logging_server::callback, this);
    }

    void callback(const quasimodo_msgs::retrieval_query_result& res)
    {
        soma_manager::SOMA2InsertObjs::Request req;

        size_t N = res.result.retrieved_clouds.size();
        req.objects.resize(N);
        for (size_t i = 0; i < N; ++i) {
            req.objects[i].cloud = res.result.retrieved_clouds[i];
            req.objects[i].pose = res.result.retrieved_initial_poses[i].poses[0];
            req.objects[i].logtimestamp = ros::Time::now().sec;
            req.objects[i].id = res.result.retrieved_image_paths[i].strings[0];
        }

        ros::ServiceClient service = n.serviceClient<soma_manager::SOMA2InsertObjs>("/soma2/insert_objects");
        soma_manager::SOMA2InsertObjs::Response resp;
        if (service.call(req, resp)) {
            cout << "Successfully inserted queries!" << endl;
        }
        else {
            cout << "Could not connect to SOMA!" << endl;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_logging_server");

    logging_server ls(ros::this_node::getName());

    ros::spin();

    return 0;
}
