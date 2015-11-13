#include "ros/ros.h"
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include "quasimodo_msgs/query_cloud.h"
#include <pcl_ros/point_cloud.h>

using namespace std;

using HistT = pcl::Histogram<250>;

template<typename VocabularyT>
class retrieval_server {
public:
    ros::NodeHandle n;
    ros::ServiceServer service;

    boost::filesystem::path vocabulary_path;

    VocabularyT vt;
    dynamic_object_retrieval::vocabulary_summary summary;

    Eigen::Matrix3f K;

    retrieval_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        string string_path;
        pn.param<std::string>("vocabulary_path", string_path, std::string(""));
        vocabulary_path = boost::filesystem::path(string_path);

        summary.load(vocabulary_path);

        if (vt.empty()) {
            dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
            vt.set_min_match_depth(3);
            vt.compute_normalizing_constants();
        }

        service = n.advertiseService(name, &retrieval_server::service_callback, this);
    }

    bool service_callback(quasimodo_msgs::query_cloud::Request& req,
                          quasimodo_msgs::query_cloud::Response& res)
    {
        using result_type = vector<pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(req.cloud, *cloud);

        result_type retrieved_paths;
        result_type reweighted_paths;
        VocabularyT vt;
        tie(retrieved_paths, reweighted_paths) = dynamic_object_retrieval::query_reweight_vocabulary(vt, cloud, K, 10, vocabulary_path, summary, false);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_retrieval_service");

    retrieval_server<vocabulary_tree<HistT, 8> > rs(ros::this_node::getName());

    ros::spin();

    return 0;
}
