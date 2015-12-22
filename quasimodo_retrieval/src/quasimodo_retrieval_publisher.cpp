#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>

#include <k_means_tree/k_means_tree.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include "ros/ros.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

template<typename VocabularyT>
class retrieval_publisher {
public:
    ros::NodeHandle n;
    ros::Publisher pub;

    boost::filesystem::path vocabulary_path;
    boost::filesystem::path data_path;
    int32_t number_query;
    string retrieval_output;

    VocabularyT vt;
    dynamic_object_retrieval::vocabulary_summary summary;

    retrieval_publisher(const std::string& name)
    {
        ros::NodeHandle pn("~");

        string vocabulary_string;
        pn.param<std::string>("vocabulary_path", vocabulary_string, std::string(""));
        vocabulary_path = boost::filesystem::path(vocabulary_string);

        string data_string;
        pn.param<std::string>("data_path", data_string, std::string(""));
        data_path = boost::filesystem::path(data_string);

        pn.param<int32_t>("number_query", number_query, 10);

        pn.param<std::string>("output", retrieval_output, std::string("retrieval_result"));

        summary.load(vocabulary_path);

        if (vt.empty()) {
            dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
            vt.set_min_match_depth(3);
            vt.compute_normalizing_constants();
        }

        pub = n.advertise<quasimodo_msgs::retrieval_query_result>(retrieval_output, 1);

        vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
        for (const string& xml : folder_xmls) {
            run_retrieval(xml);
        }
    }

    tuple<vector<cv::Mat>, vector<cv::Mat>, vector<cv::Mat>, vector<int> >
    generate_images_for_object(const CloudT::Ptr& cloud, const Eigen::Matrix3f& K,
        const boost::filesystem::path& sweep_xml,
        const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms)
    {
        tuple<vector<cv::Mat>, vector<cv::Mat>, vector<cv::Mat>, vector<int> > images;

        int height = 480;
        int width = 640;

        for (int i = 0; i < transforms.size(); ++i) {
            cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
            int sum = 0;
            for (const PointT& p : cloud->points) {
                Eigen::Vector4f q = transforms[i]*p.getVector4fMap();
                if (q(2)/q(3) < 0) {
                    continue;
                }
                Eigen::Vector3f r = K*q.head<3>();
                int x = int(r(0)/r(2));
                int y = int(r(1)/r(2));
                if (x >= width || x < 0 || y >= height || y < 0) {
                    continue;
                }
                mask.at<uint8_t>(y, x) = 255;
                ++sum;
            }
            if (sum < 100) {
                continue;
            }

            int erosion_size = 4;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size(2*erosion_size + 1, 2*erosion_size+1),
                                       cv::Point(erosion_size, erosion_size));
            cv::dilate(mask, mask, element);
            cv::erode(mask, mask, element);


            get<0>(images).push_back(mask);
            get<1>(images).push_back(benchmark_retrieval::sweep_get_rgb_at(sweep_xml, i));
            get<2>(images).push_back(benchmark_retrieval::sweep_get_depth_at(sweep_xml, i));
            get<3>(images).push_back(i);

            /*
            cv::imshow("Mask", get<0>(images).back());
            cv::imshow("Rgb", get<1>(images).back());
            cv::imshow("Depth", get<2>(images).back());
            cv::waitKey();
            */
        }

        // now probably do some dilation on the masks

        return images;
    }

    void run_retrieval(const string& sweep_xml)
    {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

        Eigen::Matrix3f K;
        vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
        sensor_msgs::CameraInfo camera_info;
        tie(K, transforms, camera_info) = benchmark_retrieval::get_camera_info_and_transforms(sweep_xml);
        CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(sweep_xml);
        Eigen::Matrix4f T = benchmark_retrieval::get_global_camera_rotation(labels);

        for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
            CloudT::Ptr query_cloud;
            string query_label;
            cv::Mat query_image;
            cv::Mat query_mask;
            size_t scan_index;
            tie(query_cloud, query_label, query_image, query_mask, scan_index) = tup;
            cv::Mat query_depth = benchmark_retrieval::sweep_get_depth_at(sweep_xml, scan_index);
            CloudT::Ptr refined_query = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, transforms[scan_index], K);

            vector<CloudT::Ptr> retrieved_clouds;
            vector<boost::filesystem::path> sweep_paths;
            auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth,
                                                                               K, number_query, vocabulary_path, summary, false);
            tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

            vector<float> scores;
            for (auto s : results.first) {
                scores.push_back(s.second.score);
            }

            vector<vector<Eigen::Matrix4f>, Eigen::aligned_allocator<Eigen::Matrix4f > > initial_poses;
            vector<vector<cv::Mat> > masks(retrieved_clouds.size());
            vector<vector<cv::Mat> > images(retrieved_clouds.size());
            vector<vector<cv::Mat> > depths(retrieved_clouds.size());
            vector<vector<string> > paths(retrieved_clouds.size());
            for (int i = 0; i < retrieved_clouds.size(); ++i) {
                vector<int> inds;
                auto sweep_data = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_paths[i].string(), std::vector<std::string>{"RoomIntermediateCloud"}, false, false);
                vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > sweep_transforms;
                for (tf::StampedTransform& t : sweep_data.vIntermediateRoomCloudTransformsRegistered) {
                    Eigen::Affine3d e;
                    tf::transformTFToEigen(t, e);
                    sweep_transforms.push_back(e.inverse().matrix().cast<float>());
                }
                tie(masks[i], images[i], depths[i], inds) = generate_images_for_object(retrieved_clouds[i], K, sweep_paths[i], sweep_transforms);
                for (int j = 0; j < inds.size(); ++j) {
                    paths[i].push_back(sweep_paths[i].string() + " " + to_string(inds[j]));
                }
            }

            cv::Mat full_query_image = benchmark_retrieval::sweep_get_rgb_at(sweep_xml, scan_index);
            quasimodo_msgs::retrieval_query_result result = construct_msgs(refined_query, full_query_image, query_depth, query_mask,
                                                                           camera_info, labels.transformToGlobal, retrieved_clouds, initial_poses,
                                                                           images, depths, masks, paths, scores);
            pub.publish(result);

            ros::spinOnce();
        }
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
        cv_pub_ptr->image = cv_image;
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

    quasimodo_msgs::retrieval_query_result construct_msgs(const CloudT::Ptr& query_cloud,
                                                          const cv::Mat& query_image,
                                                          const cv::Mat& query_depth,
                                                          const cv::Mat& query_mask,
                                                          const sensor_msgs::CameraInfo& camera_info,
                                                          const tf::StampedTransform& room_transform,
                                                          const vector<CloudT::Ptr>& clouds,
                                                          const vector<vector<Eigen::Matrix4f>, Eigen::aligned_allocator<Eigen::Matrix4f > >& initial_poses,
                                                          const vector<vector<cv::Mat> >& images,
                                                          const vector<vector<cv::Mat> >& depths,
                                                          const vector<vector<cv::Mat> >& masks,
                                                          const vector<vector<string> >& paths,
                                                          const vector<float>& scores)
    {
        quasimodo_msgs::retrieval_query_result res;

        /*
        retrieval_query query
        =====================
        sensor_msgs/PointCloud2 cloud
        sensor_msgs/Image image
        sensor_msgs/Image depth
        sensor_msgs/Image mask
        sensor_msgs/CameraInfo camera
        int32 number_query
        geometry_msgs/Transform room_transform
        */
        pcl::toROSMsg(*query_cloud, res.query.cloud);
        convert_to_img_msg(query_image, res.query.image);
        convert_to_depth_msg(query_depth, res.query.depth);
        convert_to_mask_msg(query_mask, res.query.mask);
        res.query.camera = camera_info;
        res.query.number_query = number_query;
        tf::transformTFToMsg(room_transform, res.query.room_transform);

        /*
        retrieval_result result
        =======================
        sensor_msgs/PointCloud2[] retrieved_clouds
        geometry_msgs/Pose[][] retrieved_initial_poses
        sensor_msgs/Image[][] retrieved_images
        sensor_msgs/Image[][] retrieved_depths
        sensor_msgs/Image[][] retrieved_masks
        string[][] retrieved_image_paths
        float64[] retrieved_distance_scores
        */

        int number_retrieved = clouds.size();

        res.result.retrieved_clouds.resize(number_retrieved);
        res.result.retrieved_initial_poses.resize(number_retrieved);
        res.result.retrieved_images.resize(number_retrieved);
        res.result.retrieved_depths.resize(number_retrieved);
        res.result.retrieved_masks.resize(number_retrieved);
        res.result.retrieved_image_paths.resize(number_retrieved);
        res.result.retrieved_distance_scores.resize(number_retrieved);

        for (int i = 0; i < number_retrieved; ++i) {
            pcl::toROSMsg(*clouds[i], res.result.retrieved_clouds[i]);
            int nbr_images = images[i].size();

            res.result.retrieved_initial_poses[i].poses.resize(nbr_images);
            res.result.retrieved_images[i].images.resize(nbr_images);
            res.result.retrieved_depths[i].images.resize(nbr_images);
            res.result.retrieved_masks[i].images.resize(nbr_images);
            res.result.retrieved_image_paths[i].strings.resize(nbr_images);

            for (int j = 0; j < nbr_images; ++j) {
                //res.result.retrieved_initial_poses[i][j] = geometry_msgs::Pose();
                convert_to_img_msg(images[i][j], res.result.retrieved_images[i].images[j]);
                convert_to_depth_msg(depths[i][j], res.result.retrieved_depths[i].images[j]);
                convert_to_mask_msg(masks[i][j], res.result.retrieved_masks[i].images[j]);
                res.result.retrieved_image_paths[i].strings[j] = paths[i][j];
            }
            res.result.retrieved_distance_scores[i] = scores[i];
        }

        return res;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_retrieval_publisher");

    retrieval_publisher<grouped_vocabulary_tree<HistT, 8> > rs(ros::this_node::getName());

    ros::spin();

    return 0;
}
