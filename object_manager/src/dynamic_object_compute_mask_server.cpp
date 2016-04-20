#include <ros/ros.h>
#include <object_manager/DynamicObjectComputeMaskService.h>
#include <semantic_map/room_xml_parser.h>
#include <semantic_map/metaroom_xml_parser.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <geometry_msgs/Transform.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <convex_segmentation/CloudArray.h>
#include <pcl/features/normal_3d_omp.h>
#include <object_manager/dynamic_object_xml_parser.h>\

typedef pcl::PointXYZRGB PointType;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

//pcl::visualization::PCLVisualizer *p;

CloudPtr compute_mask_per_view_conv_seg(CloudPtr view, CloudPtr object, CloudPtr prevMask,
                                        cv::Mat& mask_image, vector<int>& mask_indices);
CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms);

bool dynamic_object_compute_mask_service(
        object_manager::DynamicObjectComputeMaskService::Request  &req,
        object_manager::DynamicObjectComputeMaskService::Response &res)
{
    ROS_INFO("Received a dynamic object compute mask request");
    ROS_INFO_STREAM("Object " << req.object_xml);
    res.result = false;

    // Load object data
    ObjectData object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(req.object_xml);

    if ((!object.objectCloud) || (!object.objectCloud->points.size())){
        // could not load object
        ROS_ERROR_STREAM("dynamic_object_compute_mask_service: could not load object from " << req.object_xml);
        throw std::runtime_error(
                    std::string("dynamic_object_compute_mask_service: could not load object. \n"));
        return true;
    }

    // check that the object has additional views
    if (!object.vAdditionalViews.size()){
        ROS_ERROR_STREAM("dynamic_object_compute_mask_service: object from " << req.object_xml<<" has no additional views to register.");
        throw std::runtime_error(
                    std::string("dynamic_object_compute_mask_service: object has no additional views. \n"));
        return true;
    }

    // check that the object has additional view registered transforms
    if (!object.vAdditionalViewsTransformsRegistered.size()){
        ROS_ERROR_STREAM("dynamic_object_compute_mask_service: object from " << req.object_xml<<" has no additional view registered transforms.");
        throw std::runtime_error(
                    std::string("dynamic_object_compute_mask_service: object has no additional view registered transforms. \n"));
        return true;
    }

    // transform the transforms in the object frame
    vector<CloudPtr> registered_object_views;
    for (size_t i=0; i<object.vAdditionalViews.size();i++){
        CloudPtr transformedCloud1(new Cloud);
        pcl_ros::transformPointCloud(*object.vAdditionalViews[i], *transformedCloud1,object.vAdditionalViewsTransformsRegistered[i]);
        pcl_ros::transformPointCloud(*transformedCloud1, *transformedCloud1,object.additionalViewsTransformToObservation);
        registered_object_views.push_back(transformedCloud1);
    }

    // load original object from disk
    unsigned found = req.object_xml.find_last_of("/");
    std::string obs_folder = req.object_xml.substr(0,found+1);
    DynamicObjectXMLParser parser(obs_folder, true);
    DynamicObject::Ptr original_object = parser.loadFromXML(req.object_xml,true);
    original_object->clearAdditionalViewMasks();

    // compute the mask per view
    CloudPtr prev_mask(new Cloud);
    ROS_INFO_STREAM("Computing masks for the additional views");
    for (size_t i=0; i<registered_object_views.size(); i++ ){
        ROS_INFO_STREAM("Computing masks for the additional view "<<i);
        CloudPtr registered_view_to_origin(new Cloud); registered_object_views[i];
        pcl_ros::transformPointCloud(*registered_object_views[i], *registered_view_to_origin, object.vAdditionalViewsTransformsRegistered[i].inverse());

        CloudPtr object_to_origin(new Cloud);
        pcl_ros::transformPointCloud(*object.objectCloud, *object_to_origin,object.additionalViewsTransformToObservation.inverse());
        pcl_ros::transformPointCloud(*object_to_origin, *object_to_origin, object.vAdditionalViewsTransformsRegistered[i].inverse());


        CloudPtr prev_mask_to_origin(new Cloud);
        pcl_ros::transformPointCloud(*prev_mask, *prev_mask_to_origin, object.vAdditionalViewsTransformsRegistered[i].inverse());

        cv::Mat mask_image;
        vector<int> mask_indices;
        CloudPtr mask = compute_mask_per_view_conv_seg(registered_view_to_origin, object_to_origin, prev_mask_to_origin,
                                                       mask_image, mask_indices);

        *prev_mask = *mask;
        pcl_ros::transformPointCloud(*prev_mask, *prev_mask, object.vAdditionalViewsTransformsRegistered[i]); // transform to map frame

//        // visualize mask
//        p->addPointCloud(registered_view_to_origin, "view");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> mask_handler(mask, 0, 255,0);
//        p->addPointCloud (mask,mask_handler,"mask");
//        p->spin();
//        p->removeAllPointClouds();

//        // visualize mask image
//        ROS_INFO_STREAM("Mask no indices "<<mask_indices.size());
//        cv::imshow( "Display window", mask_image );
//        cv::waitKey(0);

        original_object->addAdditionalViewMask(mask_image, mask_indices);
    }
    std::string xml_file = parser.saveAsXML(original_object);
    ROS_INFO_STREAM("Object saved at "<<xml_file);

    res.result = true;


    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_object_compute_mask_service");
    ros::NodeHandle n;

//    p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
//    p->addCoordinateSystem();

    ros::ServiceServer service = n.advertiseService("dynamic_object_compute_mask_server", dynamic_object_compute_mask_service);
    ROS_INFO("dynamic_object_compute_mask_server started.");
    ros::spin();

    return 0;
}

CloudPtr compute_mask_per_view_conv_seg(CloudPtr view, CloudPtr object, CloudPtr prev_mask,
                                        cv::Mat& mask_image, vector<int>& mask_indices){
    //    // compute normals
    //    double radius = 0.05;
    //    boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals(new pcl::PointCloud<pcl::Normal>);
    //    pcl::NormalEstimationOMP<PointType, pcl::Normal> normalEstimation;
    //    normalEstimation.setNumberOfThreads(2);
    //    normalEstimation.setInputCloud(filtered_view);
    //    normalEstimation.setRadiusSearch(radius);
    //    Tree::Ptr kdtree(new Tree);
    //    normalEstimation.setSearchMethod(kdtree);
    //    normalEstimation.compute(*normals);

    // mask image -> empty by default
    mask_image = cv::Mat::zeros(480, 640, CV_8UC3);

    // find convex segments
    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false); // do not filter
    supervoxel_segmentation::Graph* g;
    supervoxel_segmentation::Graph* convex_g;
    vector<CloudPtr> supervoxels;
    vector<CloudPtr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(view, false);// normals, false);

    CloudPtr mask(new Cloud);
    for (int i = 0; i < convex_segments.size(); ++i) {
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(object);
        segment.setTargetCloud(convex_segments[i]);
        segment.setDistanceThreshold(0.001);
        typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (convex_segments[i]);
        segment.setSearchMethod(tree);
        CloudPtr remainder(new Cloud);
        segment.segment(*remainder);
        if (remainder->points.size() != object->points.size()){
            *mask += *convex_segments[i];
        }
    }

    if (!mask->points.size()){
        ROS_ERROR_STREAM("Could not find mask using the initial object indices. Will try with the previous mask");
        if (prev_mask->points.size()){
            for (int i = 0; i < convex_segments.size(); ++i) {
                pcl::SegmentDifferences<PointType> segment;
                segment.setInputCloud(prev_mask);
                segment.setTargetCloud(convex_segments[i]);
                segment.setDistanceThreshold(0.001);
                typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
                tree->setInputCloud (convex_segments[i]);
                segment.setSearchMethod(tree);
                CloudPtr remainder(new Cloud);
                segment.segment(*remainder);
                if (remainder->points.size() != object->points.size()){
                    *mask += *convex_segments[i];
                }
            }
        }

    }

    if (!mask->points.size()){
        ROS_ERROR_STREAM("Could not find mask. ");
    } else {
        // find indices in original point cloud
        std::vector<int> nn_indices (1);
        std::vector<float> nn_distances (1);
        typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (view);

        // Iterate through the source data set
        for (int i = 0; i < static_cast<int> (mask->points.size ()); ++i)
        {
            if (!isFinite (mask->points[i]))
                continue;
            // Search for the closest point in the target data set (number of neighbors to find = 1)
            if (!tree->nearestKSearch (mask->points[i], 1, nn_indices, nn_distances))
            {
                PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, mask->points[i].x, mask->points[i].y, mask->points[i].z);
                continue;
            }

            if (nn_distances[0] < 0.001)
            {
                mask_indices.push_back (nn_indices[0]);
            }
        }

        // create mask image
        for (int index : mask_indices)
        {
            pcl::PointXYZRGB point = view->points[index];
            int y = index / mask_image.cols;
            int x = index % mask_image.cols;
            mask_image.at<cv::Vec3b>(y, x)[0] = point.b;
            mask_image.at<cv::Vec3b>(y, x)[1] = point.g;
            mask_image.at<cv::Vec3b>(y, x)[2] = point.r;
        }

    }

    return mask;

}

CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms){
    CloudPtr mergedCloud(new Cloud);

    for (size_t i=0; i<intermediate_clouds.size(); i++)
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*intermediate_clouds[i], transformed_cloud,intermediate_transforms[i]);
        *mergedCloud+=transformed_cloud;
    }
    return mergedCloud;
}



