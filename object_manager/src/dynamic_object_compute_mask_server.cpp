#include <pwd.h>
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
#include <object_manager/dynamic_object_xml_parser.h>
#include <std_msgs/Float32.h>

#include <observation_registration_services/ProcessRegisteredViews.h>
#include <observation_registration_services/ObservationRegistrationService.h>

typedef pcl::PointXYZRGB PointType;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;

using namespace std;

ros::ServiceClient surfel_client;
ros::ServiceClient registration_client;
pcl::visualization::PCLVisualizer *p;
std::string g_segmentation_method;

std::vector<CloudPtr> compute_convex_segmentation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals);

CloudPtr compute_mask_per_view_from_object(CloudPtr view, CloudPtr object,
                                           cv::Mat& mask_image, vector<int>& mask_indices, double neighbor_distance);


CloudPtr find_object_using_metaroom(std::string sweep_xml, std::string object_xml, CloudPtr registered_views_cloud, CloudPtr object_cloud, const double& cluster_tolerance);
CloudPtr find_object_using_conv_seg(CloudPtr reg_views_at_origin, pcl::PointCloud<pcl::Normal>::Ptr reg_views_at_origin_normals, CloudPtr object_cloud_at_origin);

bool dynamic_object_compute_mask_service(
        object_manager::DynamicObjectComputeMaskService::Request  &req,
        object_manager::DynamicObjectComputeMaskService::Response &res)
{
    ROS_INFO("Received a dynamic object compute mask request");
    ROS_INFO_STREAM("Object " << req.object_xml);
    ROS_INFO_STREAM("Observation "<<req.observation_xml);

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
    // load observation data and transform object into the observation frame
    // (originally it's in the frame of the second observation (or the metaroom's frame) that this observation is registered to)
    auto observation_data = SimpleXMLParser<PointType>::loadRoomFromXML(req.observation_xml,{"RoomCompleteCloud", "RoomIntermediateCloud"}, false, false);
    pcl::transformPointCloud(*object.objectCloud, *object.objectCloud, Eigen::Matrix4f(observation_data.roomTransform.inverse()));

    // process the additional view clouds (surfelize!)
    observation_registration_services::ProcessRegisteredViews srv;
    for (size_t i=0; i<original_object->m_vAdditionalViews.size(); i++){
        //
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*(original_object->m_vAdditionalViews[i]), cloud_msg);
        srv.request.registered_views.push_back(cloud_msg);

        geometry_msgs::Transform transform_msg;
        tf::Transform first_transform_inv = original_object->m_vAdditionalViewsTransformsRegistered[0].inverse();
        tf::Transform combined; combined.mult(first_transform_inv, original_object->m_vAdditionalViewsTransformsRegistered[i]);

        tf::transformTFToMsg(combined,transform_msg);
        srv.request.registered_views_transforms.push_back(transform_msg);
    }
    // pass camera intrinsics to the surfelize server
    image_geometry::PinholeCameraModel camParams;
    if (observation_data.vIntermediateRoomCloudCamParamsCorrected.size()){
        camParams = observation_data.vIntermediateRoomCloudCamParamsCorrected[0];
    } else {
        camParams = observation_data.vIntermediateRoomCloudCamParams[0];
    }
    std_msgs::Float32 intrinsic;
    intrinsic.data = (float)camParams.fx(); srv.request.intrinsics.push_back(intrinsic);
    intrinsic.data = (float)camParams.fy(); srv.request.intrinsics.push_back(intrinsic);
    intrinsic.data = (float)camParams.cx(); srv.request.intrinsics.push_back(intrinsic);
    intrinsic.data = (float)camParams.cy(); srv.request.intrinsics.push_back(intrinsic);

    ROS_INFO_STREAM("Using surfelize_server service");
    CloudPtr object_views(new Cloud);
    CloudPtr object_views_at_origin(new Cloud);
    pcl::PointCloud<pcl::Normal>::Ptr object_views_normals(new pcl::PointCloud<pcl::Normal>);
    bool surfelized = false;
    if (surfel_client.call(srv)){
        CloudPtr surfelized_object_views(new Cloud);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::fromROSMsg(srv.response.processed_cloud, *processed_cloud);
        // transform back into the observation frame
        for (const pcl::PointXYZRGBNormal& pn : processed_cloud->points) {
            PointType p;
            p.getVector3fMap() = pn.getVector3fMap();
            p.rgba = pn.rgba;
            pcl::Normal n;
            n.getNormalVector3fMap() = pn.getNormalVector3fMap();
            object_views_normals->push_back(n);
            surfelized_object_views->push_back(p);
        }
        *object_views_at_origin = *surfelized_object_views;
        pcl_ros::transformPointCloud(*surfelized_object_views, *surfelized_object_views,object.vAdditionalViewsTransformsRegistered[0]);
        pcl_ros::transformPointCloud(*surfelized_object_views, *surfelized_object_views,object.additionalViewsTransformToObservation);
        *object_views = *surfelized_object_views;
        surfelized = true;
    } else {
        ROS_INFO_STREAM("Could not call surfelize_server to process the object views. Will proceed with the unfiltered views.");
        for (CloudPtr view: registered_object_views){
            *object_views += *view;
        }
//        pcl_ros::transformPointCloud(*object_views, *object_views,object.additionalViewsTransformToObservation);
        // downsample views
        object_views = MetaRoom<PointType>::downsampleCloud(object_views->makeShared());
    }

    CloudPtr segmented_object_cloud(new Cloud);
    if ((g_segmentation_method == "convex_segmentation") && (surfelized)){
        // transform object cloud to the origin (needed for convex segmentation)
        ROS_INFO_STREAM("Segmenting object using the convex segmentation");
        CloudPtr object_cloud_at_origin(new Cloud);
        pcl_ros::transformPointCloud(*object.objectCloud, *object_cloud_at_origin,object.additionalViewsTransformToObservation.inverse());
        pcl_ros::transformPointCloud(*object_cloud_at_origin, *object_cloud_at_origin,object.vAdditionalViewsTransformsRegistered[0].inverse());


        segmented_object_cloud = find_object_using_conv_seg(object_views_at_origin, object_views_normals, object_cloud_at_origin);
        // transform object from origin to observation frame of ref
        pcl_ros::transformPointCloud(*segmented_object_cloud, *segmented_object_cloud,object.vAdditionalViewsTransformsRegistered[0]);
        pcl_ros::transformPointCloud(*segmented_object_cloud, *segmented_object_cloud,object.additionalViewsTransformToObservation);
    } else {
        // default -> metaroom
        ROS_INFO_STREAM("Segmenting object using the metaroom segmentation");
        double cluster_tolerance = 0.02;
        if (!surfelized){
            cluster_tolerance = 0.03;
        }
        segmented_object_cloud = find_object_using_metaroom(req.observation_xml, req.object_xml, object_views, object.objectCloud, cluster_tolerance);
    }

    // compute the mask per view
    CloudPtr prev_mask(new Cloud);
    ROS_INFO_STREAM("Computing masks for the additional views");
    for (size_t i=0; i<registered_object_views.size(); i++ ){
        ROS_INFO_STREAM("Computing masks for the additional view "<<i);
        CloudPtr registered_view(new Cloud);
        *registered_view = *registered_object_views[i];

        cv::Mat mask_image;
        vector<int> mask_indices;
        double neighbor_distance = 0.001;
        if (g_segmentation_method != "meta_room"){
            neighbor_distance = 0.00025;
        }
        CloudPtr mask = compute_mask_per_view_from_object(registered_view, segmented_object_cloud, mask_image, mask_indices, neighbor_distance);
        //        // visualize mask image
//        ROS_INFO_STREAM("Mask no indices "<<mask_indices.size());
//        cv::imshow( "Display window", mask_image );
//        cv::waitKey(0);

        original_object->addAdditionalViewMask(mask_image, mask_indices);
    }
    std::string xml_file = parser.saveAsXML(original_object);
    ROS_INFO_STREAM("Object saved at "<<xml_file);

    // return segmented object
    sensor_msgs::PointCloud2 segmented_cloud_msg;
    pcl::toROSMsg(*segmented_object_cloud, segmented_cloud_msg);
    res.segmented_object = segmented_cloud_msg;

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_object_compute_mask_service");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    surfel_client = n.serviceClient<observation_registration_services::ProcessRegisteredViews>("/surfelize_server");
    registration_client = n.serviceClient<observation_registration_services::ObservationRegistrationService>("/observation_registration_server");
    n_private.param<string>("segmentation_method",g_segmentation_method,"meta_room");
//    g_segmentation_method = "convex_segmentation";
//    g_segmentation_method = "meta_room";

//    p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
//    p->addCoordinateSystem();

    ros::ServiceServer service = n.advertiseService("dynamic_object_compute_mask_server", dynamic_object_compute_mask_service);

     if (g_segmentation_method != "convex_segmentation"){
             g_segmentation_method = "meta_room";
     }

    ROS_INFO("dynamic_object_compute_mask_server started.");
    ROS_INFO_STREAM("Object segmentation method is "<<g_segmentation_method);
    ros::spin();

    return 0;
}


CloudPtr compute_mask_per_view_from_object(CloudPtr view, CloudPtr object,
                                           cv::Mat& mask_image, vector<int>& mask_indices, double neighbor_distance){
    using namespace std;
    // mask image -> empty by default
    mask_image = cv::Mat::zeros(480, 640, CV_8UC3);
    CloudPtr mask(new Cloud);
    if (!object->points.size()){
        ROS_ERROR_STREAM("Could not find mask. The segmented object has 0 points.");
        return mask;
    }

    // compute mask
    // find indices in original point cloud
    std::vector<int> nn_indices (1);
    std::vector<float> nn_distances (1);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (object);

    // Iterate through the source data set
    for (int i = 0; i < static_cast<int> (view->points.size ()); ++i)
    {
        if (!isFinite (view->points[i]))
            continue;
        // Search for the closest point in the target data set (number of neighbors to find = 1)
        if (!tree->nearestKSearch (view->points[i], 1, nn_indices, nn_distances))
        {
            PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, view->points[i].x, view->points[i].y, view->points[i].z);
            continue;
        }

        if (nn_distances[0] < neighbor_distance)
        {
            mask_indices.push_back (i);
            mask->push_back(object->points[nn_indices[0]]);
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

    // visualize
//    {
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> view_handler(view, 255, 0,0);
//        p->addPointCloud (view,view_handler,"view");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> object_handler(object, 0, 255, 0);
//        p->addPointCloud (object,object_handler,"object");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> mask_handler(mask, 0, 255, 255);
//        p->addPointCloud (mask,mask_handler,"mask");
//        p->spin();
//        p->removeAllPointClouds();
//    }


}

std::vector<CloudPtr> compute_convex_segmentation(CloudPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){

    using Graph = supervoxel_segmentation::Graph;
    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false); // do not filter
    Graph* g;
    Graph* convex_g;
    vector<CloudPtr> supervoxels;
    vector<CloudPtr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, normals, false);

    std::vector<CloudPtr> segments;
    for (CloudPtr c : convex_segments) {
        segments.push_back(c);
    }

    return segments;

}

CloudPtr find_object_using_conv_seg(CloudPtr reg_views_at_origin, pcl::PointCloud<pcl::Normal>::Ptr reg_views_at_origin_normals, CloudPtr object_cloud_at_origin){
    using namespace std;
    CloudPtr built_object_cloud(new Cloud);

    // compute convex segmentation
    vector<CloudPtr> convex_segments = compute_convex_segmentation(reg_views_at_origin, reg_views_at_origin_normals);
    //     transform initial set of inidices to the origin
    for (int i = 0; i < convex_segments.size(); ++i) {
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(object_cloud_at_origin);
        segment.setTargetCloud(convex_segments[i]);
        segment.setDistanceThreshold(0.0001);
        typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (convex_segments[i]);
        segment.setSearchMethod(tree);
        CloudPtr remainder(new Cloud);
        segment.segment(*remainder);
        if (remainder->points.size() < object_cloud_at_origin->points.size()*0.9){
            *built_object_cloud += *convex_segments[i];
        }
    }

    // visualize
//    {
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> views_handler(reg_views_at_origin, 0, 255,0);
//        p->addPointCloud (reg_views_at_origin,views_handler,"views");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> object_handler(object_cloud_at_origin, 0, 255, 255);
//        p->addPointCloud (object_cloud_at_origin,object_handler,"object");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> built_object_handler(built_object_cloud, 255, 0, 255);
//        p->addPointCloud (built_object_cloud,built_object_handler,"built_object");
//        p->spin();
//        p->removeAllPointClouds();
//    }

    return built_object_cloud;
}

CloudPtr find_object_using_metaroom(std::string observation_xml, std::string object_xml, CloudPtr registered_views_cloud, CloudPtr object_cloud, const double& cluster_tolerance){
    using namespace std;
    CloudPtr built_object_cloud(new Cloud);
    // load observation
    auto observation_data = SimpleXMLParser<PointType>::loadRoomFromXML(observation_xml,{"RoomCompleteCloud"}, false, true);
    auto observation_data_int_transforms = SimpleXMLParser<PointType>::loadRoomFromXML(observation_xml,{"RoomIntermediateCloud"}, false, false);

    // find previous observation
    string room_waypoint_id = observation_data.roomWaypointId;
    string root_folder = SimpleXMLParser<PointType>::getRootFolderFromSweepXml(observation_xml);

    if (root_folder == ""){
        passwd* pw = getpwuid(getuid());
        root_folder = std::string(pw->pw_dir);
        root_folder+="/.semanticMap";
    }
    ROS_INFO_STREAM("Looking for additional observations in "<<root_folder);

    vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(root_folder, room_waypoint_id);
    if (matchingObservations.size() < 2){
        ROS_ERROR_STREAM("Not enough observations to build metaroom.");
        return built_object_cloud;
    }
    string previous_observation_xml = "";
    for (size_t i=0; i<matchingObservations.size(); i++){
        if (matchingObservations[i] == observation_xml){
            if (i!=0){
                previous_observation_xml = matchingObservations[i-1];
            }
        }
    }
    if (previous_observation_xml != ""){
        cout<<"Previous obs "<<previous_observation_xml<<endl;
    } else {
        ROS_ERROR_STREAM("Not enough observations to build metaroom");
        return built_object_cloud;
    }
    //    string previous_observation_xml = matchingObservations[1];
    // register observations
    tf::Transform registration_transform; registration_transform.setIdentity();

    observation_registration_services::ObservationRegistrationService srv;
    srv.request.source_observation_xml = previous_observation_xml; // source
    srv.request.target_observation_xml = observation_xml; //target
    Eigen::Matrix4f registered_transform;
    Eigen::Matrix4f room_transform = observation_data.roomTransform;

    if (!room_transform.isIdentity(0.001)){
        registered_transform = room_transform;
    }

    bool reg_server_called = false;
    if (registration_client.call(srv))
    {
        ROS_INFO_STREAM("Registration done using the observation_registration_server service. Number of constraints "<<srv.response.total_correspondences);
        if (srv.response.total_correspondences <= 0){
            ROS_INFO_STREAM("Registration unsuccessful due to insufficient constraints.");
        } else {
            // registration successfull -> get registered transform
            tf::Transform tf_registered_transform;
            tf::transformMsgToTF(srv.response.transform, tf_registered_transform);
            pcl_ros::transformAsMatrix(tf_registered_transform, registered_transform);
            reg_server_called = true;
        }
    } else {
        ROS_ERROR_STREAM("Could not call observation_registration_server service.");
    }

    if (registered_transform.isIdentity(0.001) && !reg_server_called){
        ROS_ERROR_STREAM("Registration failed while constructing meta-room");
        return built_object_cloud;
    }
    auto previous_observation_data = SimpleXMLParser<PointType>::loadRoomFromXML(previous_observation_xml,{"RoomCompleteCloud"}, false, true);
    CloudPtr observation_interior_cloud = MetaRoom<PointType>::downsampleCloud(observation_data.completeRoomCloud->makeShared());
    CloudPtr previous_interior_cloud = MetaRoom<PointType>::downsampleCloud(previous_observation_data.completeRoomCloud->makeShared());
    pcl::transformPointCloud(*previous_interior_cloud, *previous_interior_cloud,registered_transform);

//    {
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> room_handler(observation_interior_cloud, 255, 0,0);
//        p->addPointCloud (observation_interior_cloud,room_handler,"room");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> views_handler(registered_views_cloud, 0, 255,0);
//        p->addPointCloud (registered_views_cloud,views_handler,"views");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> prev_handler(previous_interior_cloud, 0, 0, 255);
//        p->addPointCloud (previous_interior_cloud,prev_handler,"clusters");
//        p->spin();
//        p->removeAllPointClouds();
//    }

    // compute differences and update metaroom
    CloudPtr differenceRoomToPrevRoom(new Cloud);
    CloudPtr differencePrevRoomToRoom(new Cloud);
    // compute the differences
    pcl::SegmentDifferences<PointType> segment;
    segment.setInputCloud(observation_interior_cloud);
    segment.setTargetCloud(previous_interior_cloud);
    segment.setDistanceThreshold(0.001);
    typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (previous_interior_cloud);
    segment.setSearchMethod(tree);
    segment.segment(*differenceRoomToPrevRoom);

    segment.setInputCloud(previous_interior_cloud);
    segment.setTargetCloud(observation_interior_cloud);
    tree->setInputCloud(observation_interior_cloud);
    segment.segment(*differencePrevRoomToRoom);

    CloudPtr toBeAdded(new Cloud());
    CloudPtr toBeRemoved(new Cloud());

    OcclusionChecker<PointType> occlusionChecker;
    occlusionChecker.setSensorOrigin(observation_data_int_transforms.vIntermediateRoomCloudTransforms[0].getOrigin()); // since it's already transformed
    auto occlusions = occlusionChecker.checkOcclusions(differenceRoomToPrevRoom, differencePrevRoomToRoom,720 );

    CloudPtr meta_room(new Cloud);
    segment.setInputCloud(observation_interior_cloud);
    segment.setTargetCloud(differenceRoomToPrevRoom);
    tree->setInputCloud(differenceRoomToPrevRoom);
    segment.segment(*meta_room);
    if (occlusions.toBeAdded->points.size()){
        *meta_room += *occlusions.toBeAdded;
    }

    CloudPtr dynamic_clusters(new Cloud);
    segment.setInputCloud(registered_views_cloud);
    segment.setTargetCloud(meta_room);
    tree->setInputCloud(meta_room);
    segment.setDistanceThreshold(0.001);
    segment.segment(*dynamic_clusters);

    // find object cluster
    std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(dynamic_clusters,cluster_tolerance,50,1000000);
    for (int i = 0; i < vClusters.size(); ++i) {
        segment.setInputCloud(object_cloud);
        segment.setTargetCloud(vClusters[i]);
        tree->setInputCloud (vClusters[i]);
        CloudPtr remainder(new Cloud);
        segment.segment(*remainder);
        if (remainder->points.size() < object_cloud->points.size()*0.9){
            *built_object_cloud += *vClusters[i];
        }
    }

    //    // visualize
//    {
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> meta_room_handler(meta_room, 255, 0,0);
//        p->addPointCloud (meta_room,meta_room_handler,"meta_room");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> views_handler(registered_views_cloud, 0, 255,0);
//        p->addPointCloud (registered_views_cloud,views_handler,"views");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> clusters_handler(dynamic_clusters, 0, 0, 255);
//        p->addPointCloud (dynamic_clusters,clusters_handler,"clusters");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> object_handler(object_cloud, 0, 255, 255);
//        p->addPointCloud (object_cloud,object_handler,"object");
//        pcl::visualization::PointCloudColorHandlerCustom<PointType> built_object_handler(built_object_cloud, 255, 0, 255);
//        p->addPointCloud (built_object_cloud,built_object_handler,"built_object");
//        p->spin();
//        p->removeAllPointClouds();
//    }

    cout<<"Built object has "<<built_object_cloud->points.size()<<endl;


    return built_object_cloud;

}
