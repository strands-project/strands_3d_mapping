#include <object_3d_retrieval/supervoxel_segmentation.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <object_manager/dynamic_object_xml_parser.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <ctime>
#include <QDir>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/visualize.h>

using namespace std;

typedef pcl::PointCloud<SurfelType> CloudSurfel;
typedef typename CloudSurfel::Ptr CloudSurfelPtr;

using PointT = pcl::PointXYZRGB;
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> CloudT;
typedef typename CloudT::Ptr CloudPtr;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using Graph = supervoxel_segmentation::Graph;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;
using pcl::visualization::PointCloudColorHandlerCustom;
struct cam_intrinsics {
    double fx, fy, cx, cy;
    cam_intrinsics(double fx_, double fy_, double cx_, double cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_)
    {}
};

template<typename PointT,
         typename LeafContainerT = pcl::octree::OctreeContainerPointIndices,
         typename BranchContainerT = pcl::octree::OctreeContainerEmpty >
class OctreePointCloudOverlap : public pcl::octree::OctreePointCloud<PointT,
        LeafContainerT, BranchContainerT, pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT> >

{

public:

    /** \brief Constructor.
         *  \param resolution_arg:  octree resolution at lowest octree level
         * */
    OctreePointCloudOverlap(const double resolution_arg) :
        pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT,
        pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT> >(resolution_arg)
    {
        //pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT> >::enableDynamicDepth(0);
    }

    /** \brief Empty class constructor. */
    virtual ~OctreePointCloudOverlap()
    {
    }

    /** \brief Get a indices from all leaf nodes that did not exist in previous buffer.
         * \param indicesVector_arg: results are written to this vector of int indices
         * \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to become serialized.
         * \return number of point indices
         */
    std::size_t getNewOccupiedVoxelCenters()
    {

        std::vector<pcl::octree::OctreeContainerPointIndices*> leaf_containers;
        this->serializeNewLeafs(leaf_containers); // this method is actually public, so actually no need to subclass
        return leaf_containers.size();
    }
};

double compute_overlap(CloudT::Ptr& A, CloudT::Ptr& B, float resolution)
{
    // Octree resolution - side length of octree voxels

    // Instantiate octree-based point cloud change detection class
    OctreePointCloudOverlap<PointT> octree(resolution);

    // Add points from cloudA to octree
    octree.setInputCloud(A);
    octree.addPointsFromInputCloud();

    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    double nbr_total_A = octree.getOccupiedVoxelCenters(dummy);

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    // Add points from cloudB to octree
    octree.setInputCloud(B);
    octree.addPointsFromInputCloud();

    double nbr_total_B = octree.getOccupiedVoxelCenters(dummy);

    // Get vector of point indices from octree voxels which did not exist in previous buffer

    double nbr_not_A = octree.getNewOccupiedVoxelCenters();

    if (nbr_not_A == nbr_total_B) {
        return 0.0;
    }

    /*
    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    // Important for this second part is that it seems like it cleans up empty branches first
    octree.switchBuffers();

    // Add points from cloudB to octree
    octree.setInputCloud(A);
    octree.addPointsFromInputCloud();

    double nbr_not_B = octree.getNewOccupiedVoxelCenters();

    double nbr_both = nbr_total_A - nbr_not_B;
    double nbr_total = nbr_total_A + nbr_not_A;
    */

    double nbr_both = nbr_total_B - nbr_not_A;
    double nbr_total = nbr_total_A + nbr_not_A;

    double overlap_fraction = nbr_both / nbr_total;

    return overlap_fraction;
}

CloudT::Ptr crop_with_cloud(CloudT::Ptr& cloud, CloudT::Ptr& crop_cloud)
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(crop_cloud);
    CloudT::Ptr overlap_cloud(new CloudT);
    // now, associate each point in segment with a surfel in the surfel cloud!
    for (PointT p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        kdtree.nearestKSearchT(p, 1, indices, distances);
        if (distances[0] < 0.01f*0.01f) {
            overlap_cloud->push_back(p);
        }
    }
    return overlap_cloud;
}

vector<Eigen::Matrix4f> getRegisteredViewPoses(const string& poses_file, const int& no_transforms);
vector<cv::Mat> getAdditionalViews(const string& views_folder, const int& no_transforms);
vector<cv::Mat> getAdditionalViewLabels(const string& labels_folder, const int& no_transforms);
void project_cloud_in_image(cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Affine3f pose, cam_intrinsics params);
CloudPtr project_mask_in_cloud(cv::Mat mask, CloudPtr &view,  Eigen::Matrix4f transform);

int main(int argc, char **argv)
{
    cam_intrinsics default_params(528.0, 525.0, 317, 245);
    bool DISPLAY = false;

    string room_xml;
    if (argc>1){
        room_xml = argv[1];
    } else {
        cout<<"Provide room xml file. EXITTING."<<endl;
    }

    int index = room_xml.find_last_of("/");
    string room_folder = room_xml.substr(0,index+1);
    room_folder+="/";
    cout<<"Sweep folder "<<room_folder<<endl;
    pcl::visualization::PCLVisualizer* pg;
    if (DISPLAY){
        pg = new pcl::visualization::PCLVisualizer (argc, argv, "global_transform");
        pg->addCoordinateSystem();
//        cv::namedWindow("mask", cv::WINDOW_OPENGL);
        //        cv::namedWindow("image", cv::WINDOW_OPENGL);
    }

    QStringList objectFiles = QDir(room_folder.c_str()).entryList(QStringList("*object*.xml"));
    vector<ObjectData> objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(room_folder+"room.xml");
    int object_id;
    cout<<"Loaded "<<objects.size()<<" objects "<<endl;
    for (size_t i=0; i< objects.size(); i++){
        if (objects[i].vAdditionalViews.size() != 0){
            object_id = i;
        }
    }

    auto object_ = objects[object_id];
    object_.vAdditionalViews.insert(object_.vAdditionalViews.begin(), object_.intermediateCloud);
    pcl_ros::transformPointCloud(*object_.objectCloud, *object_.objectCloud,object_.transformToGlobal.inverse());
    pcl_ros::transformPointCloud(*object_.objectCloud, *object_.objectCloud,object_.calibratedTransform.inverse());
    cout<<"Relevant object: object_"<<object_id<<endl;

    // load object registered poses, views and label
    stringstream ss_pose_file; ss_pose_file<<room_folder<<"/object_"<<object_id<<"/";
    vector<Eigen::Matrix4f> poses = getRegisteredViewPoses(ss_pose_file.str() + "poses.txt", objects[object_id].vAdditionalViews.size());
    cout<<"Loaded "<<poses.size()<<" registered poses."<<endl;
    string views_folder = ss_pose_file.str() + "mesh/";
    vector<cv::Mat> object_views = getAdditionalViews(views_folder, objects[object_id].vAdditionalViews.size());
    vector<cv::Mat> object_view_labels = getAdditionalViewLabels(views_folder, objects[object_id].vAdditionalViews.size());
    cout<<"Loaded "<<object_views.size()<<" object views and "<<object_view_labels.size()<<" labels "<<endl;

    // load surfel_map
    string surfel_map_name = "surfel_map_3.pcd";
    CloudSurfelPtr surfel_map_cloud ( new CloudSurfel);
    pcl::PCDReader reader;
    reader.read (ss_pose_file.str() + surfel_map_name, *surfel_map_cloud);
    cout<<"Loaded surfel map. Points: "<<surfel_map_cloud->points.size()<<endl;
    // convert to XYZ for display
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> surfel_map_cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*surfel_map_cloud, *surfel_map_cloud_xyz);

    // Display
    if (DISPLAY){
        pg->addPointCloud(surfel_map_cloud_xyz, "surfel_map");
        PointCloudColorHandlerCustom<PointType> object_h (object_.objectCloud, 0, 255, 0);
        pg->addPointCloud(object_.objectCloud, object_h, "object_cloud");
        pg->spin();
        pg->removeAllPointClouds();
    }


    /* TO DO --- SEGMENTATION
     *
     *  surfel_map_cloud  -> surfel map
     *  object_.objectCloud -> mask // use for selecting segment
     */
    CloudT::Ptr cloud(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    cloud->reserve(surfel_map_cloud->size());
    normals->reserve(surfel_map_cloud->size());
    for (const SurfelType& s : surfel_map_cloud->points) {
        PointType p;
        p.getVector3fMap() = s.getVector3fMap();
        p.rgba = s.rgba;
        NormalT n;
        n.getNormalVector3fMap() = s.getNormalVector3fMap();
        cloud->push_back(p);
        normals->push_back(n);
    }

    int colormap[][3] = {
        {166,206,227},
        {31,120,180},
        {178,223,138},
        {51,160,44},
        {251,154,153},
        {227,26,28},
        {253,191,111},
        {255,127,0},
        {202,178,214},
        {106,61,154},
        {255,255,153},
        {177,89,40},
        {141,211,199},
        {255,255,179},
        {190,186,218},
        {251,128,114},
        {128,177,211},
        {253,180,98},
        {179,222,105},
        {252,205,229},
        {217,217,217},
        {188,128,189},
        {204,235,197},
        {255,237,111}
    };

    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false); // do not filter
    Graph* g;
    Graph* convex_g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> convex_segments;
    map<size_t, size_t> indices;
    //std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, normals, false);

    /*
    {
        CloudT::Ptr colored_segments(new CloudT);
        int counter = 0;
        for (CloudT::Ptr& c : convex_segments) {
            for (PointT p : c->points) {
                p.r = colormap[counter%24][0];
                p.g = colormap[counter%24][1];
                p.b = colormap[counter%24][2];
                colored_segments->push_back(p);
            }
            ++counter;
        }

        for (PointT p : *object_.objectCloud) {
            p.r = 0; p.g = 0; p.b = 0;
            p.z -= 0.01;
            colored_segments->push_back(p);
        }
        dynamic_object_retrieval::visualize(colored_segments);
    }
    */

    delete g;
    delete convex_g;

    // sample with MR segmentation
    string mr_segmented_surfel = "object_surfel.pcd";
    CloudT::Ptr mr_segmented_cloud ( new CloudT);
    //CloudT::Ptr mr_segmented_cloud ( new CloudT);
    reader.read (ss_pose_file.str() + "/mesh/" + mr_segmented_surfel, *mr_segmented_cloud);
    double max_overlap = 0.0;
    int max_index = 0;
    int counter = 0;
    /*
    for (CloudT::Ptr& segment : convex_segments) {
        double overlap = compute_overlap(segment, object_.objectCloud, 0.1);
        if (overlap > max_overlap) {
            max_index = counter;
            max_overlap = overlap;
        }
        ++counter;
    }
    if (max_overlap == 0.0) {
        cout << "Could not find any overlapping segment, exiting..." << endl;
        return 0;
    }
    */
    //pcl::copyPointCloud(*convex_segments[max_index], *mr_segmented_cloud);
    /*
    dynamic_object_retrieval::visualize(convex_segments[max_index]);
    */

    cout<<"Loaded MR segmentation. Points: "<<mr_segmented_cloud->points.size()<<endl;
    // convert to XYZ for display
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> mr_segmented_cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*mr_segmented_cloud, *mr_segmented_cloud_xyz);

    // Display
    if (DISPLAY){
        pg->addPointCloud(surfel_map_cloud_xyz, "surfel_map");
        PointCloudColorHandlerCustom<pcl::PointXYZ> object_h (mr_segmented_cloud_xyz, 0, 255, 0);
        pg->addPointCloud(mr_segmented_cloud_xyz, object_h, "object_cloud");
        pg->spin();
        pg->removeAllPointClouds();
    }

#if 1
    ofstream myfile;
    string overlap_file = room_folder + "overlaps.txt";
    myfile.open(overlap_file);
#endif

    //myfile << "Writing this to a file.\n";
    // Project in images and get mask
    for (size_t i=0; i<object_views.size(); i++){
        // we should constrain mr_segmented_cloud to only be in object_.vAdditionalViews[i]
        CloudPtr projected = project_mask_in_cloud(object_view_labels[i], object_.vAdditionalViews[i], poses[i]);
        CloudT::Ptr cropped_cloud = crop_with_cloud(mr_segmented_cloud, object_.vAdditionalViews[i]);
        double overlap = compute_overlap(cropped_cloud, projected, 0.02);
        cout << "Got overlap: " << overlap << endl;
#if 1
        myfile << overlap << "\n";
#endif
        if (DISPLAY){
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cropped_cloud_xyz ( new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*cropped_cloud, *cropped_cloud_xyz);
            PointCloudColorHandlerCustom<pcl::PointXYZ> object_h (cropped_cloud_xyz, 0, 255, 0);
            PointCloudColorHandlerCustom<PointType> projected_h (projected, 255, 0, 0);
            pg->addPointCloud(cropped_cloud_xyz, object_h, "object_cloud");
            pg->addPointCloud(projected, projected_h, "projected");
            pg->spin();
            pg->removeAllPointClouds();
        }
    }
#if 1
    myfile.close();
#endif
}

vector<Eigen::Matrix4f> getRegisteredViewPoses(const string& poses_file, const int& no_transforms)
{
    vector<Eigen::Matrix4f> toRet;
    ifstream in(poses_file);
    if (!in.is_open()){
        cout<<"ERROR: cannot find poses file "<<poses_file<<endl;
        return toRet;
    }
    cout<<"Loading additional view registered poses from "<<poses_file<<endl;

    for (size_t i=0; i<no_transforms+1; i++){
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


vector<cv::Mat> getAdditionalViews(const string& views_folder, const int& no_transforms){

    vector<cv::Mat> toRet;

    for (size_t i=0; i<no_transforms+1; i++){
        stringstream ss_view;
        ss_view << std::setfill('0')<<std::setw(8)<<i<<".jpg";
        cv::Mat image = cv::imread(views_folder + ss_view.str());
        toRet.push_back(image);
    }

    return toRet;
}

vector<cv::Mat> getAdditionalViewLabels(const string& labels_folder, const int& no_transforms){
    vector<cv::Mat> toRet;

    for (size_t i=0; i<no_transforms+1; i++){
        stringstream ss_view;
        ss_view << std::setfill('0')<<std::setw(8)<<i<<"_label_0.jpg";
        cv::Mat image = cv::imread(labels_folder + ss_view.str());
        toRet.push_back(image);
    }
    return toRet;
}

void project_cloud_in_image(cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Affine3f pose, cam_intrinsics params){


    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // transform original cloud in camera coordinates
    pcl::transformPointCloud (*cloud, *transformed_cloud, pose);

    cv::Mat mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    for (pcl::PointXYZ point : transformed_cloud->points){
        // project on image plane
        double x = point.x / point.z;
        double y = point.y / point.z;

        x*=params.fx;
        y*=params.fy;

        x+=params.cx;
        y+=params.cy;

        mask.at<u_int8_t>(y, x) = image.at<u_int8_t>(y,x); // convert to uint 16 from meters
    }

    imshow("mask", mask);
    imshow("image", image);
    cv::waitKey(0);
}

CloudPtr project_mask_in_cloud(cv::Mat mask, CloudPtr& view, Eigen::Matrix4f transform){
    CloudPtr projected(new CloudT);
//    imshow("mask", mask);
//    cv::waitKey(0);

    for (size_t y = 0; y < mask.rows; ++y) {
        for (size_t x = 0; x < mask.cols; ++x) {
            //int indepoint = cloud->points[y*toRet.first.cols + x];
            // RGB
            if ((mask.at<cv::Vec3b>(y, x)[0] == 255) &&
                    (mask.at<cv::Vec3b>(y, x)[1] == 255) &&
                    (mask.at<cv::Vec3b>(y, x)[2] == 255))
            {

                int index = y*mask.cols + x;
                if (pcl::isFinite(view->points[index])) {
                    projected->push_back(view->points[index]);
                }

            }
        }
    }

    pcl::transformPointCloud(*projected, *projected, transform);
    pcl::transformPointCloud(*view, *view, transform);
    return projected;
}
