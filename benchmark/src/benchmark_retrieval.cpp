#include "object_3d_benchmark/benchmark_retrieval.h"
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

namespace benchmark_retrieval {

double get_match_accuracy(CloudT::Ptr& object, CloudT::Ptr& cluster)
{
    /*
    CloudT::Ptr visualization_cloud(new CloudT);
    *visualization_cloud += *object;
    *visualization_cloud += *cluster;
    dynamic_object_retrieval::visualize(visualization_cloud);
    */
    if ((object->points[0].getVector3fMap()-cluster->points[0].getVector3fMap()).norm() > 1.0f) {
        cout << "This was not a match!" << endl;
        return -1;
    }

    // check if it's a match
    CloudT::Ptr difference(new CloudT());
    pcl::SegmentDifferences<PointT> segment;
    segment.setInputCloud(object);
    segment.setTargetCloud(cluster);
    segment.setDistanceThreshold(0.001);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cluster);
    segment.setSearchMethod(tree);
    segment.segment(*difference);

    if (difference->points.size() > 0.9*object->points.size()) { // 0.9
        cout << "This was not a match!" << endl;
        return -1.0;
    }
    else {
        double percentage1 = double(object->points.size() - difference->points.size()) / double(object->points.size());
        segment.setInputCloud(cluster);
        segment.setTargetCloud(object);
        pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>);
        tree->setInputCloud (object);
        segment.setSearchMethod(tree2);
        segment.segment(*difference);
        double percentage2 = double(cluster->points.size() - difference->points.size()) / double(cluster->points.size());
        cout << "This was a match with score" << (percentage1+percentage2)/2.0 << endl;
        return (percentage1+percentage2)/2.0;
    }
}

vector<pair<CloudT::Ptr, string> > find_labels(vector<CloudT::Ptr>& input_segmented_dynamics, const vector<boost::filesystem::path>& sweep_paths)
                                               //semantic_map_load_utilties::LabelledData<PointT>& labelled_clusters)
{
    vector<pair<CloudT::Ptr, string> > labelled_segmented_dynamics;

    for (auto tup : dynamic_object_retrieval::zip(input_segmented_dynamics, sweep_paths)) {
        CloudT::Ptr segmented_dynamic;
        boost::filesystem::path sweep_xml;
        tie(segmented_dynamic, sweep_xml) = tup;

        LabelT labelled_clusters = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml.string());

        bool found = false;
        for (size_t i = 0; i < labelled_clusters.objectClouds.size(); ++i) {
            double accuracy = get_match_accuracy(segmented_dynamic, labelled_clusters.objectClouds[i]);
            if (accuracy != -1) {
                labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic, labelled_clusters.objectLabels[i]));
                found = true;
                break;
            }
        }

        if (!found) {
            labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic, string("")));
        }
    }

    return labelled_segmented_dynamics;
}

pair<Eigen::Matrix3f, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > get_camera_matrix_and_transforms(const string& sweep_xml)
{
    semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

    image_geometry::PinholeCameraModel model = data.vIntermediateRoomCloudCamParams[0];
    cv::Matx33d cvK = model.intrinsicMatrix();
    Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);

    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    Eigen::Affine3d first;
    tf::transformTFToEigen(data.vIntermediateRoomCloudTransforms[0], first);
    //first = first.inverse();
    for (tf::StampedTransform t : data.vIntermediateRoomCloudTransforms) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(t, e);
        camera_transforms.push_back((e.inverse()*first).matrix().cast<float>());
    }

    return make_pair(dK.cast<float>().transpose(), camera_transforms);
}

Eigen::Matrix4f get_global_camera_rotation(semantic_map_load_utilties::LabelledData<PointT>& labels)
{
    tf::StampedTransform tt = labels.transformToGlobal;
    Eigen::Affine3d e;
    tf::transformTFToEigen(tt, e);
    Eigen::Matrix4f T = e.matrix().cast<float>();
    T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    return T;
}

cv::Mat sweep_get_depth_at(const boost::filesystem::path& sweep_xml, size_t scan_index)
{
    stringstream ss;
    ss << "depth_" << std::setw(4) << std::setfill('0') << scan_index;
    boost::filesystem::path depth_path = sweep_xml.parent_path() / (ss.str() + ".png");
    cv::Mat depth_image = cv::imread(depth_path.string());
    return depth_image;
}

} // namespace benchmark_retrieval
