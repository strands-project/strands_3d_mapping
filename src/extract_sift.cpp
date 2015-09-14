#include "dynamic_object_retrieval/visualize.h"

#include <pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>

#include <extract_sift/extract_sift.h>
#include <sift/sift.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

POINT_CLOUD_REGISTER_POINT_STRUCT (SiftT,
                                   (float[128], histogram, histogram)
)

namespace extract_sift {

void crop_cloud(CloudT::Ptr& cloud, const Eigen::Matrix4f& T, const Eigen::Vector4f& p)
{
    // for now, just replace all other points with inf
    // this keeps the structure of the points, which lets us extract sift features later
    for (PointT& point : cloud->points) {
        if (p.transpose()*T*point.getVector4fMap() > 0.0f) {
            point.x = std::numeric_limits<float>::infinity();
            point.y = std::numeric_limits<float>::infinity();
            point.z = std::numeric_limits<float>::infinity();
        }
    }
}

void visualize_nonoverlapping(vector<CloudT::Ptr>& cropped_clouds, std::vector<tf::StampedTransform>& transforms)
{
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

    CloudT::Ptr sweep_cloud(new CloudT);
    for (int i = 0; i < 17; ++i) {
        CloudT::Ptr cloud(new CloudT);
        Eigen::Affine3d e;
        tf::transformTFToEigen(transforms[i], e);
        pcl::transformPointCloud(*cropped_clouds[i], *cloud, e);
        for (PointT& p : cloud->points) {
            p.r = colormap[i%24][0];
            p.g = colormap[i%24][1];
            p.b = colormap[i%24][2];
        }
        *sweep_cloud += *cloud;
    }

    dynamic_object_retrieval::visualize(sweep_cloud, 0.05f);
}

void extract_nonoverlapping_sift(SiftCloudT::Ptr& sweep_features, CloudT::Ptr& sweep_keypoints,
                                 vector<CloudT::Ptr>& cropped_clouds, std::vector<tf::StampedTransform>& transforms)
{
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

    for (int i = 0; i < 17; ++i) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(transforms[i], e);

        cv::Mat img(480, 640, CV_8UC3);
        for (int y = 0; y < 480; ++y) {
            for (int x = 0; x < 640; ++x) {
                int ind = y*640+x;
                cv::Vec3b& c = img.at<cv::Vec3b>(y, x);
                c[2] = cropped_clouds[i]->at(ind).r;
                c[1] = cropped_clouds[i]->at(ind).g;
                c[0] = cropped_clouds[i]->at(ind).b;
            }
        }
        cv::FastFeatureDetector detector;
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(img, keypoints);
        cv::SIFT::DescriptorParams descriptor_params;
        descriptor_params.isNormalize = true; // always true, shouldn't matter
        descriptor_params.magnification = 3.0; // 3.0 default
        descriptor_params.recalculateAngles = true; // true default
        cv::SiftDescriptorExtractor extractor;
        cv::Mat descriptors;
        extractor.compute(img, keypoints, descriptors);

        SiftCloudT::Ptr feature_cloud(new SiftCloudT);
        CloudT::Ptr keypoint_cloud(new CloudT);
        int j = 0;
        for (cv::KeyPoint k : keypoints) {
            const cv::Point2f& p2 = k.pt;
            int ind = p2.y*640+p2.x;
            const PointT& p = cropped_clouds[i]->at(ind);
            if (pcl::isFinite(p)) {
                PointT newp = pcl::transformPoint(p, e);
                newp.r = colormap[i%24][0];
                newp.g = colormap[i%24][1];
                newp.b = colormap[i%24][2];
                keypoint_cloud->push_back(newp);
                SiftT sp;
                for (int k = 0; k < 128; ++k) {
                    sp.histogram[k] = descriptors.at<float>(j, k);
                }
                feature_cloud->push_back(sp);
            }
            ++j;
        }

        *sweep_features += *feature_cloud;
        *sweep_keypoints += *keypoint_cloud;
    }
}

void extract_sift_for_sweep(const boost::filesystem::path& xml_path)
{
    using scan_data = semantic_map_load_utilties::IntermediateCloudCompleteData<PointT>;
    scan_data data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(xml_path.string());

    vector<CloudT::Ptr> cropped_clouds;

    for (CloudT::Ptr& c : data.vIntermediateRoomClouds) {
        cropped_clouds.push_back(CloudT::Ptr(new CloudT(*c)));
    }

    for (int i = 0; i < 17; ++i) {
        // check overlap with i+1 mod size, also cut based on previous comparison
        int next = (i+1)%17;

        tf::StampedTransform T = data.vIntermediateRoomCloudTransforms[i];
        Eigen::Affine3d e;
        tf::transformTFToEigen(T, e);
        Eigen::Matrix4f current_transform = e.matrix().cast<float>();
        Eigen::Vector3f current_direction = current_transform.block<3, 1>(0, 2); // might be (0, 0) also, might also be the rows

        T = data.vIntermediateRoomCloudTransforms[next];
        tf::transformTFToEigen(T, e);
        Eigen::Matrix4f next_transform = e.matrix().cast<float>();
        Eigen::Vector3f next_direction = next_transform.block<3, 1>(0, 2); // might be (0, 0) also, might also be the rows

        // this will be in a plane in between the camera directions
        Eigen::Vector3f in_plane = current_direction.cross(next_direction);
        Eigen::Vector3f z_axis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f normal = z_axis.cross(in_plane);

        // in between translations of scans
        Eigen::Vector3f plane_point = 0.5f*(current_transform.block<3, 1>(0, 3)+next_transform.block<3, 1>(0, 3));
        float d = -plane_point.dot(normal);

        // plane parameters
        Eigen::Vector4f p;
        p.head<3>() = normal;
        p(3) = d;

        crop_cloud(cropped_clouds[i], current_transform, p);
        crop_cloud(cropped_clouds[next], next_transform, -p);
    }

    //visualize_nonoverlapping(cropped_clouds, data.vIntermediateRoomCloudTransforms);

    SiftCloudT::Ptr sweep_features(new SiftCloudT);
    CloudT::Ptr sweep_keypoints(new CloudT);
    extract_nonoverlapping_sift(sweep_features, sweep_keypoints, cropped_clouds, data.vIntermediateRoomCloudTransforms);
    dynamic_object_retrieval::visualize(sweep_keypoints);

    pcl::io::savePCDFileBinary((xml_path.parent_path() / "sift_features.pcd").string(), *sweep_features);
    pcl::io::savePCDFileBinary((xml_path.parent_path() / "sift_keypoints.pcd").string(), *sweep_keypoints);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const boost::filesystem::path& cloud_path)
{
    CloudT::Ptr cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *cloud);
    return get_sift_for_cloud_path(cloud_path, cloud);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const vector<boost::filesystem::path>& cloud_paths)
{
    CloudT::Ptr cloud(new CloudT);
    for (boost::filesystem::path cloud_path : cloud_paths) {
        CloudT::Ptr keypoints(new CloudT);
        pcl::io::loadPCDFile(cloud_path.string(), *keypoints);
        *cloud += *keypoints;
    }
    return get_sift_for_cloud_path(cloud_paths[0], cloud);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const boost::filesystem::path& cloud_path, CloudT::Ptr& cloud)
{
    boost::filesystem::path sweep_path = cloud_path.parent_path().parent_path();
    SiftCloudT::Ptr features(new SiftCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pcl::io::loadPCDFile((sweep_path / "sift_features.pcd").string(), *features);
    pcl::io::loadPCDFile((sweep_path / "sift_keypoints.pcd").string(), *keypoints);

    // now we should check the intersection using e.g. an octree
    // pick all the sift keypoints close enough to a point in keypoints
    pcl::octree::OctreePointCloudSearch<PointT> octree(0.1f);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    SiftCloudT::Ptr overlapping_features(new SiftCloudT);
    CloudT::Ptr overlapping_keypoints(new CloudT);
    int counter = 0;
    for (const PointT& p : keypoints->points) {
        if (octree.isVoxelOccupiedAtPoint(p)) {
            overlapping_features->push_back(features->at(counter));
            overlapping_keypoints->push_back(p);
        }
        ++counter;
    }

    return make_pair(overlapping_features, overlapping_keypoints);
}

} // namespace extract_sift
