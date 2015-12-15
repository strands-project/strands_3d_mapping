#include "dynamic_object_retrieval/visualize.h"

#include <pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>

#include <extract_sift/extract_sift.h>
#include <sift/sift.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
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

void extract_sift_for_sweep(const boost::filesystem::path& xml_path, bool visualize)
{
    using scan_data = semantic_map_load_utilties::IntermediateCloudCompleteData<PointT>;
    scan_data data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(xml_path.string());

    auto sweep_data = SimpleXMLParser<PointT>::loadRoomFromXML(xml_path.string(), std::vector<std::string>{"RoomIntermediateCloud"}, false);

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
    extract_nonoverlapping_sift(sweep_features, sweep_keypoints, cropped_clouds, sweep_data.vIntermediateRoomCloudTransformsRegistered);

    if (visualize) {
        dynamic_object_retrieval::visualize(sweep_keypoints);
    }

    pcl::io::savePCDFileBinary((xml_path.parent_path() / "sift_features.pcd").string(), *sweep_features);
    pcl::io::savePCDFileBinary((xml_path.parent_path() / "sift_keypoints.pcd").string(), *sweep_keypoints);
}

tuple<cv::Mat, cv::Mat, int, int> compute_image_for_cloud(CloudT::Ptr& cloud, const Eigen::Matrix3f& K)
{
    vector<int> xs, ys;
    vector<cv::Vec3b> colors;
    vector<float> depths;
    xs.reserve(cloud->size());
    ys.reserve(cloud->size());
    colors.reserve(cloud->size());
    depths.reserve(cloud->size());

    for (PointT& p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (true) {//x >= 0 && y >= 0) {
            xs.push_back(x);
            ys.push_back(y);
            cv::Vec3b temp;
            temp[0] = p.b;
            temp[1] = p.g;
            temp[2] = p.r;
            colors.push_back(temp);
            depths.push_back(p.z);
        }
    }

    std::vector<int>::iterator minx = std::min_element(xs.begin(), xs.end());
    std::vector<int>::iterator maxx = std::max_element(xs.begin(), xs.end());
    std::vector<int>::iterator miny = std::min_element(ys.begin(), ys.end());
    std::vector<int>::iterator maxy = std::max_element(ys.begin(), ys.end());

    int height = *maxy - *miny + 1;
    int width = *maxx - *minx + 1;

    cout << "Height: " << height << endl;
    cout << "Width: " << width << endl;

    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat depth = cv::Mat::zeros(height, width, CV_32F);

    for (size_t i = 0; i < xs.size(); ++i) {
        int y = ys[i] - *miny;
        int x = xs[i] - *minx;
        image.at<cv::Vec3b>(y, x) = colors[i];
        depth.at<float>(y, x) = depths[i];
    }

    return make_tuple(image, depth, *minx, *miny);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> extract_sift_features(cv::Mat& image, cv::Mat& depth, int minx, int miny, const Eigen::Matrix3f& K)
{
    std::vector<cv::KeyPoint> cv_keypoints;
    cv::FastFeatureDetector detector;
    detector.detect(image, cv_keypoints);

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SIFT::DescriptorParams descriptor_params;
    descriptor_params.isNormalize = true; // always true, shouldn't matter
    descriptor_params.magnification = 3.0; // 3.0 default
    descriptor_params.recalculateAngles = true; // true default

    cv::Mat cv_features;
    cv::SiftDescriptorExtractor extractor;
    extractor.compute(image, cv_keypoints, cv_features);

    CloudT::Ptr keypoints(new CloudT);
    // get back to 3d coordinates
    for (cv::KeyPoint k : cv_keypoints) {
        cv::Point2f p2 = k.pt;
        Eigen::Vector3f p3;
        p3(0) = p2.x + float(minx);
        p3(1) = p2.y + float(miny);
        p3(2) = 1.0f;
        p3 = K.colPivHouseholderQr().solve(p3);
        uint16_t depth_val = depth.at<uint16_t>(int(p2.y), int(p2.x));
        if (depth_val == 0) {
            continue;
        }
        p3 *= float(depth_val)/p3(2)/10000.0f;
        PointT p;
        p.getVector3fMap() = p3;
        keypoints->push_back(p);
    }

    //cv::Mat img_keypoints;
    //cv::drawKeypoints(image, cv_keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //cv::imshow("my keypoints", img_keypoints);
    //cv::waitKey(0);

    SiftCloudT::Ptr features(new SiftCloudT);
    features->reserve(cv_features.rows);
    for (int j = 0; j < cv_features.rows; ++j) {
        // we need to check if the points are finite
        SiftT sp;
        for (int k = 0; k < 128; ++k) {
            sp.histogram[k] = cv_features.at<float>(j, k);
        }
        features->push_back(sp);
    }

    return make_pair(features, keypoints);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> extract_sift_for_image(cv::Mat& image, cv::Mat& depth, const Eigen::Matrix3f& K)
{
    // remove the black around the objects
    cv::Mat gray_query;
    cv::cvtColor(image, gray_query, CV_BGR2GRAY);
    //gray_query = gray_query != 0;

    cv::Mat row_sum, col_sum;
    cv::reduce(gray_query, row_sum, 1, CV_REDUCE_SUM, CV_32S);
    cv::reduce(gray_query, col_sum, 0, CV_REDUCE_SUM, CV_32S);

    int minx = gray_query.cols;
    int maxx = 0;
    int miny = gray_query.rows;
    int maxy = 0;

    for (int i = 0; i < gray_query.rows; ++i) {
        if (row_sum.at<int32_t>(i) > 0) {
            miny = i < miny? i : miny;
            maxy = i > maxy? i : maxy;
        }
    }

    for (int i = 0; i < gray_query.cols; ++i) {
        if (col_sum.at<int32_t>(i) > 0) {
            minx = i < minx? i : minx;
            maxx = i > maxx? i : maxx;
        }
    }

    cv::Mat cropped_image;
    cv::Mat cropped_depth;
    cv::Rect cropped_region = cv::Rect(minx, miny, maxx-minx+1, maxy-miny+1);
    image(cropped_region).copyTo(cropped_image);
    depth(cropped_region).copyTo(cropped_depth);

    //cv::imshow("cropped_image", cropped_image);
    //cv::imshow("cropped_depth", cropped_depth);
    //cv::waitKey();

    return extract_sift_features(cropped_image, cropped_depth, minx, miny, K);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> extract_sift_for_cloud(CloudT::Ptr& cloud, const Eigen::Matrix3f& K)
{
    int minx, miny;
    cv::Mat image, depth;
    tie(image, depth, minx, miny) = compute_image_for_cloud(cloud, K);
    //cv::imshow("Generated image", image);
    //cv::waitKey();
    SiftCloudT::Ptr features;
    CloudT::Ptr keypoints;
    tie(features, keypoints) = extract_sift_features(image, depth, minx, miny, K);
    return make_pair(features, keypoints);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const boost::filesystem::path& cloud_path)
{
    cout << "With path " << cloud_path.string() << endl;
    CloudT::Ptr cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *cloud);
    return get_sift_for_cloud_path(cloud_path, cloud);
}

pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const vector<boost::filesystem::path>& cloud_paths)
{
    CloudT::Ptr cloud(new CloudT);
    for (boost::filesystem::path cloud_path : cloud_paths) {
        cout << "Loading cloud: " << cloud_path.string() << endl;
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

    CloudT::Ptr vis_cloud(new CloudT);
    *vis_cloud += *keypoints;
    *vis_cloud += *cloud;
    //dynamic_object_retrieval::visualize(vis_cloud);

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
