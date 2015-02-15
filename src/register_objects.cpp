#include "object_3d_retrieval/register_objects.h"
#include "sift/sift.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h> // backprojection could be interesting also
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

#define VISUALIZE false

using namespace std;

register_objects::register_objects()
{
}

void register_objects::set_input_clouds(CloudPtrT& t1, const Eigen::Matrix3f& tk1,
                                        CloudPtrT& t2, const Eigen::Matrix3f& tk2)
{
    c1 = t1;
    c2 = t2;
    k1 = tk1;
    k2 = tk2;
}

pair<int, int> register_objects::calculate_image_for_cloud(cv::Mat& image, cv::Mat& depth, CloudPtrT& cloud, const Eigen::Matrix3f& K)
{
    vector<int> xs, ys;
    vector<cv::Vec3b> colors;
    vector<float> depths;
    xs.reserve(cloud->size());
    ys.reserve(cloud->size());
    colors.reserve(cloud->size());
    depths.reserve(cloud->size());

    for (PointT& p : cloud->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (x >= 0 && y >= 0) {
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
    image = cv::Mat::zeros(height, width, CV_8UC3);
    depth = cv::Mat::zeros(height, width, CV_32F);

    for (size_t i = 0; i < xs.size(); ++i) {
        int y = ys[i] - *miny;
        int x = xs[i] - *minx;
        image.at<cv::Vec3b>(y, x) = colors[i];
        depth.at<float>(y, x) = depths[i];
    }

    return make_pair(*minx, *miny);
}

void register_objects::calculate_features_for_image(cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints, CloudPtrT& cloud, cv::Mat& image,
                                                    cv::Mat& depth, int minx, int miny, const Eigen::Matrix3f& K)
{
    //cv::SIFT::DetectorParams detector_params;
    //detector_params.edgeThreshold = 15.0; // 10.0 default
    //detector_params.threshold = 0.04; // 0.04 default
    //cv::SiftFeatureDetector detector(detector_params);
    cv::FastFeatureDetector detector;

    //cv::StarFeatureDetector detector;
    //cv::MserFeatureDetector detector;
    detector.detect(image, keypoints);

    //-- Step 2: Calculate descriptors (feature vectors)
    //cv::Mat descriptors;
    // the length of the descriptors is 128
    // the 3D sift keypoint detectors are probably very unreliable
    cv::SIFT::DescriptorParams descriptor_params;
    descriptor_params.isNormalize = true; // always true, shouldn't matter
    descriptor_params.magnification = 3.0; // 3.0 default
    descriptor_params.recalculateAngles = true; // true default

    cv::SiftDescriptorExtractor extractor;
    //cv::BriefDescriptorExtractor extractor;
    //sift.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute(image, keypoints, descriptors);

    // get back to 3d coordinates
    for (cv::KeyPoint k : keypoints) {
        cv::Point2f p2 = k.pt;
        Eigen::Vector3f p3;
        p3(0) = p2.x + float(minx);
        p3(1) = p2.y + float(miny);
        p3(2) = 1.0f;
        p3 = K.colPivHouseholderQr().solve(p3);
        p3 *= depth.at<float>(int(p2.y), int(p2.x))/p3(2);
        PointT p;
        p.getVector3fMap() = p3;
        cloud->push_back(p);
        //cout << p.getVector3fMap().transpose() << endl;
    }
}

void register_objects::initial_alignment()
{
    auto compute_fpfhs = [] (CloudPtrT& cloud)
    {
        pcl::NormalEstimationOMP<PointT, NormalT> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<PointT>::Ptr fpfh_tree(new pcl::search::KdTree<PointT>);
        ne.setSearchMethod(fpfh_tree);
        NormalCloudPtrT normals(new NormalCloudT);
        ne.setRadiusSearch(0.03);
        ne.compute(*normals);

        pcl::FPFHEstimation<PointT, NormalT> fe;
        fe.setInputCloud(cloud);
        fe.setInputNormals(normals);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);
        fe.setRadiusSearch(0.05);
        fe.compute(*fpfhs);

        return fpfhs;
    };

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1 = compute_fpfhs(c1);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2 = compute_fpfhs(c2);

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> ce;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    ce.setInputSource(fpfhs1);
    ce.setInputTarget(fpfhs2);
    //ce.setSearchMethodSource();
    //ce.setSearchMethodTarget();
    ce.determineCorrespondences(*correspondences);

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> cr;
    pcl::Correspondences sac_correspondences;
    cr.setInputSource(c1);
    cr.setInputTarget(c2);
    cr.setInputCorrespondences(correspondences);
    cr.setInlierThreshold(0.02);
    cr.setMaximumIterations(10000);
    cr.getCorrespondences(sac_correspondences);
    T = cr.getBestTransformation();

    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation(*c1, *c2, sac_correspondences, T);

    if (sac_correspondences.empty()) { // No samples could be selected
        cout << "Still can't find any transformation" << endl;
    }
    else {
        cout << "Found a nice transformation!" << endl;
    }

}

void register_objects::do_registration()
{
    // have something that directly checks if the size difference is too big etc

    cv::Mat image1, image2, depth1, depth2;
    int x1, y1, x2, y2;
    tie(x1, y1) = calculate_image_for_cloud(image1, depth1, c1, k1);
    tie(x2, y2) = calculate_image_for_cloud(image2, depth2, c2, k2);
    cv::Mat descriptors1, descriptors2;
    CloudPtrT cloud1(new CloudT); // cloud corresponding to keypoints1
    CloudPtrT cloud2(new CloudT); // cloud corresponding to keypoints2
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    calculate_features_for_image(descriptors1, keypoints1, cloud1, image1, depth1, x1, y1, k1);
    calculate_features_for_image(descriptors2, keypoints2, cloud2, image2, depth2, x2, y2, k2);

    // matching descriptors
    //cv::BFMatcher matcher;
    cv::FlannBasedMatcher matcher;
    vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches); // query / train

    if (VISUALIZE) {
        cv::namedWindow("matches", 1);
        cv::Mat img_matches;
        cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, img_matches);
        cv::imshow("matches", img_matches);
        cv::waitKey(0);
    }

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    // do ransac on all the matches to find the transformation
    for (cv::DMatch m : matches) {
        pcl::Correspondence c;
        c.index_match = m.trainIdx;
        c.index_query = m.queryIdx;
        c.distance = m.distance;
        correspondences->push_back(c);
    }

    // TODO: add a check here to see if it's actually possible to estimate transformation
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> cr;
    pcl::Correspondences sac_correspondences;
    cr.setInputSource(cloud1);
    cr.setInputTarget(cloud2);
    cr.setInputCorrespondences(correspondences);
    cr.setInlierThreshold(0.02);
    cr.setMaximumIterations(10000);
    cr.getCorrespondences(sac_correspondences);
    T = cr.getBestTransformation();

    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation(*cloud1, *cloud2, sac_correspondences, T);

    if (sac_correspondences.empty() || correspondences->size() == sac_correspondences.size()) { // No samples could be selected
        // alternative way of estimating the transformation that doesn't depend as heavily on keypoints
        initial_alignment();
    }

    cout << "Estimated transformation: " << endl;
    cout << T << endl;

    CloudT::Ptr new_cloud(new CloudT);
    pcl::transformPointCloud(*c1, *new_cloud, T);

    if (VISUALIZE) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(c2);
        viewer->addPointCloud<PointT>(c2, rgb1, "cloud1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(new_cloud);
        viewer->addPointCloud<PointT>(new_cloud, rgb2, "cloud2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }

}

double register_objects::get_match_score()
{
    // transform one cloud into the coordinate frame of the other
    CloudT::Ptr new_cloud(new CloudT);
    pcl::transformPointCloud(*c1, *new_cloud, T);

    double mean_color = 0.0;
    double mean_dist = 0.0;
    size_t counter = 0;

    // use kdtree to find all of the reasonably close points
    pcl::KdTreeFLANN<PointT> kdtree1;
    kdtree1.setInputCloud(new_cloud);
    for (const PointT& p : c2->points) {
        vector<int> indices;
        vector<float> distances;
        kdtree1.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty() || distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = distances[0];
        if (dist > 0.1) {
            continue;
        }
        PointT q = c1->at(indices[0]);
        // compare distance and color
        Eigen::Vector3d pc(p.r, p.g, p.b);
        Eigen::Vector3d qc(q.r, q.g, q.b);
        mean_color += (pc - qc).norm();
        mean_dist += dist;
        ++counter;
    }

    pcl::KdTreeFLANN<PointT> kdtree2;
    kdtree2.setInputCloud(c2);
    for (const PointT& p : new_cloud->points) {
        vector<int> indices;
        vector<float> distances;
        kdtree2.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty() || distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = distances[0];
        if (dist > 0.1) {
            continue;
        }
        PointT q = c2->at(indices[0]);
        // compare distance and color
        Eigen::Vector3d pc(p.r, p.g, p.b);
        Eigen::Vector3d qc(q.r, q.g, q.b);
        mean_color += (pc - qc).norm();
        mean_dist += dist;
        ++counter;
    }

    mean_color *= 1.0/double(counter);
    mean_dist *= 1.0/double(counter);

    // blah, how to weigh distance and color, we need some learning here
    // probability 0-1 of being the same object, how to go from 0-1 to (1-e, 1+e)
    // use the logarithm for now
    double color_weight = 0.05;
    double dist = 1.0/(color_weight*mean_color+mean_dist);

    //double weight = std::max(1.0 - 2*log(color_weight*mean_color+mean_dist), 1.0);
    double weight = std::max(1.0, 5.0*fabs(dist));
    //double weight = -log(color_weight*mean_color+mean_dist);

    cout << "Distance: " << dist << endl;
    cout << "Weight: " << weight << endl;

    return 1.0/mean_dist;
}

void register_objects::get_transformation(Eigen::Matrix4f& trans)
{
    trans = T;
}
