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

float register_objects::sRGB_LUT[256] = {- 1};
float register_objects::sXYZ_LUT[4000] = {- 1};

register_objects::register_objects()
{

}

void register_objects::set_input_clouds(CloudPtrT& t1, CloudPtrT& t2)
{
    c1 = t1;
    c2 = t2;
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

    if (descriptors1.empty() || descriptors2.empty()) {
        // shit just hit the fan
        T.setIdentity();
        /*if (c1->size() < 50000 && c2->size() < 50000) {
            initial_alignment();
        }*/
        return;
    }

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
    cr.setMaximumIterations(1000);
    cr.getCorrespondences(sac_correspondences);
    T = cr.getBestTransformation();

    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation(*cloud1, *cloud2, sac_correspondences, T);

    if (sac_correspondences.empty() || correspondences->size() == sac_correspondences.size()) { // No samples could be selected
        T.setIdentity();
        // alternative way of estimating the transformation that doesn't depend as heavily on keypoints
        /*if (c1->size() < 50000 && c2->size() < 50000) {
            initial_alignment();
        }*/
    }

    cout << "Estimated transformation: " << endl;
    cout << T << endl;

    if (VISUALIZE) {
        CloudT::Ptr new_cloud(new CloudT);
        pcl::transformPointCloud(*c1, *new_cloud, T);
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

void register_objects::do_registration(SiftCloudT::Ptr& sift_cloud1, SiftCloudT::Ptr& sift_cloud2,
                                       CloudT::Ptr& keypoint_cloud1, CloudT::Ptr& keypoint_cloud2)
{
    cout << "We are actually inside the registration!" << endl;
    if (sift_cloud1->empty() || sift_cloud2->empty()) {
        T.setIdentity();
        return;
    }

    cv::Mat descriptors1(sift_cloud1->size(), 128, CV_32F);
    cv::Mat descriptors2(sift_cloud2->size(), 128, CV_32F);

    for (int i = 0; i < sift_cloud1->size(); ++i) {
        for (int j = 0; j < 128; ++j) {
            descriptors1.at<float>(i, j) = sift_cloud1->at(i).histogram[j];
        }
    }

    for (int i = 0; i < sift_cloud2->size(); ++i) {
        for (int j = 0; j < 128; ++j) {
            descriptors2.at<float>(i, j) = sift_cloud2->at(i).histogram[j];
        }
    }

    // matching descriptors
    //cv::BFMatcher matcher;
    cv::FlannBasedMatcher matcher;
    vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches); // query / train

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
    cr.setInputSource(keypoint_cloud1);
    cr.setInputTarget(keypoint_cloud2);
    cr.setInputCorrespondences(correspondences);
    cr.setInlierThreshold(0.01);
    cr.setMaximumIterations(4000);
    cr.getCorrespondences(sac_correspondences);
    T = cr.getBestTransformation();

    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation(*keypoint_cloud1, *keypoint_cloud2, sac_correspondences, T);

    if (sac_correspondences.empty() || correspondences->size() == sac_correspondences.size()) { // No samples could be selected
        T.setIdentity();
    }

    cout << "Estimated transformation: " << endl;
    cout << T << endl;

    if (VISUALIZE) {
        CloudT::Ptr new_cloud(new CloudT);
        pcl::transformPointCloud(*c1, *new_cloud, T);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(1, 1, 1);
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


// from https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/features/include/pcl/features/impl/shot.hpp
// see project_root/license.txt for details
void register_objects::RGB2CIELAB(unsigned char R, unsigned char G, unsigned char B, float &L, float &A, float &B2)
{
    if (sRGB_LUT[0] < 0) {
        for (int i = 0; i < 256; i++) {
            float f = static_cast<float> (i) / 255.0f;
            if (f > 0.04045) {
                sRGB_LUT[i] = powf ((f + 0.055f) / 1.055f, 2.4f);
            }
            else {
                sRGB_LUT[i] = f / 12.92f;
            }
        }
        for (int i = 0; i < 4000; i++) {
            float f = static_cast<float> (i) / 4000.0f;
            if (f > 0.008856) {
                sXYZ_LUT[i] = static_cast<float> (powf (f, 0.3333f));
            }
            else {
                sXYZ_LUT[i] = static_cast<float>((7.787 * f) + (16.0 / 116.0));
            }
        }
    }

    float fr = sRGB_LUT[R];
    float fg = sRGB_LUT[G];
    float fb = sRGB_LUT[B];

    // Use white = D65
    const float x = fr * 0.412453f + fg * 0.357580f + fb * 0.180423f;
    const float y = fr * 0.212671f + fg * 0.715160f + fb * 0.072169f;
    const float z = fr * 0.019334f + fg * 0.119193f + fb * 0.950227f;

    float vx = x / 0.95047f;
    float vy = y;
    float vz = z / 1.08883f;

    vx = sXYZ_LUT[int(vx*4000)];
    vy = sXYZ_LUT[int(vy*4000)];
    vz = sXYZ_LUT[int(vz*4000)];

    L = 116.0f * vy - 16.0f;
    if (L > 100) {
        L = 100.0f;
    }
    A = 500.0f * (vx - vy);
    if (A > 120) {
        A = 120.0f;
    }
    else if (A <- 120) {
        A = -120.0f;
    }
    B2 = 200.0f * (vy - vz);
    if (B2 > 120) {
        B2 = 120.0f;
    }
    else if (B2<- 120) {
        B2 = -120.0f;
    }
}

pair<double, double> register_objects::get_match_score()
{
    if (c1->empty() || c2->empty()) {
        return make_pair(0.1, 100);
    }

    int c1_finite = 0;
    for (const PointT& p : c1->points) {
        if (pcl::isFinite(p)) {
            ++c1_finite;
        }
    }

    int c2_finite = 0;
    for (const PointT& p : c2->points) {
        if (pcl::isFinite(p)) {
            ++c2_finite;
        }
    }

    cout << "C1 finite: " << double(c1_finite)/double(c1->size()) << endl;
    cout << "C2 finite: " << double(c2_finite)/double(c2->size()) << endl;

    cout << "Entering register objects" << endl;

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
        //cout << p.getVector3fMap().transpose() << endl;
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices(1);
        vector<float> distances(1);
        kdtree1.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty() || distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = sqrt(distances[0]);
        /*if (dist > 0.1) {
            continue;
        }*/
        PointT q = new_cloud->at(indices[0]);
        // compare distance and color
        Eigen::Vector3d pc(p.r, p.g, p.b);
        Eigen::Vector3d qc(q.r, q.g, q.b);
        mean_color += (pc - qc).norm();
        mean_dist += dist;
        ++counter;
    }

    cout << "First part done" << endl;
    cout << "Counter: " << counter << endl;

    pcl::KdTreeFLANN<PointT> kdtree2;
    CloudT::Ptr temp(new CloudT(*c2));
    kdtree2.setInputCloud(temp);
    for (const PointT& p : new_cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices(1);
        vector<float> distances(1);
        kdtree2.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = distances[0];
        if (dist > 0.05) { // make this a parameter
            continue;
        }
        //cout << indices.size() << endl;
        //cout << c2->size() << endl;
        //cout << indices[0] << endl;
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

    if (counter == 0) {
        mean_dist = 0.1; // make this a parameter
        mean_color = 100;
    }

    /*if (mean_dist < 0.003 && mean_dist != 0) {
        mean_dist = 0.003;
    }*/

    cout << "Weight: " << 1.0/mean_dist << endl;

    return make_pair(1.0/mean_dist, 1.0/mean_color);
}

void register_objects::get_transformation(Eigen::Matrix4f& trans)
{
    trans = T;
}

void register_objects::visualize_cloud(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void register_objects::visualize_feature_segmentation(CloudT::Ptr& segment_keypoints, CloudT::Ptr& cloud)
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(segment_keypoints);

    CloudT::Ptr resulting_cloud(new CloudT);
    for (const PointT& p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        kdtree.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = sqrt(distances[0]);
        if (dist < 0.05) {
            resulting_cloud->push_back(p);
        }
    }

    visualize_cloud(resulting_cloud);
}

void register_objects::get_feature_segmentation(CloudT::Ptr& resulting_cloud, CloudT::Ptr& segment_keypoints, CloudT::Ptr& cloud)
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(segment_keypoints);

    for (const PointT& p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        kdtree.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = sqrt(distances[0]);
        if (dist < 0.05) {
            resulting_cloud->push_back(p);
        }
    }
}

void register_objects::register_using_features(PFHCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints,
                                               PFHCloudT::Ptr& segment_features, CloudT::Ptr& segment_keypoints)
{
    if (query_features->empty() || segment_features->empty()) {
        T.setIdentity();
        return;
    }

    cv::Mat descriptors1(query_features->size(), N, CV_32F);
    cv::Mat descriptors2(segment_features->size(), N, CV_32F);

    for (int i = 0; i < query_features->size(); ++i) {
        for (int j = 0; j < N; ++j) {
            descriptors1.at<float>(i, j) = query_features->at(i).histogram[j];
        }
    }

    for (int i = 0; i < segment_features->size(); ++i) {
        for (int j = 0; j < N; ++j) {
            descriptors2.at<float>(i, j) = segment_features->at(i).histogram[j];
        }
    }

    // matching descriptors
    cv::FlannBasedMatcher matcher;
    vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    cout << "Query features size: " << query_features->size() << endl;
    cout << "Segment features size: " << segment_features->size() << endl;
    cout << "Matches size: " << matches.size() << endl;

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
    cr.setInputSource(query_keypoints);
    cr.setInputTarget(segment_keypoints);
    cr.setInputCorrespondences(correspondences);
    cr.setInlierThreshold(0.03);
    cr.setMaximumIterations(10000);
    cr.getCorrespondences(sac_correspondences);
    T = cr.getBestTransformation();

    pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est;
    trans_est.estimateRigidTransformation(*query_keypoints, *segment_keypoints, sac_correspondences, T);

    if (sac_correspondences.empty() || correspondences->size() == sac_correspondences.size()) { // No samples could be selected
        T.setIdentity();
    }

    cout << "Estimated transformation: " << endl;
    cout << T << endl;

    if (VISUALIZE) {
        CloudT::Ptr new_cloud(new CloudT);
        pcl::transformPointCloud(*c1, *new_cloud, T);
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
