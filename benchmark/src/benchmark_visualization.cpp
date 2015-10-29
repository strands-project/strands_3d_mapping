#include "object_3d_benchmark/benchmark_visualization.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

using namespace std;

namespace benchmark_retrieval {

// actually, we should start in the middle and just take
// the first even dividend, but this works too
pair<int, int> get_similar_sizes(int i)
{
    int smallest = i;
    pair<int, int> sizes;
    for (int j = 1; j < i; ++j) {
        if (i % j == 0) {
            int k = i / j;
            int cand = std::abs(j - k);
            if (cand < smallest) {
                smallest = cand;
                sizes.first = j;
                sizes.second = k;
            }
        }
    }
    return sizes;
}

pair<Eigen::Matrix4f, Eigen::Matrix3f> get_camera_parameters_for_cloud(CloudT::Ptr& cloud)
{

}

void interpolate_projection(cv::Mat& image)
{
    // Convert image to grayscale
    cv::Mat filled_in;
    cv::cvtColor(image, filled_in, CV_BGR2GRAY);
    filled_in = filled_in != 255;

    // Dilate to fill holes
    cv::Mat needs_filling;
    cv::dilate(filled_in, needs_filling, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(13,13)));

    needs_filling -= filled_in;

    cv::inpaint(image, needs_filling, image, 5, cv::INPAINT_TELEA);
}

// the question is if we actually need the paths for this?
cv::Mat make_image(std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> >& results)
{
    pair<int, int> sizes = get_similar_sizes(results.size());

    int width = 200;
    int height = 200;

    cv::Mat visualization = cv::Mat::zeros(height*sizes.first, width*sizes.second, CV_8UC3);

    int counter = 0;
    for (auto tup : results) {
        CloudT::Ptr cloud;
        boost::filesystem::path path;
        tie(cloud, path) = tup;

        // OK, how do we render the object into the camera? maybe simply simulate a camera,
        // then resize all of the images to have similar sizes? SOUNDS GOOD!

        // how can we get the camera transform for the segments? would it be better to
        // just check the main direction of the segment and estimate the transform based on that? YES

        // so, with the camera matrix K and the estimated transform we just project the points
        // manually into an opencv mat? Sounds reasonable

        Eigen::Vector4f point;
        pcl::compute3DCentroid(*cloud, point);

        Eigen::Vector3f x, y, z;
        z = point.head<3>();
        z.normalize();
        x = Eigen::Vector3f(0.0f, 0.0f, -1.0f).cross(z);
        y = z.cross(x);

        Eigen::Matrix4f T;
        T.setIdentity();
        T.block<1, 3>(0, 0) = x.transpose();
        T.block<1, 3>(1, 0) = y.transpose();
        T.block<1, 3>(2, 0) = z.transpose();

        CloudT::Ptr transformed_cloud(new CloudT);
        pcl::transformPointCloud(*cloud, *transformed_cloud, T);
        for (PointT& p : transformed_cloud->points) {
            p.getVector3fMap() /= p.z;
        }

        // from this we should be able to compute a good focal length from the given widths
        // probably easiest from a  min max?
        PointT min_pt;
        PointT max_pt;
        pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);

        float gap_x = std::fabs(max_pt.x - min_pt.x);
        float gap_y = std::fabs(max_pt.y - min_pt.y);

        float focal = std::min(float(width)/gap_x, float(height)/gap_y);

        // allright, now all we need is something for computing the offset
        // this should be possible directly from the focal and the min max points
        // I guess just centralize in both directions

        float offset_x = -focal*min_pt.x+0.5f*(float(width)-focal*gap_x);
        float offset_y = -focal*min_pt.y+0.5f*(float(height)-focal*gap_y);

        Eigen::Matrix3f K;
        K << focal, 0.0f, offset_x, 0.0f, focal, offset_y, 0.0f, 0.0f, 1.0f;
        cv::Mat sub_image(height, width, CV_8UC3);
        sub_image = cv::Scalar(255, 255, 255);
        for (const PointT& p : transformed_cloud->points) {
            Eigen::Vector3f cp = K*p.getVector3fMap();
            cp = 1.0f/cp[2]*cp;
            int cx = int(cp[0]);
            int cy = int(cp[1]);
            if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
                continue;
            }
            cv::Vec3b& pixel = sub_image.at<cv::Vec3b>(cy, cx);
            pixel[0] = p.b;
            pixel[1] = p.g;
            pixel[2] = p.r;
        }

        int offset_height = counter / sizes.second;
        int offset_width = counter % sizes.second;

        interpolate_projection(sub_image);
        sub_image.copyTo(visualization(cv::Rect(offset_width*width, offset_height*height, width, height)));

        //cv::imshow("test", visualization);
        //cv::waitKey();

        ++counter;
    }

    return visualization;
}

} // namespace benchmark_retrieval
