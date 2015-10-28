#include "object_3d_benchmark/benchmark_visualization.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

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

// the question is if we actually need the paths for this?
cv::Mat make_image(std::vector<std::pair<CloudT::Ptr, boost::filesystem::path> >& results)
{
    cout << __FILE__ << ", " << __LINE__ << endl;
    pair<int, int> sizes = get_similar_sizes(results.size());

    int width = 200;
    int height = 200;

    cv::Mat visualization = cv::Mat::zeros(height*sizes.first, width*sizes.second, CV_8UC3);

    cout << __FILE__ << ", " << __LINE__ << endl;

    int counter = 0;
    for (auto tup : results) {
        cout << __FILE__ << ", " << __LINE__ << endl;
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

        cout << __FILE__ << ", " << __LINE__ << endl;
        Eigen::Vector3f x, y, z;
        z = point.head<3>();
        z.normalize();
        x = Eigen::Vector3f(0.0f, 0.0f, -1.0f).cross(z);
        y = z.cross(x);
        /*if (x(2) < 0.0f) {
            x = -x;
            y = -y;
        }*/

        cout << __FILE__ << ", " << __LINE__ << endl;
        Eigen::Matrix4f T;
        T.setIdentity();

        /*
        T.block<3, 1>(0, 0) = x;
        T.block<3, 1>(0, 1) = y;
        T.block<3, 1>(0, 2) = z;
        */


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

        float offset_x = -focal*min_pt.x;
        float offset_y = -focal*min_pt.y;

        cout << __FILE__ << ", " << __LINE__ << endl;

        Eigen::Matrix3f K;
        K << focal, 0.0f, offset_x, 0.0f, focal, offset_y, 0.0f, 0.0f, 1.0f;
        cout << __FILE__ << ", " << __LINE__ << endl;
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
        cout << __FILE__ << ", " << __LINE__ << endl;
        int offset_height = counter / sizes.second;
        int offset_width = counter % sizes.second;
        cout << "offset height: " << offset_height << endl;
        cout << "offset width: " << offset_width << endl;
        cout << "size first: " << sizes.first << endl;
        cout << "size second: " << sizes.second << endl;
        cout << "pixel offset height: " << offset_height*height << endl;
        cout << "pixel offset width: " << offset_width*width << endl;
        cout << "actual height: " << visualization.rows << endl;
        cout << "actual width: " << visualization.cols << endl;
        sub_image.copyTo(visualization(cv::Rect(offset_width*width, offset_height*height, width, height)));
        //visualization(cv::Rect(offset_width*width, offset_height*height, width, height)) = sub_image;

        cv::imshow("test", visualization);
        cv::waitKey();

        cout << __FILE__ << ", " << __LINE__ << endl;
        ++counter;
    }

    return visualization;
}

} // namespace benchmark_retrieval
