#include "object_3d_benchmark/benchmark_visualization.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

#include "object_3d_benchmark/surfel_renderer.h"
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <dynamic_object_retrieval/visualize.h>

using namespace std;

namespace benchmark_retrieval {

cv::Mat make_visualization_image(cv::Mat& query_image, const string& query_label, vector<CloudT::Ptr>& clouds,
                                 vector<boost::filesystem::path>& sweep_paths, const vector<string>& optional_text,
                                 const Eigen::Matrix4f& T)
{
    /*vector<CloudT::Ptr> original_clouds;
    for (CloudT::Ptr& c : clouds) {
        original_clouds.push_back(CloudT::Ptr(new CloudT(*c)));
        for (PointT& p : c->points) {
            p.getVector4fMap() = T*p.getVector4fMap();
        }
    }*/
    cv::Mat result_image;
    vector<cv::Mat> individual_images;
    tie(result_image, individual_images) = make_image(clouds, T, sweep_paths, optional_text);
    return add_query_image(result_image, query_image, query_label);
    //return result_image;
}

cv::Mat make_visualization_image(cv::Mat& query_image, const string& query_label, vector<CloudT::Ptr>& clouds,
                                 vector<boost::filesystem::path>& sweep_paths, const vector<string>& optional_text,
                                 const Eigen::Matrix4f& T, vector<cv::Mat>& individual_images)
{
    cv::Mat result_image;
    tie(result_image, individual_images) = make_image(clouds, T, sweep_paths, optional_text);
    individual_images.insert(individual_images.begin(), query_image.clone());
    return add_query_image(result_image, query_image, query_label);
}

cv::Mat make_visualization_image(CloudT::Ptr& query_cloud, cv::Mat& query_mask, const boost::filesystem::path& query_path,
                                 const Eigen::Matrix3f& K, const Eigen::Matrix4f& query_transform,
                                 const string& query_label, vector<CloudT::Ptr>& clouds,
                                 vector<boost::filesystem::path>& sweep_paths, const vector<string>& optional_text,
                                 const Eigen::Matrix4f& T)
{
    boost::filesystem::path surfels_path = query_path.parent_path() / "surfel_map.pcd";
    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);

    cout << "Loading surfel cloud: " << surfels_path.string() << endl;

    pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);

    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);

    // now, associate each point in segment with a surfel in the surfel cloud!
    SurfelCloudT::Ptr render_cloud(new SurfelCloudT);
    for (PointT p : query_cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        SurfelT s; s.x = p.x; s.y = p.y; s.z = p.z;
        kdtree.nearestKSearchT(s, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        render_cloud->push_back(surfel_cloud->at(indices[0]));
    }

    cv::Mat image = render_surfel_image(render_cloud, query_transform, K, 480, 640);
    cv::Mat inverted_mask;
    cv::bitwise_not(query_mask, inverted_mask);
    image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
    cv::Mat result_image;
    vector<cv::Mat> individual_images;
    tie(result_image, individual_images) = make_image(clouds, T, sweep_paths, optional_text);
    return add_query_image(result_image, image, query_label);
    //return result_image;
}

cv::Mat add_query_image(cv::Mat& results, cv::Mat& query_image, const std::string& query_label)
{

    cv::Mat gray_query;
    cv::cvtColor(query_image, gray_query, CV_BGR2GRAY);
    gray_query = gray_query != 255;

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

    cv::Mat cropped_query;
    cv::Rect cropped_region = cv::Rect(minx, miny, maxx-minx+1, maxy-miny+1);
    query_image(cropped_region).copyTo(cropped_query);
    if (cropped_query.cols == 0 || cropped_query.rows == 0) {
        query_image.copyTo(cropped_query);
    }
    //cropped_query.setTo(cv::Scalar(255, 255, 255), gray_query(cropped_region) < 5);

    int resized_width = int(double(results.rows)/double(cropped_query.rows)*double(cropped_query.cols));
    if (resized_width == 0) {
        resized_width = 300;
    }
    cv::Mat resized_query;


    cout << cropped_query.cols << ", " << cropped_query.rows << endl;
    cout << resized_width << ", " << results.rows << endl;
    cv::resize(cropped_query, resized_query, cv::Size(resized_width, results.rows), 0, 0, cv::INTER_CUBIC);

    //put_text(resized_query, query_label);

    cv::Mat result_image(results.rows, results.cols + resized_query.cols, CV_8UC3);
    cv::Mat left(result_image, cv::Rect(0, 0, resized_query.cols, resized_query.rows)); // Copy constructor
    resized_query.copyTo(left);
    cv::Mat right(result_image, cv::Rect(resized_query.cols, 0, results.cols, results.rows)); // Copy constructor
    results.copyTo(right);

    return result_image;
}

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

void put_text(cv::Mat& image, const string& text)
{
    cv::putText(image, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.4, cv::Scalar(0,0,255), 1, CV_AA);
}

cv::Mat project_image(CloudT::Ptr& transformed_cloud, const Eigen::Matrix3f& K, size_t height, size_t width)
{
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

    interpolate_projection(sub_image);

    return sub_image;
}

cv::Mat render_image(CloudT::Ptr& cloud, const Eigen::Matrix4f& T, const Eigen::Matrix3f& K,
                     size_t height, size_t width, const boost::filesystem::path& sweep_path)
{
    boost::filesystem::path surfels_path = sweep_path.parent_path() / "surfel_map.pcd";
    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);

    cout << "Loading surfel cloud: " << surfels_path.string() << endl;

    pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);

    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);

    // now, associate each point in segment with a surfel in the surfel cloud!
    SurfelCloudT::Ptr render_cloud(new SurfelCloudT);
    for (PointT p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        SurfelT s; s.x = p.x; s.y = p.y; s.z = p.z;
        kdtree.nearestKSearchT(s, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        render_cloud->push_back(surfel_cloud->at(indices[0]));
    }

    cout << "Render cloud size: " << render_cloud->size() << endl;
    cout << "Original cloud size: " << cloud->size() << endl;

    cv::Mat sub_image = render_surfel_image(render_cloud, T, K, height, width);

    return sub_image;
}

// the question is if we actually need the paths for this?
pair<cv::Mat, vector<cv::Mat> > make_image(std::vector<CloudT::Ptr>& results, const Eigen::Matrix4f& room_transform,
                                           vector<boost::filesystem::path>& sweep_paths, const std::vector<std::string>& optional_text)
{
    pair<int, int> sizes = make_pair(1, std::min(10, int(results.size())));//get_similar_sizes(results.size());

    int width = 200;
    int height = 200;

    cv::Mat visualization = cv::Mat::zeros(height*sizes.first, width*sizes.second, CV_8UC3);
    vector<cv::Mat> individual_images;

    int counter = 0;
    for (CloudT::Ptr& cloud : results) {

        // OK, how do we render the object into the camera? maybe simply simulate a camera,
        // then resize all of the images to have similar sizes? SOUNDS GOOD!

        // how can we get the camera transform for the segments? would it be better to
        // just check the main direction of the segment and estimate the transform based on that? YES

        // so, with the camera matrix K and the estimated transform we just project the points
        // manually into an opencv mat? Sounds reasonable

        CloudT::Ptr centered_cloud(new CloudT);
        pcl::transformPointCloud(*cloud, *centered_cloud, room_transform);

        Eigen::Vector4f point;
        pcl::compute3DCentroid(*centered_cloud, point);

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
        pcl::transformPointCloud(*centered_cloud, *transformed_cloud, T);
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

        cv::Mat sub_image;

        if (false) {//counter % 2 == 0) {
            sub_image = project_image(transformed_cloud, K, height, width);
        }
        else {
            sub_image = render_image(cloud, T*room_transform, K, height, width, sweep_paths[counter]);
        }

        if (!optional_text.empty()) {
            put_text(sub_image, optional_text[counter]);
        }

        int offset_height = counter / sizes.second;
        int offset_width = counter % sizes.second;
        sub_image.copyTo(visualization(cv::Rect(offset_width*width, offset_height*height, width, height)));
        individual_images.push_back(sub_image);

        //cv::imshow("test", visualization);
        //cv::waitKey();

        ++counter;
    }

    if (visualization.rows == 0) {
        visualization = cv::Mat::zeros(height*2, width*5, CV_8UC3);
    }

    return make_pair(visualization, individual_images);
}

} // namespace benchmark_retrieval
