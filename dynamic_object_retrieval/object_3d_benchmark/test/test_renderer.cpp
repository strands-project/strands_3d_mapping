#include <object_3d_benchmark/surfel_renderer.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <dynamic_object_retrieval/summary_iterators.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide path to sweep data..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& xml : folder_xmls) {
        boost::filesystem::path xml_path = xml;
        boost::filesystem::path surfels_path = xml_path.parent_path() / "surfel_map.pcd";
        Eigen::Matrix3f K;
        vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
        tie(K, transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(xml);

        SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
        pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);

        // here, I just want to set the file, position and camera
        // parameters (and image size) and render to a cv mat
        size_t height, width;
        height = 480;
        width = 640;

        for (const Eigen::Matrix4f& T : transforms) {
            cv::Mat image = benchmark_retrieval::render_surfel_image(surfel_cloud, T, K, height, width);
            cv::imshow("Surfel Image", image);
            cv::waitKey();
        }
    }
}
