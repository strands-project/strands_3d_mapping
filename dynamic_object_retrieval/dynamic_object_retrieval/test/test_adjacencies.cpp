#include <dynamic_object_retrieval/visualize.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <cereal/types/set.hpp>
#include <cereal/archives/binary.hpp>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>
#include <dynamic_object_retrieval/summary_iterators.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
//using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

/**
 * Some thoughts on were we need to look to improve the convex segmentation:
 *
 * We need to cut more often, e.g. the sofa in the first part of this sequence should not be attached to the wall
 *
 * We need to make sure that the PCL supervoxel segmentation returns the whole scene
 *
 */

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide data path and vocabulary path..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    size_t counter = 0;
    for (const string& xml : folder_xmls) {
        if (counter > 10 && counter < folder_xmls.size() - 10) {
            ++counter;
            continue;
        }
        boost::filesystem::path xml_path(xml);
        dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(xml_path.parent_path());

        CloudT::Ptr visualization_cloud(new CloudT);

        CloudT::Ptr centroids(new CloudT);
        for (CloudT::Ptr& c : segments) {
            *visualization_cloud += *c;
            Eigen::Vector4f point;
            pcl::compute3DCentroid(*c, point);
            centroids->push_back(PointT());
            centroids->back().getVector4fMap() = point;
        }

        stringstream ss;
        ss << "group" << setfill('0') << setw(6) << counter;
        boost::filesystem::path adjacencies_path = vocabulary_path / "vocabulary_vectors" / ss.str() / "adjacencies.cereal";
        cout << "Reading " << adjacencies_path.string() << endl;
        set<pair<int, int> > adjacencies;
        ifstream ina(adjacencies_path.string());
        {
            cereal::BinaryInputArchive archive_i(ina);
            archive_i(adjacencies);
        }
        ina.close();

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor(1, 1, 1);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(visualization_cloud);
        viewer->addPointCloud<PointT>(visualization_cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

        int i = 0;
        for (const pair<int, int>& a : adjacencies) {
            string line_id = string("line") + to_string(i);
            cout << "Centroids size: " << centroids->size() << endl;
            cout << "Adjacency: (" << a.first << ", " << a.second << ")" << endl;
            viewer->addLine<PointT>(centroids->at(a.first), centroids->at(a.second), 1.0, 0.0, 0.0, line_id);
            ++i;
        }

        //viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }

        ++counter;
    }

    return 0;
}

