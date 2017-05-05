#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/visualize.h"
#include <metaroom_xml_parser/load_utilities.h>
#include "retrieval_tools/surfel_type.h"
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

using namespace std;

class point_picker {
public:

    point_picker () : did_pick(false) {
        viewer.reset (new pcl::visualization::PCLVisualizer ("Viewer", true));
        viewer->registerPointPickingCallback (&point_picker::pick_callback, *this);
    }

    ~point_picker () {}

    void set_input_cloud(CloudT::Ptr cloud)
    {
        cloud_temp = cloud;
    }

    PointT get_point() {
        return picked_point;
    }

    bool picked() const {
        return did_pick;
    }

    void show_viewer()
    {
        // Visualizer
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_temp);
        viewer->addPointCloud<PointT>(cloud_temp, rgb, "Cloud");
        viewer->resetCameraViewpoint ("Cloud");
        viewer->spin();
    }

protected:

    void pick_callback(const pcl::visualization::PointPickingEvent& event, void*)
    {
        if (event.getPointIndex () == -1)
            return;

        did_pick = true;

        cout << "Picked a point!" << endl;
        event.getPoint(picked_point.x,picked_point.y,picked_point.z);

        //cout<<"first selected point: "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
        //cout<<"second selected point: "<<p[3]<<" "<<p[4]<<" "<<p[5]<<endl;
    }

private:
    // Point cloud data
    CloudT::Ptr cloud_temp;

    // The visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // The picked point
    PointT picked_point;
    bool did_pick;
};

int main(int argc, char** argv)
{
    const int colormap[][3] = {
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

    if (argc < 2) {
        cout << "Please provide the path to the sweep..." << endl;
        return 0;
    }
    boost::filesystem::path sweep_path = boost::filesystem::path(argv[1]);

    cout << "Creating iterators..." << endl;

    // get all convex segments for this sweep
    dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(sweep_path);
    dynamic_object_retrieval::sweep_convex_segment_index_map indices(sweep_path);


    cout << "Analyzing convex segments..." << endl;

    CloudT::Ptr colored_cloud(new CloudT);
    vector<CloudT::Ptr> clouds;
    for (auto tup : dynamic_object_retrieval::zip(segments, indices)) {
        // here it seems like we're gonna have to wrap our own solution by wrapping two octrees (or voxelgrids?)
        CloudT::Ptr c;
        size_t index;
        tie(c, index) = tup;
        clouds.push_back(CloudT::Ptr(new CloudT(*c)));

        cout << "Index: " << index << endl;

        // now, associate each point in segment with a surfel in the surfel cloud!
        for (PointT p : c->points) {
            if (!pcl::isFinite(p)) {
                continue;
            }
            p.r = colormap[index % 24][0];
            p.g = colormap[index % 24][1];
            p.b = colormap[index % 24][2];
            colored_cloud->push_back(p);
        }
    }

    cout << "Looking at " << sweep_path.string() << endl;

    point_picker pick_viewer;
    pick_viewer.set_input_cloud(colored_cloud); // A pointer to a cloud
    pick_viewer.show_viewer();

    if (!pick_viewer.picked()) {
        cout << "Did not pick point, exiting..." << endl;
        return 0;
    }

    PointT selected = pick_viewer.get_point();

    cout << "Selected point: " << selected.getVector3fMap().transpose() << endl;
    int selected_index = -1;
    int index = 0;
    for (CloudT::Ptr& cloud : clouds) {
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(cloud);

        vector<int> indices;
        vector<float> distances;
        kdtree.nearestKSearchT(selected, 1, indices, distances);

        if (distances[0] == 0) {
            selected_index = index;
            break;
        }
        ++index;
    }

    if (selected_index == -1) {
        cout << "Could not find any matching cloud!" << endl;
        return 0;
    }

    dynamic_object_retrieval::visualize(clouds[selected_index]);

    boost::filesystem::path annotation_path = sweep_path / "annotation.txt";
    string annotation;
    cout << "Enter annotation for " << sweep_path.string() << ":" << endl;
    cin >> annotation;

    ofstream f(annotation_path.string());
    f << annotation << " " << selected_index << '\n';
    f.close();

    return 0;
}
