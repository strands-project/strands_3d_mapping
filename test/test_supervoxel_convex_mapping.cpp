#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

map<size_t, size_t> load_convex_segment_indices(const boost::filesystem::path& sweep_path)
{
    map<size_t, size_t> convex_segment_indices;
    std::ifstream in((sweep_path / "subsegments" / "convex_segment_indices.cereal").string(), ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(convex_segment_indices);
    }
    in.close();
    return convex_segment_indices;
}

int main(int argc, char** argv)
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

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::subsegment_cloud_map supervoxel_clouds(data_path);
    dynamic_object_retrieval::subsegment_index_map supervoxel_indices(data_path);
    dynamic_object_retrieval::subsegment_sweep_path_map sweep_paths(data_path);

    map<size_t, size_t> convex_segment_indices;
    CloudT::Ptr visualization_cloud(new CloudT);
    for (auto tup : dynamic_object_retrieval::zip(supervoxel_clouds, supervoxel_indices, sweep_paths)) {
        CloudT::Ptr cloud;
        size_t ind;
        boost::filesystem::path sweep_path;
        tie(cloud, ind, sweep_path) = tup;

        //cout << ind << endl;
        if (ind == 0) {
            if (!visualization_cloud->empty()) {
                dynamic_object_retrieval::visualize(visualization_cloud);
            }
            visualization_cloud->clear();
            convex_segment_indices.clear();
            convex_segment_indices = load_convex_segment_indices(sweep_path);
            /*
            for (const pair<size_t, size_t>& p : convex_segment_indices) {
                cout << p.first << ", " << p.second << endl;
            }
            exit(0);
            */
        }

        int convex_ind = convex_segment_indices[ind]; // this actually creates a new entry, initialized to 0
        //cout << "===" << endl;
        //cout << convex_ind << endl;
        //cout << convex_segment_indices.size() << endl;
        for (PointT p : cloud->points) {
            p.r = colormap[convex_ind%24][0];
            p.g = colormap[convex_ind%24][1];
            p.b = colormap[convex_ind%24][2];
            visualization_cloud->push_back(p);
        }

    }


    return 0;
}
