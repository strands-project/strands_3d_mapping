#include <dynamic_object_retrieval/summary_iterators.h>
#include <object_3d_benchmark/benchmark_overlap.h>

#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

map<string, string> compute_path_label_correspondence(const boost::filesystem::path& data_path)
{
    const double overlap_threshold = 0.2;

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    map<string, string> path_labels;
    for (const string& xml : folder_xmls) {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(xml);

        boost::filesystem::path xml_path = boost::filesystem::path(xml);
        // get all convex segments for this sweep

        for (int i = 0; i < labels.objectClouds.size(); ++i) {

            dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(xml_path.parent_path());
            dynamic_object_retrieval::sweep_convex_segment_map segment_paths(xml_path.parent_path());

            for (auto tup : dynamic_object_retrieval::zip(segments, segment_paths)) {
                // here it seems like we're gonna have to wrap our own solution by wrapping two octrees (or voxelgrids?)
                CloudT::Ptr c;
                boost::filesystem::path path;
                tie(c, path) = tup;

                double overlap_fraction = benchmark_retrieval::compute_overlap(labels.objectClouds[i], c);

                if (overlap_fraction > overlap_threshold) {
                    path_labels[path.string()] = labels.objectLabels[i];
                    cout << "Found correspondence: " << path.string() << " -> " << labels.objectLabels[i] << endl;
                    break;
                }
            }
        }
    }

    return path_labels;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide path to annotated sweep data..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    map<string, string> path_labels = compute_path_label_correspondence(data_path);

    ofstream out((data_path / "segment_path_labels.cereal").string(), std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(path_labels);
    }
    out.close();

    return 0;
}
