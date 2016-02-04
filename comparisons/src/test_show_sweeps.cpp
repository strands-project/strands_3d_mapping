#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2) {
        std::cout << "Please provide the path to the data..." << std::endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};
    std::vector<std::string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
    int counter = 0;
    for (const std::string& xml : folder_xmls) {
        if (counter > 400) {
            break;
        }
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(xml);
        for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectImages, labels.objectMasks, labels.objectScanIndices)) {
            CloudT::Ptr object_cloud;
            std::string query_label;
            cv::Mat query_image;
            cv::Mat query_mask;
            size_t scan_index;
            tie(object_cloud, query_label, query_image, query_mask, scan_index) = tup;
            bool found = false;
            for (const std::string& is_check : objects_to_check) {
                if (query_label.compare(0, is_check.size(), is_check) == 0) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                continue;
            }
            cout << xml << endl;
            ++counter;
        }
    }

    return 0;
}



