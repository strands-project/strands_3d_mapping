#include "dynamic_object_retrieval/dynamic_retrieval.h"
#include "dynamic_object_retrieval/visualize.h"

#include "vocabulary_tree/vocabulary_tree.h"

#include <cereal/archives/binary.hpp>
#include <pcl/io/pcd_io.h>

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: ./dynamic_query_vocabulary /path/to/vocabulary /path/to/cloud" << endl;
        return 0;
    }

    boost::filesystem::path vocabulary_path(argv[1]);
    boost::filesystem::path cloud_path(argv[2]);

    HistCloudT::Ptr features(new HistCloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *features);

    vocabulary_tree<HistT, 8> vt;
    vector<boost::filesystem::path> retrieved_paths = dynamic_object_retrieval::query_vocabulary(features, 10, vt, vocabulary_path);

    for (boost::filesystem::path path : retrieved_paths) {
        cout << "Path: " << path.string() << endl;
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(path.string(), *cloud);
        dynamic_object_retrieval::visualize(cloud);
    }

    return 0;
}
