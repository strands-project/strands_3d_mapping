#include "k_means_tree/k_means_tree.h"
#include "vocabulary_tree/vocabulary_tree.h"

#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define N 250

using namespace std;

using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

int main(int argc, char** argv)
{
    if (argc < 3) {
        return -1;
    }
    string database_path(argv[1]);
    string database_indices_path(argv[2]);
    string query_path(argv[3]);

    // the data base contains all of the vectors we are matching to
    HistCloudT::Ptr database_cloud(new HistCloudT);
    if (pcl::io::loadPCDFile<HistT> (database_path, *database_cloud) == -1) //* load the file
    {
        cout << "Could not read file " << database_path << endl;
        return (-1);
    }
    else {
        cout << "Read file " << database_path << endl;
    }

    // the query cloud is the cloud we are matching to the data base of vectors
    HistCloudT::Ptr query_cloud(new HistCloudT);
    if (pcl::io::loadPCDFile<HistT> (query_path, *query_cloud) == -1) //* load the file
    {
        cout << "Could not read file " << query_path << endl;
        return (-1);
    }
    else {
        cout << "Read file " << query_path << endl;
    }

    // entry i in this vector determines what cluster entry y in database_cloud belongs to
    vector<int> database_indices;
    ifstream in(database_indices_path, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(database_indices);
    }
    in.close();

    vocabulary_tree<HistT, 8> vt;
    vt.set_input_cloud(database_cloud, database_indices);
    vt.add_points_from_input_cloud();

    int nbr_results = 10; // the top results that we get back

    using result_type = vocabulary_tree<HistT, 8>::result_type;
    vector<result_type> scores;
    // both of these are interesting to try out
    vt.top_combined_similarities(scores, query_cloud, nbr_results);
    //vt.top_similarities(scores, query_cloud, nbr_results);

    for (result_type score : scores) {
        cout << "We got index " << score.index << " with score " << score.score << endl;
    }

    return 0;
}
