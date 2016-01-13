#include <vlad/vlad_representation.h>
#include <vlad/bow_representation.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide data set path and query index..." << endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    int query_index(atoi(argv[2]));

    vlad_representation::vlad_repr repr;
    repr.dimension = 250; // pfh feature dimension
    repr.numCenters = nbr_centers; // seems good defaults

    vlad_representation::build_vlad_representation(data_path, 100000, repr);
    vlad_representation::encode_vlad_representation(data_path, repr);

    vl_kmeans_delete(repr.kmeans);

    //vlad_representation::encode_vlad_point();
    //vlad_representation::query_vlad_representation(data_path, query_index);

    //bow_representation::encode_bow_representation(data_path);
    //bow_representation::query_bow_representation(data_path, query_index);

    return 0;
}
