#include <iostream>

#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/retrieval_client.h"

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return 0;
    }

    string root_path(argv[1]); //"/home/nbore/Data/Instances/";
    string segment_path = root_path + "supervoxel_segments";

    object_retrieval obr_segments(segment_path);

    retrieval_client::save_pfhrgb_features_for_supervoxels(obr_segments);

    return 0;
}
