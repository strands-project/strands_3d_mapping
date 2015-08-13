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
    string scan_path = root_path + "scan_segments";

    object_retrieval obr_scans(scan_path);
    obr_scans.segment_name = "scan";

    retrieval_client::save_sift_features(obr_scans);

    return 0;
}

