#include <iostream>

#include "object_3d_retrieval/object_retrieval.h"

#include <cereal/archives/json.hpp>
#include <eigen_cereal/eigen_cereal.h>

using namespace std;
using namespace retrieval_client;

int main(int argc, char** argv)
{
    if (argc < 4) {
        cout << " Please supply the annotated data path, the noise data path and the option: \n"
             << " 1. Train convex segment vocabulary tree on noise data\n"
             << " 2. Add annotated data to convex segment vocabulary tree\n"
             << " 3. Train subsegment vocabulary tree on noise data\n"
             << " 4. Add annotated data to subsegment vocabulary tree" << endl;
        return;
    }

    string annotated_root_path(argv[1]); // "/home/nbore/Data/Instances/";
    string annotated_segment_path = annotated_root_path + "supervoxel_segments";

    string noise_root_path(argv[2]); // "/home/nbore/Data/semantic_map/";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    int option = atoi(argv[3]);

    object_retrieval obr_segments_annotated(annotated_segment_path);
    object_retrieval obr_segments_noise(noise_segment_path);

    // TODO: read and write these to a couple of json files
    int noise_scans_size = 3526;
    int noise_segments_size = 63136;

    switch (option) {
        case 1:
            obr_segments_noise.train_vocabulary_incremental(4000, false); // 12000
            break;
        case 2:
            obr_segments_noise.add_others_to_vocabulary(10000, obr_segments_annotated.segment_path, noise_segments_size);
            break;
        case 3:
            obr_segments_noise.train_grouped_vocabulary(12000, false);
            break;
        case 4:
            obr_segments_noise.add_others_to_grouped_vocabulary(10000, obr_segments_annotated, noise_scans_size);
            break;
        default:
            cout << "The option provded is not valid..." << endl;
    }

    return 0;
}
