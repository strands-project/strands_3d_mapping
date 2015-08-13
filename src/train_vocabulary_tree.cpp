#include <iostream>

#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/retrieval_client.h"

#include <eigen_cereal/eigen_cereal.h>

using namespace std;

void save_noise_data_sizes(object_retrieval& obr_scans_noise, object_retrieval& obr_segments_noise)
{
    int noise_scans_size = 3526;
    int noise_segments_size = 63136;

    retrieval_client::write_noise_segment_size(noise_scans_size, obr_scans_noise);
    retrieval_client::write_noise_segment_size(noise_segments_size, obr_segments_noise);
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        cout << " Please supply the annotated data path, the noise data path and the option: \n"
             << " 1. Train convex segment vocabulary tree on noise data\n"
             << " 2. Add annotated data to convex segment vocabulary tree\n"
             << " 3. Train subsegment vocabulary tree on noise data\n"
             << " 4. Add annotated data to subsegment vocabulary tree" << endl;
        return 0;
    }

    string annotated_root_path(argv[1]); // "/home/nbore/Data/Instances/";
    string annotated_segment_path = annotated_root_path + "supervoxel_segments";

    string noise_root_path(argv[2]); // "/home/nbore/Data/semantic_map/";
    string noise_scan_path = noise_root_path + "scan_segments";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    int option = atoi(argv[3]);

    object_retrieval obr_segments_annotated(annotated_segment_path);
    object_retrieval obr_segments_noise(noise_segment_path);
    object_retrieval obr_scans_noise(noise_scan_path);
    obr_scans_noise.segment_name = "scan";

    int noise_segments_size;
    int noise_scans_size;

    switch (option) {
    case 1:
        //obr_segments_noise.train_vocabulary_incremental(4000, false);
        save_noise_data_sizes(obr_scans_noise, obr_segments_noise);
        break;
    case 2:
        noise_segments_size = retrieval_client::read_noise_segment_size(obr_segments_noise);
        obr_segments_noise.add_others_to_vocabulary(10000, obr_segments_annotated.segment_path, noise_segments_size);
        break;
    case 3:
        obr_segments_noise.train_grouped_vocabulary(12000, false);
        save_noise_data_sizes(obr_scans_noise, obr_segments_noise);
        break;
    case 4:
        noise_scans_size = retrieval_client::read_noise_segment_size(obr_scans_noise);
        obr_segments_noise.add_others_to_grouped_vocabulary(10000, obr_segments_annotated, noise_scans_size);
        break;
    default:
        cout << "The option provided is not valid..." << endl;
    }

    return 0;
}
