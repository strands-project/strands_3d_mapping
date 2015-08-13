#include <iostream>

#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/retrieval_client.h"

#include <cereal/archives/json.hpp>
#include <eigen_cereal/eigen_cereal.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << " Please supply the annotated data path, the noise data path and the option: \n" << endl;
        return 0;
    }

    string annotated_root_path(argv[1]); // "/home/nbore/Data/Instances/";
    string annotated_scan_path = annotated_root_path + "scan_segments";

    string noise_root_path(argv[2]); // "/home/nbore/Data/semantic_map/";
    string noise_scan_path = noise_root_path + "scans";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    object_retrieval obr_segments_noise(noise_segment_path);
    object_retrieval obr_scans_noise(noise_scan_path);
    obr_scans_noise.segment_name = "scan";

    object_retrieval obr_scans_annotated(annotated_scan_path);
    obr_scans_annotated.segment_name = "scan";

    if (obr_segments_noise.gvt.empty()) {
        obr_segments_noise.read_vocabulary(obr_segments_noise.gvt);
    }
    obr_segments_noise.gvt.set_min_match_depth(3); // the depth has to match the one in the querying file!
    obr_segments_noise.gvt.compute_normalizing_constants();

    retrieval_client::save_oversegmented_grouped_vocabulary_index_vectors(obr_scans_annotated, obr_segments_noise);
    retrieval_client::save_oversegmented_grouped_vocabulary_index_vectors(obr_scans_noise, obr_segments_noise);

    return 0;
}
