#ifndef DESCRIPTOR_CONFIG_H
#define DESCRIPTOR_CONFIG_H

#include <string>

// PFHRGB
struct descriptor_config {
    // descriptor size:
    // PFHRGB:
    static const int N = 250;
    // SHOT:
    //static const int N = 1344;

    // vocabulary files:
    const static std::string vocabulary_file;
    const static std::string grouped_vocabulary_file;
    const static std::string grouped_associations_file;

    // segment feature files:
    const static std::string feature_segment_file;
    const static std::string keypoint_segment_file;

    // segment split feature files:
    const static std::string split_feature_stem;
    const static std::string split_keypoints_stem;

    // cached grouped vocabulary vectors, scan:
    const static std::string scan_vocabulary_vectors;
    const static std::string scan_vocabulary_index_vectors;
    const static std::string scan_vocabulary_norms;
    const static std::string scan_split_centers;
};

#endif // DESCRIPTOR_CONFIG_H
