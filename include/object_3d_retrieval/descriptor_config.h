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
    static const char* const vocabulary_file;
    static const char* const grouped_vocabulary_file;
    static const char* const grouped_associations_file;

    // segment feature files:
    static const char* const feature_segment_file;
    static const char* const keypoint_segment_file;

    // segment split feature files:
    static const char* const split_feature_stem;
    static const char* const split_keypoints_stem;

    // cached grouped vocabulary vectors, scan:
    static const char* const scan_vocabulary_vectors;
    static const char* const scan_vocabulary_index_vectors;
    static const char* const scan_vocabulary_norms;
    static const char* const scan_split_centers;
};

#endif // DESCRIPTOR_CONFIG_H
