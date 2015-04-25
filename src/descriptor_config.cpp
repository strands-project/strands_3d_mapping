#include "object_3d_retrieval/descriptor_config.h"

// PFHRGB:

// vocabulary files:
const std::string descriptor_config::vocabulary_file = "vocabulary_pfhrgb_3.cereal";
const std::string descriptor_config::grouped_vocabulary_file = "vocabulary_grouped_3.cereal";
const std::string descriptor_config::grouped_associations_file = "group_subgroup_3.cereal";

// segment feature files:
const std::string descriptor_config::feature_segment_file = "pfhrgb_cloud.pcd";
const std::string descriptor_config::keypoint_segment_file = "pfhrgb_points_file.pcd";

// segment split feature files:
const std::string descriptor_config::split_feature_stem = "split_features";
const std::string descriptor_config::split_keypoints_stem = "split_points";

// cached grouped vocabulary vectors, scan:
const std::string descriptor_config::scan_vocabulary_vectors = "grouped_vocabulary_vectors_1.cereal";
const std::string descriptor_config::scan_vocabulary_norms = "grouped_vocabulary_norms_1.cereal";
const std::string descriptor_config::scan_split_centers = "split_centers_1.pcd";

// SHOT:
/*
// vocabulary files:
const std::string descriptor_config::vocabulary_file = "vocabulary_shot.cereal";
const std::string descriptor_config::grouped_vocabulary_file = "vocabulary_grouped_shot.cereal";
const std::string descriptor_config::grouped_associations_file = "group_subgroup_1.cereal";

// segment feature files:
const std::string descriptor_config::feature_segment_file = "shot_cloud.pcd";
const std::string descriptor_config::keypoint_segment_file = "shot_points_file.pcd";

// segment split feature files:
const std::string descriptor_config::split_feature_stem = "split_shot_features";
const std::string descriptor_config::split_keypoints_stem = "split_shot_points";

// cached grouped vocabulary vectors, scan:
const std::string descriptor_config::scan_vocabulary_vectors = "grouped_vocabulary_vectors_shot.cereal";
const std::string descriptor_config::scan_vocabulary_norms = "grouped_vocabulary_norms_shot.cereal";
const std::string descriptor_config::scan_split_centers = "split_centers_shot.pcd";
*/
