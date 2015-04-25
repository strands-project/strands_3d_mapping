#ifndef DATASET_CONVENIENCE_H
#define DATASET_CONVENIENCE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "object_3d_retrieval/object_retrieval.h"
#include "simple_xml_parser.h"
#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

namespace dataset_convenience {

using namespace std;

typedef typename SimpleSummaryParser::EntityStruct Entities;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = object_retrieval::HistT; // where is this used?
using HistCloudT = pcl::PointCloud<HistT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using index_score = object_retrieval::index_score;

void subsample_cloud(CloudT::Ptr& cloud_in, CloudT::Ptr& cloud_out);
void translate_cloud(CloudT::Ptr& cloud, const Eigen::Vector3f& offset);
void get_rooms(vector<SimpleSummaryParser::EntityStruct>& entities, string summary_xml_path);
void get_room_from_xml(vector<CloudT::Ptr>& clouds,
                       vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& transforms,
                       string& room_id, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics,
                       SimpleSummaryParser::EntityStruct& entity);
void visualize_annotations(map<int, string>& annotated, const string& annotations_path);
void aggregate_features(object_retrieval& obr, const string& aggregate_dir);
void aggregate_pfhrgb_features(object_retrieval& obr);
std::string annotation_for_scan(int i, object_retrieval& obr);


} // namespace dataset_convenience

#endif // DATASET_CONVENIENCE_H
