//
// Created by chris on 23/11/15.
//

#ifndef PROJECT_NBVFINDER_H
#define PROJECT_NBVFINDER_H


#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>


#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "SensorModel.h"
#include "TargetVolume.h"
#include <boost/shared_ptr.hpp>

namespace nbv_planning {

    class NBVFinder {
    public:
        typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
        typedef typename Cloud::Ptr CloudPtr;
        typedef boost::shared_ptr<octomap::OcTree> OcTreeT;


        NBVFinder(const SensorModel &m_sensor_model, double threshold_a=0.999);

        bool set_target_volume(TargetVolume volume);
        const TargetVolume& get_target_volume() {
            return m_target_volume;
        }
        const SensorModel& get_sensor_model() {
            return m_sensor_model;
        }
        bool set_candidate_views(const std::vector<Eigen::Affine3d> &views);

        bool update_current_volume(CloudPtr cloud);
        bool update_current_volume(CloudPtr cloud, const Eigen::Affine3d &sensor_origin);

        bool choose_next_view(unsigned int &selected_view_index);
        float evaluate_view(Eigen::Affine3d &view);

        void save_temporary_map();
    protected:
        void create_octree();
        double get_node_value(const octomap::OcTreeKey& key);

        OcTreeT m_octree;
        SensorModel m_sensor_model;
        TargetVolume m_target_volume;
        std::vector<Eigen::Affine3d> m_candidate_views;
        std::vector<unsigned int> m_available_view_idx;
        double m_a;

    };
}

#endif //PROJECT_NBVFINDER_H
