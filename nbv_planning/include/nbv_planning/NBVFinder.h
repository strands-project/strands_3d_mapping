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
    /**
     * The NBVFinder class is the main base class for the view selector, given a TargetVolume and SensorModel it builds
     * an octomap of the target volume and selected views from a given set of candidates.
     */
    class NBVFinder {
    public:
        typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
        typedef typename Cloud::Ptr CloudPtr;
        typedef boost::shared_ptr<octomap::OcTree> OcTreeT;

        /**
         * Constructor requires a valid SensorModel to use for evaluating candidate viewing positions, and a specific
         * threshold_a.
         * @param sensor_model The SensorModel object containing the camera parameters.
         * @param threshold_a  The threshold set in equation 10. This controls how far into unknown space rays effect
         */
        NBVFinder(const SensorModel &sensor_model, double threshold_a=0.999);

        /**
         * Set the target volume in which the view planner is trying to maximise knowledge of. This initialises the internal
         * octomap.
         */
        bool set_target_volume(TargetVolume volume);

        /**
         * Returns the target volume used by this NBV finder.
         */
        const TargetVolume& get_target_volume() const {
            return m_target_volume;
        }

        /**
         * Returns the sensor model used by this NBV finder.
         */
        const SensorModel& get_sensor_model() const {
            return m_sensor_model;
        }

        /**
         * Set the candidate views for use by this view finder. Candidate views are given as the sensor origin,
         * where the sensor is defined by the SensorModel assigned to the view finder at construction time. Candidate
         * views are Eigen::Affine3d objects, i.e Eigen::Transform s, and are expressed in the frame of the map.
         * @param views  A vector of candidate views from which the view finder can select.
         */
        bool set_candidate_views(const std::vector<Eigen::Affine3d> &views);

        /**
         * Calculate the volume that is actually viewed by the candidate views. This will be different to that which
         * is set as the TargetVolume.
         * @param min The lowest (x,y,z) co-ordinate of an axis aligned bounding box covering the viewed space
         * @param max The maximum (x,y,z) co-ordinate of an axis aligned bounding box covering the viewed space
         */
        bool calculate_viewed_volume(Eigen::Vector3d &min, Eigen::Vector3d &max) const;

        /**
         * Update the current occupancy knowledge of the target volume.
         * @param cloud A pointcloud already transformed to the map frame.
         */
        bool update_current_volume(CloudPtr cloud);

        /**
         * Update the current occupancy knowledge of the target volume. Only the part of the supplied cloud inside
         * the volume covered by all candidate views is used to update the internal map.
         * @param cloud A pointcloud to update with. This is non const and might get changed by the function.
         * @param sensor_origin The position of the sensor when the pointcloud was captured.
         */
        bool update_current_volume(CloudPtr cloud, const Eigen::Affine3d &sensor_origin);

        /**
         * Select the next best view from the candidate views.
         * @param selected_view_index  The index of the view that was selected.
         * @param disable_view   If true then the selected view will be removed from candidates, so as to not be selected again
         * @return false if no view could be chosen (e.g none left to select from)
         */
        bool choose_next_view(bool disable_view, unsigned int &selected_view_index, double &view_score);

        /**
         * Evaluate a view, returning the expected information of that view.
         * @param view An Eigen::Affine3d representation of the sensor position in the map frame.
         */
        float evaluate_view(Eigen::Affine3d &view) const;

        /**
         * Save a temporary copy of the internal octomap. Stores to /tmp/volume.bt.
         */
        void save_temporary_map();

        /**
         * Return the number of cells in the target volume that have not been observed yet.
         */
        int count_unobserved_cells() const;
    protected:
        void create_octree();
        double get_node_value(const octomap::OcTreeKey& key) const;

        OcTreeT m_octree;
        SensorModel m_sensor_model;
        TargetVolume m_target_volume;
        std::vector<Eigen::Affine3d> m_candidate_views;
        std::vector<unsigned int> m_available_view_idx;
        double m_a;

    };
}

#endif //PROJECT_NBVFINDER_H
