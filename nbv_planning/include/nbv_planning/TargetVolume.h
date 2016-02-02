//
// Created by chris on 24/11/15.
//

#ifndef NBV_PLANNING_TARGETVOLUME_H
#define NBV_PLANNING_TARGETVOLUME_H


#include <Eigen/Eigen>
#include <vector>
#include <iostream>
namespace nbv_planning {
    /**
     * Class for storing the target volume for selecting the best views of.
     */
    class TargetVolume {

    public:
        /**
         * Create a basic target volume centered at 0,0,0 with size 5m^3 and 1cm per voxel.
         */
        TargetVolume() : m_origin(0, 0, 0), m_extents(2.5, 2.5, 2.5), m_scale(0.01) {
        }

        /**
         * @param scale How many meteres per voxel in the internal octomap representation
         * @param origin The center of the volume, usually the object centroid
         * @param extents The size/2 of the volume.
         */
        TargetVolume(float scale, const Eigen::Vector3f &origin, const Eigen::Vector3f &extents) :
                m_scale(scale), m_origin(origin), m_extents(extents) {

        }

        float get_x_origin() const {
            return m_origin[0];
        }

        float get_y_origin() const {
            return m_origin[1];
        }

        float get_z_origin() const {
            return m_origin[2];
        }

        float get_x_size() const {
            return m_extents[0] * 2;
        }

        float get_y_size() const {
            return m_extents[1] * 2;
        }

        float get_z_size() const {
            return m_extents[2] * 2;
        }

        float get_scale() const {
            return m_scale;
        }

        const Eigen::Vector3f &get_origin() const {
            return m_origin;
        }

        const Eigen::Vector3f &get_extents() const {
            return m_extents;
        }

        friend std::ostream & operator<< (std::ostream& os, const TargetVolume& volume);
    private:
        float m_scale;
        Eigen::Vector3f m_origin;
        Eigen::Vector3f m_extents;
    };
}

#endif //NBV_PLANNING_TARGETVOLUME_H
