//
// Created by chris on 30/11/15.
//

#ifndef NBV_PLANNING_RAY_H
#define NBV_PLANNING_RAY_H

#include <Eigen/Eigen>
#include <vector>
#include "nbv_planning/TargetVolume.h"

namespace nbv_planning {
    /**
     * Class for storing 3D rays.
     */
    class Ray {
    public:
        /**
         * @param position The origin of the ray
         * @param direction Unit vector giving ray direction
         */
        Ray(Eigen::Vector3d position, Eigen::Vector3d direction) : position_(position), direction_(direction),
                                                                   length_(-1), clipped_start_(0) {

        };

        /**
         * @param position The origin of the ray
         * @param direction Unit vector giving ray direction
         * @param length The length the ray goes for - usually the maximum sensor length
         */
        Ray(Eigen::Vector3d position, Eigen::Vector3d direction, float length) : position_(position),
                                                                                 direction_(direction),
                                                                                 length_(length),
                                                                                 clipped_start_(0){

        };

        /**
         * Get the ray position
         */
        const Eigen::Vector3d &position() const {
            return position_;
        }

        /**
         * Get the ray direction.
         */
        const Eigen::Vector3d &direction() const {
            return direction_;
        }

        /**
         * Get the ray length
         */
        float length() const {
            return length_;
        }

        /**
         * Get the position the ray would start at if it was clipped to a specific TargetVolume. This is the position that
         * the ray enters the target volume if it has been clipped, 0 otherwise.
         * @return The distance along the ray to the clipped start position.
         */
        float clipped_start() const {
            return clipped_start_;
        }

        /**
         * Clip the ray to a target volume. This reduces the length of the ray and sets clipped_start to the start position.
         * @param TargetVolume the volume to clip this ray to.
         */
        bool clip_to_volume(const TargetVolume &volume) {
            // r.dir is unit direction vector of ray
            double dx = 1.0f / direction_(0);
            double dy = 1.0f / direction_(1);
            double dz = 1.0f / direction_(2);
            // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
            // r.org is origin of ray
            Eigen::Vector3f lb = volume.get_origin()-volume.get_extents();
            Eigen::Vector3f rt = volume.get_origin()+volume.get_extents();
            float t1 = (lb.x() - position_.x()) * dx;
            float t2 = (rt.x() - position_.x()) * dx;
            float t3 = (lb.y() - position_.y()) * dy;
            float t4 = (rt.y() - position_.y()) * dy;
            float t5 = (lb.z() - position_.z()) * dz;
            float t6 = (rt.z() - position_.z()) * dz;

            float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
            float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));
            float t;
            // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behind us
            if (tmax < 0) {
                length_=0;
                return false;
            }
            // if tmin > tmax, ray doesn't intersect AABB
            if (tmin > tmax) {
                length_=0;
                return false;
            }
            //position_ = position_+tmin*direction_;
            clipped_start_ = tmin;
            length_-=tmin;
            if (length_>(tmax-tmin))
                length_=tmax;//-tmin;
            return true;
        };

    private:
        Eigen::Vector3d position_, direction_;
        float length_, clipped_start_;
    };

    typedef typename std::vector<Ray> Rays;
}

#endif //NBV_PLANNING_RAY_H
