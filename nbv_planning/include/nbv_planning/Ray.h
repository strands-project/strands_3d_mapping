//
// Created by chris on 30/11/15.
//

#ifndef NBV_PLANNING_RAY_H
#define NBV_PLANNING_RAY_H

#include <Eigen/Eigen>
#include <vector>
#include "nbv_planning/TargetVolume.h"

namespace nbv_planning {
    class Ray {
    public:
        Ray(Eigen::Vector3d position, Eigen::Vector3d direction) : position_(position), direction_(direction),
                                                                   length_(-1), clipped_start_(0) {

        };

        Ray(Eigen::Vector3d position, Eigen::Vector3d direction, float length) : position_(position),
                                                                                 direction_(direction),
                                                                                 length_(length),
                                                                                 clipped_start_(0){

        };

        const Eigen::Vector3d &position() const {
            return position_;
        }

        const Eigen::Vector3d &direction() const {
            return direction_;
        }

        float length() const {
            return length_;
        }

        float clipped_start() const {
            return clipped_start_;
        }

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
