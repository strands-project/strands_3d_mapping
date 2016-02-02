//
// Created by chris on 24/11/15.
//

#ifndef NBV_PLANNING_SENSORMODEL_H
#define NBV_PLANNING_SENSORMODEL_H

#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include "nbv_planning/Ray.h"
#include <exception>

namespace  nbv_planning {
    /**
     * Class for storing the intrinsic parameters of a depth camera, and the Rays associated to it.
     */
    class SensorModel {
    public:
        typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> ProjectionMatrix;

        /**
         * @param image_height The height of the image
         * @param image_width  The width of the image
         * @param projection_matrix The 3 x 4 projection matrix for this camera. Row major double Eigen::Matrix.
         *                            [fx'  0  cx' Tx]
         *                        P = [ 0  fy' cy' Ty]
         *                            [ 0   0   1   0]
         * @param max_range The maximum depth sensed
         * @param min_range The minimum depth sensed
         * @param sub_sample Factor to reduce sampling by. This number of rays are skipped in the x and y direction between rays.
         */
        SensorModel(float image_height, float image_width, ProjectionMatrix &projection_matrix, float max_range, float min_range, int sub_sample=140);

        /**
         * Create a sensor model for an Xtion Pro Live camera using standard parameters
         */
        static SensorModel XTION() {
            ProjectionMatrix P;
            return SensorModel(480, 640, P, 4, 0.3);
        }

        float get_image_height() const {
            return m_image_height;
        }

        float get_image_width() const {
            return m_image_width;
        }

        const ProjectionMatrix& get_projection_matrix() const {
            return m_projection_matrix;
        }

        float get_max_range() const {
            return m_max_range;
        }

        float get_min_range() const {
            return m_min_range;
        }

        /**
         * Get the viewing frustrum of this camera, in camera co-ordinates. The frustrum is returned as 6 doubles,
         * in the format used in opengl; left, right, top, bottom, near, far
         */
        std::vector<double> get_frustum() const;

        /**
         * Convert a given frustrum into  8 coordinates, the first 4 are the front plane and the last 4 are the back plane.
         * @param frustrum The frustrum expressed as a vector of 6 scalars: [left, right, top, bottom, near, far]
         */
        static std::vector<Eigen::Vector3d> frustum_to_vertices(const std::vector<double> &frustum);

        /** Get the frustrum associated with this sensor model as 8 vertices.
         * @param max_range If > 0, then clip the returned frustrum vertices to  that range.
         */
        std::vector<Eigen::Vector3d> get_frustum_vertices(double max_range=0) const;

        /**
         * Return the rays that this sensor model contains, transformed to a given view frame and clipped to a given target
         * volume.
         * @param view Eigen transform giving the sensor origin
         * @param target_volume The volume to clip the rays too before returning
         * @return std::vector<Ray>
         */
        Rays get_rays(const Eigen::Affine3d &view, const TargetVolume &target_volume) const;

        friend std::ostream& operator<<(std::ostream& os, const SensorModel& sm);

    private:
        SensorModel() {} ;

        float m_image_height, m_image_width;
        float m_max_range;
        float m_min_range;
        int m_sub_sample;
        Rays m_rays;
        //Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        ProjectionMatrix m_projection_matrix;

        class sensor_model_exception : public std::exception {
            virtual const char* what() const throw()  {
                return "Invalid minimum range for sensor model. Must be greater than 0.";
            }
        } m_minimum_range_error;
    };

}
#endif //NBV_PLANNING_SENSORMODEL_H
