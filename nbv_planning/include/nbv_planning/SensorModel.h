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
    class SensorModel {
    public:
        typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> ProjectionMatrix;

        SensorModel(float image_height, float image_width, ProjectionMatrix &projection_matrix, float max_range, float min_range, int sub_sample=140);

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

        std::vector<double> get_frustum();
        /**
         * Returns 8 coordinates for the frustum; Frist 4 are the front plane, last 4 are the back plane
         */
        static std::vector<Eigen::Vector3f> frustum_to_vertices(const std::vector<double> &frustum);
        std::vector<Eigen::Vector3f> get_frustum_vertices(double max_range=0);

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
