//
// Created by chris on 24/11/15.
//

#include <vector>
#include <iostream>
#include "nbv_planning/SensorModel.h"

/**
 *  Return the frustum of the sensor, clipped to the max distance specified.
 *
 *  This returns 6 points, likle opengl; left, right, top, bottom, near, far
 */
std::vector<double> nbv_planning::SensorModel::get_frustum() const{
//    Eigen::Vector3f ;
    // Convert the projection matrix into the viewing angles
    std::vector<double> frustum(6);
//#     [fx'  0  cx' Tx]
//# P = [ 0  fy' cy' Ty]
//#     [ 0   0   1   0]
//# By convention, this ma
    double left = m_min_range * (m_projection_matrix(0, 2) - m_image_width) / m_projection_matrix(0, 0);
    double right = m_min_range * (m_image_width - m_projection_matrix(0, 2)) / m_projection_matrix(0, 0);
    double top = m_min_range * (m_projection_matrix(1, 2) - m_image_height) / m_projection_matrix(1, 1);
    double bottom = m_min_range * (m_image_height - m_projection_matrix(1, 2)) / m_projection_matrix(1, 1);
    frustum[0] = left;
    frustum[1] = right;
    frustum[2] = top;
    frustum[3] = bottom;
    frustum[4] = m_min_range;
    frustum[5] = m_max_range;
    return frustum;
}

std::vector<Eigen::Vector3d> nbv_planning::SensorModel::frustum_to_vertices(const std::vector<double> &frustum) {
    std::vector<Eigen::Vector3d> vertices(8);
    // Front plane is easy
    vertices[0] = Eigen::Vector3d(frustum[0], frustum[2], frustum[4]); // top, left, near
    vertices[1] = Eigen::Vector3d(frustum[0], frustum[3], frustum[4]); // top, right, near
    vertices[2] = Eigen::Vector3d(frustum[1], frustum[2], frustum[4]); // bottom, left, near
    vertices[3] = Eigen::Vector3d(frustum[1], frustum[3], frustum[4]); // bottom, right, near
    // The back plane
    double diff = frustum[5] - frustum[4];
    vertices[4] = Eigen::Vector3d(frustum[0] + (diff * frustum[0]) / frustum[4],
                                 frustum[2] + (diff * frustum[2]) / frustum[4],
                                 frustum[5]); // top, left, far
    vertices[5] = Eigen::Vector3d(frustum[0] + (diff * frustum[0]) / frustum[4],
                                 frustum[3] + (diff * frustum[3]) / frustum[4],
                                 frustum[5]); // top, right, far
    vertices[6] = Eigen::Vector3d(frustum[1] + (diff * frustum[1]) / frustum[4],
                                 frustum[2] + (diff * frustum[2]) / frustum[4],
                                 frustum[5]); // bottom, left, far
    vertices[7] = Eigen::Vector3d(frustum[1] + (diff * frustum[1]) / frustum[4],
                                  frustum[3] + (diff * frustum[3]) / frustum[4], frustum[5]); // bottom, right, far

    return vertices;

}

std::vector<Eigen::Vector3d> nbv_planning::SensorModel::get_frustum_vertices(double max_range) const{
    std::vector<double> frustum = get_frustum();
    //std::cout << "Frustum: " << frustum.size() << std::endl;
    // Cap the max range...
    if (max_range > 0)
        frustum[5] = max_range;
    return nbv_planning::SensorModel::frustum_to_vertices(frustum);
}

nbv_planning::Rays nbv_planning::SensorModel::get_rays(const Eigen::Affine3d &view,
                                                       const TargetVolume &target_volume) const {
    Rays rays;
    rays.reserve(m_rays.size());
//    for (std::vector<Ray>::iterator ray = m_rays.begin();ray<m_rays.end();ray++) {
    for (unsigned int i = 0; i < m_rays.size(); i++) {
        Ray transformed_ray(view * m_rays[i].position(), view.linear() * m_rays[i].direction(), m_rays[i].length());
        transformed_ray.clip_to_volume(target_volume);
        rays.push_back(transformed_ray);
    }
    return rays;
}

nbv_planning::SensorModel::SensorModel(float image_height, float image_width, ProjectionMatrix &projection_matrix,
                                       float max_range, float min_range, int sub_sample)
        : m_image_height(image_height), m_image_width(image_width), m_projection_matrix(projection_matrix),
          m_max_range(max_range),
          m_min_range(min_range),
          m_sub_sample(sub_sample){
    if (m_min_range <= 0)
        throw m_minimum_range_error;
    m_rays.reserve((unsigned long) (m_image_width * m_image_height));
    unsigned int i = 0;
    float ray_length = m_max_range - m_min_range;
    for (unsigned int x = 0; x < m_image_width; x+=sub_sample) {
        for (unsigned y = 0; y < m_image_height; y+=sub_sample) {
            double u = x - m_projection_matrix(0, 2);
            double v = y - m_projection_matrix(1, 2);
            double f = m_projection_matrix(0, 0); // Why only use the x focal length?
            Eigen::Vector3d direction(u, v, f);
            direction.normalize();
            Eigen::Vector3d start = direction * m_min_range;
            Ray ray(start, direction, ray_length);
            m_rays.push_back(ray);
        }

    }

}

namespace nbv_planning {
    std::ostream& operator<<(std::ostream &os, const SensorModel &sm) {
        os << "Sensor Model: \n  Projection matrix:\n" << sm.m_projection_matrix << "\n";
        os << "Image size: " << sm.m_image_width << " X " << sm.m_image_height << "\n";
        os << "Range: " << sm.m_min_range << " - " << sm.m_max_range << std::endl;
        os << "Subsample: " << sm.m_sub_sample;
        return os;
    }
}