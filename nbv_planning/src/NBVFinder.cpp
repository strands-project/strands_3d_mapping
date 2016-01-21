//
// Created by chris on 23/11/15.
//

#include "nbv_planning/NBVFinder.h"
#include <octomap_ros/conversions.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

/**
* @brief Conversion from a PCL pointcloud to octomap::Pointcloud, used internally in OctoMap
*
* @param pclCloud
* @param octomapCloud
*/
namespace octomap {
    template<class PointT>
    static inline void pointcloudPCLToOctomap(const pcl::PointCloud<PointT> &pclCloud, Pointcloud &octomapCloud) {
        octomapCloud.reserve(pclCloud.points.size());

        typename
        pcl::PointCloud<PointT>::const_iterator it;
        for (it = pclCloud.begin(); it != pclCloud.end(); ++it) {
            // Check if the point is invalid
            if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
                octomapCloud.push_back(it->x, it->y, it->z);
        }
    }
}


namespace nbv_planning {

    NBVFinder::NBVFinder(const nbv_planning::SensorModel &m_sensor_model,
                         double threshold_a) : m_sensor_model(m_sensor_model), m_a(threshold_a) {
        // Initialise the candidate views vector
        m_candidate_views.clear();
        create_octree();
    }

    void NBVFinder::create_octree() {
        m_octree = OcTreeT(new octomap::OcTree(m_target_volume.get_scale()));
        m_octree->setClampingThresMin(0.12);
        m_octree->setClampingThresMax(0.97);
        m_octree->setProbHit(0.7);
        m_octree->setProbMiss(0.4);
        m_octree->setOccupancyThres(0.5);
//        Eigen::Vector3f max = m_target_volume.get_origin() + m_target_volume.get_extents();
//        octomap::point3d max_o = octomap::point3d(max.x(), max.y(), max.z());
//        Eigen::Vector3f min = m_target_volume.get_origin() - m_target_volume.get_extents();
//        octomap::point3d min_o = octomap::point3d(min.x(), min.y(), min.z());
//        std::cout << "The bounds of the octo are set...:" << max << "  +/- " << min << std::endl;
//        m_octree->setBBXMax(max_o);
//        m_octree->setBBXMin(min_o);
        m_octree->clear();
    }

    bool NBVFinder::set_target_volume(TargetVolume volume) {
        m_target_volume = volume;
        create_octree();
        return false;
    }

    bool NBVFinder::update_current_volume(NBVFinder::CloudPtr cloud, const Eigen::Affine3d &sensor_origin) {
        // Filter the pointcloud so that it only has stuff inside the target volume.
        pcl::transformPointCloud(*cloud, *cloud, sensor_origin);
//        ROS_INFO("Filtering out non-volumen cloud");
//        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
//        pass_x.setFilterFieldName("x");
//        pass_x.setFilterLimits(m_target_volume.get_x_origin() - 0.5f * m_target_volume.get_x_size(),
//                               m_target_volume.get_x_origin() + 0.5f * m_target_volume.get_x_size());
//        pass_x.setInputCloud(cloud);
//        pass_x.filter(*cloud);
//
//        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
//        pass_y.setFilterFieldName("y");
//        pass_y.setFilterLimits(m_target_volume.get_y_origin() - 0.5f * m_target_volume.get_y_size(),
//                               m_target_volume.get_y_origin() + 0.5f * m_target_volume.get_y_size());
//        pass_y.setInputCloud(cloud);
//        pass_y.filter(*cloud);
//
//        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
//        pass_z.setFilterFieldName("z");
//        pass_z.setFilterLimits(m_target_volume.get_z_origin() - 0.5f * m_target_volume.get_z_size(),
//                               m_target_volume.get_z_origin() + 0.5f * m_target_volume.get_z_size());
//        pass_z.setInputCloud(cloud);
//        pass_z.filter(*cloud);

        octomap::Pointcloud octomap_cloud;
        octomap::pointcloudPCLToOctomap(*cloud, octomap_cloud);
        std::cout << "Number of points in octomap cloud: " << octomap_cloud.size() << "\n";
        octomap::point3d origin(sensor_origin.translation().x(), sensor_origin.translation().y(),sensor_origin.translation().z());


//        Eigen::Quaterniond q = (Eigen::Quaterniond) sensor_origin.linear();
//        //TODO: The following might be necessary?
////        m.orientation.x = q.x();
////        m.orientation.y = q.y();
////        m.orientation.z = q.z();
////        m.orientation.w = q.w();
////        if (m.orientation.w < 0) {
////            m.orientation.x *= -1;
////            m.orientation.y *= -1;
////            m.orientation.z *= -1;
////            m.orientation.w *= -1;
////        }
//
//        octomap::pose6d frame(octomath::Vector3(sensor_origin.translation()[0],
//                                                sensor_origin.translation()[1],
//                                                sensor_origin.translation()[2]),
//                              octomath::Quaternion(q.w(), q.x(), q.y(), q.z()));

//        std::cout << frame;

        m_octree->insertPointCloud(octomap_cloud, origin, -1,false,true);
//        m_octree->toMaxLikelihood();
        std::cout << "Octree now has " << m_octree->calcNumNodes() << " nodes.\n";
        return false;
    }

    bool NBVFinder::set_candidate_views(const std::vector<Eigen::Affine3d> &views) {
        m_candidate_views = views;
        m_available_view_idx.reserve(m_candidate_views.size());
        for (unsigned i=0;i<m_candidate_views.size();i++)
            m_available_view_idx.push_back(i);
        return true;
    }

    float NBVFinder::evaluate_view(Eigen::Affine3d &view) {
        // Find the entry of the view rays into the view volume
        // Find the exit
        // Get the cells along the way
        double view_gain=0;
        Rays rays = m_sensor_model.get_rays(view, m_target_volume);

        float max_range = m_sensor_model.get_max_range();
        for (Rays::iterator ray = rays.begin(); ray < rays.end(); ray++) {
            Eigen::Vector3f volume_origin = m_target_volume.get_origin();
            octomap::point3d origin(ray->position()(0), ray->position()(1), ray->position()(2)),
                    start, end;
            start = origin +
                  octomap::point3d(ray->direction()(0), ray->direction()(1), ray->direction()(2)) * ray->clipped_start();
            end = origin +
                  octomap::point3d(ray->direction()(0), ray->direction()(1), ray->direction()(2)) * ray->length();

            octomap::KeyRay ray_inside_volume, full_ray;
            m_octree->computeRayKeys(start, end, ray_inside_volume);
            m_octree->computeRayKeys(origin, end, full_ray);

            // iterate over the voxels and set them good?
            if (ray_inside_volume.size() <1)
                continue; // this ray does not intersect the volume of interest

            bool counting=false; // only count the information gain for cells that are inside the target volume
            double prev_p_x = 1;
            double prev_p_o = 0;
            int number_of_cells_passed =0;

            for (octomap::KeyRay::iterator cell = full_ray.begin(); cell < full_ray.end(); cell++) {
                number_of_cells_passed+=1;

                if (*cell == *(ray_inside_volume.begin())) {
                    // We got to the start of the part of the ray that is inside the volume
                    counting=true;
                }
                double prev_cell_value = get_node_value(*(cell-1));
                double current_cell_value = get_node_value(*cell);

                // The visibility transition matrix (just one vector from); Eqn. 9
                double p_x_plus_1_given_x[] = {std::pow(m_a, number_of_cells_passed), 0.00000001};

                // Use transition matrix to predict p(x_m|o_{o:m-2}) : Equation(5)
                // Don't do full matrix multiple, only look at P(occupied) ignore p(not_occupied)
                double next_p_x = p_x_plus_1_given_x[0]*prev_p_x + p_x_plus_1_given_x[1]*(1-prev_p_x);

                // Do an update on the predicted next value : Eqn (6) & (7)
                double p_o_m_minus_1_given_x_m = 1 - prev_cell_value;
                next_p_x = p_o_m_minus_1_given_x_m*next_p_x / (p_o_m_minus_1_given_x_m*next_p_x +
                                                               (1-p_o_m_minus_1_given_x_m)*(1-next_p_x));

                // Clamp the value. This is ugly and not in the paper but seams necessary...
//                if (next_p_x > prev_p_x)
//                    next_p_x = prev_p_x;
                // actually maybe it is not necessary after all...

                prev_p_x = next_p_x;

                // The transition matrix p_o_m_given_o_m_minus_1 : Eqn (14) & (15)
                double t = 0.5+next_p_x/2.0;
                double p_o_m_given_o_m_minus_1[] = {t,1-t};

                // Predict the occupancy of the cell from previous one and visibility state : Eqn 11)
                double next_p_o[2] = {p_o_m_given_o_m_minus_1[0]*prev_p_o + p_o_m_given_o_m_minus_1[1]*(1-prev_p_o), 0};
                next_p_o[1]=1-next_p_o[0];

                // Do the update on the predicted occupancy using current value as measure : Eqn (12)
                next_p_o[0] = current_cell_value * next_p_o[0];
                next_p_o[1] = (1-current_cell_value) * next_p_o[1];
                if (next_p_o[0]+next_p_o[1] == 0)
                    next_p_o[0]=1;
                else
                    next_p_o[0] /= next_p_o[0]+next_p_o[1];

                prev_p_o = next_p_o[0];

                if (counting) {
                    // Calculate the info gain on this cell and add it to the tally
                    double prev_entropy = -current_cell_value*std::log(current_cell_value) -
                                          (1-current_cell_value)*std::log(1-current_cell_value);

                    double new_entropy = -prev_p_o*std::log(prev_p_o) -
                                          (1-prev_p_o)*std::log(1-prev_p_o);
                    double gain = prev_entropy - new_entropy; //(Eqn. 4)
                    view_gain +=gain;
                }
            }
        }

        return view_gain;
    }

    void NBVFinder::save_temporary_map() {
        // Save the octomap to disk in /tmp
        m_octree->writeBinary("/tmp/volume.bt");
    }

    double NBVFinder::get_node_value(const octomap::OcTreeKey &key) {
        double value = 0.5;
        octomap::OcTreeNode *node = m_octree->search(key,0);
        if (node != NULL) {
            value = node->getOccupancy();
        }
        return value;
    }

    bool NBVFinder::choose_next_view(unsigned int &selected_view_index) {
        // Choose which of the views provided by set_candidate_views is the best to select.
        std::vector<double> scores;
        scores.reserve(m_available_view_idx.size());

        double max_score=0;
        double score;
        unsigned  int best_index =-1;
        for (std::vector<unsigned int>::iterator view_index = m_available_view_idx.begin();
                view_index<m_available_view_idx.end();view_index++) {
            std::cout << "Evaluating view " << *view_index << std::endl;
            score = evaluate_view(m_candidate_views[*view_index]);
            if (score > max_score) {
                max_score=score;
                best_index = *view_index;
            };
            std::cout << " - score: " << score << std::endl;
        }
        if (best_index ==-1)
            return false;
        else {
            std::cout << "SELECTED VIEW: " << best_index << std::endl;
            std::vector<unsigned int>::iterator i=std::find(m_available_view_idx.begin(),
                                                            m_available_view_idx.end(), best_index);
            std::swap(*i, m_available_view_idx.back());
            m_available_view_idx.pop_back();
            selected_view_index = best_index;
            return true;
        }
    }
}