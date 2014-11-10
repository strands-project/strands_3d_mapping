#ifndef __CLOUD_MERGE__H
#define __CLOUD_MERGE__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/distances.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

template <class PointType>
class CloudMerge {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::iterator CloudIterator;

private:

    CloudPtr                                            m_IntermediateCloud;
    CloudPtr                                            m_MergedCloud;
    double                                              m_dMaximumPointDistance;

    std::vector<sensor_msgs::Image::ConstPtr> m_IntermediateDepthImages;
    std::vector<sensor_msgs::Image::ConstPtr> m_IntermediateRGBImages;
    std::vector<sensor_msgs::CameraInfoConstPtr> m_IntermediateCameraInfo;

public:

    sensor_msgs::ImagePtr                       m_IntermediateFilteredDepthImage;
    sensor_msgs::ImagePtr                       m_IntermediateFilteredRGBImage;

    sensor_msgs::CameraInfoConstPtr             m_IntermediateFilteredDepthCamInfo;


    CloudMerge(double maximumPointDistance = 4.0): m_IntermediateCloud(new Cloud()), m_MergedCloud(new Cloud())
    {
        m_dMaximumPointDistance = 4.0;
    }

    ~CloudMerge()
    {

    }

    void setMaximumPointDistance( double distance)
    {
        m_dMaximumPointDistance = distance;
    }

    void addIntermediateCloud(Cloud cloud)
    {
        *m_IntermediateCloud+=cloud;
        m_IntermediateCloud->header = cloud.header;
    }

    void addIntermediateImage(const sensor_msgs::Image::ConstPtr& depth_img, const sensor_msgs::Image::ConstPtr& rgb_img, const sensor_msgs::CameraInfo::ConstPtr& info_img)
    {
        m_IntermediateDepthImages.push_back(depth_img);
        m_IntermediateRGBImages.push_back(rgb_img);
        m_IntermediateCameraInfo.push_back(info_img);
    }

    void processIntermediateCloud()
    {
        *m_MergedCloud+=*m_IntermediateCloud;
        m_MergedCloud->header = m_IntermediateCloud->header;
        ROS_INFO_STREAM("Room observation now has "<<m_MergedCloud->points.size()<<" points");

        m_IntermediateCloud->clear();
    }

    CloudPtr subsampleIntermediateCloud() // this method first checks whether we are using point clouds directly from the sensor or whether we are averaging a set of depth images from the camera
    {
        Cloud subsampled_cloud;

	/*
        if (m_IntermediateCloud == 0)
        {
            return m_IntermediateCloud;
        }
	*/

        if (m_IntermediateCloud->points.size() != 0) // only process the intermediate point cloud if we are using it
        {
            return subsampleIntermediateCloudFromPointClouds();
        } else {
            return subsampleIntermediateCloudFromImages();
        }

    }

    void transformIntermediateCloud(const tf::Transform& transform, const std::string& new_frame = "")
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*m_IntermediateCloud, transformed_cloud,transform);
        m_IntermediateCloud->clear();
        *m_IntermediateCloud += transformed_cloud;
        if (new_frame != "")
        {
            m_IntermediateCloud->header.frame_id=new_frame;
        }
    }

    CloudPtr getIntermediateCloud()
    {
        return m_IntermediateCloud;
    }

    CloudPtr subsampleMergedCloud(double x, double y, double z)
    {
        Cloud subsampled_cloud;

        m_MergedCloud = CloudMerge<PointType>::filterPointCloud(m_MergedCloud, m_dMaximumPointDistance); // distance filtering, remove outliers and nans

        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud (m_MergedCloud);
        vg.setLeafSize (x,y,z);
        vg.filter (subsampled_cloud);

        m_MergedCloud->clear();

        *m_MergedCloud+=subsampled_cloud;

        ROS_INFO_STREAM("Room observation now has "<<m_MergedCloud->points.size()<<" points");

        return m_MergedCloud;
    }

    CloudPtr getMergedCloud()
    {
        return m_MergedCloud;
    }

    void transformMergedCloud(const tf::Transform& transform, const std::string& new_frame = "")
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*m_MergedCloud, transformed_cloud,transform);
        m_MergedCloud->clear();
        *m_MergedCloud+=transformed_cloud;
        if (new_frame != "")
        {
            m_MergedCloud->header.frame_id=new_frame;
        }
    }

    static CloudPtr filterPointCloud(CloudPtr input, const double& distance) // distance filtering from the centroid of the point cloud, remove outliers and nans
    {
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input, centroid);


        int vector_size = input->points.size();
        for (size_t i=0; i<vector_size; i++)
        {
            Eigen::Vector4f point(input->points[i].x,input->points[i].y,input->points[i].z,0.0);
            double dist_from_centroid = pcl::distances::l2(centroid,point);

            if (fabs(dist_from_centroid) > distance)
            {
                inliers->indices.push_back(i);
            }
        }

        // filter points based on indices
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (input);
        extract.setIndices (inliers);
        extract.setNegative (true);

        CloudPtr filtered_cloud(new Cloud);
        extract.filter (*filtered_cloud);

        *input = *filtered_cloud;

        return input;
    }

    void resetIntermediateCloud()
    {
        m_IntermediateCloud = CloudPtr(new Cloud());
    }

    void resetMergedCloud()
    {
        m_MergedCloud = CloudPtr(new Cloud());
    }

private:
    CloudPtr subsampleIntermediateCloudFromPointClouds()
    {
        // Point clouds are added to m_IntermediateCloud when the camera stops at one position during the sweep
        Cloud subsampled_cloud;
        if (m_IntermediateCloud->points.size() != 0)
        {
            //            ROS_INFO_STREAM("Subsampling intermediate cloud");
            //            noise_approximate_voxel_grid sor(3, 1);
            //            sor.setInputCloud(m_IntermediateCloud);
            //            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            //            sor.filter(subsampled_cloud);

            pcl::VoxelGrid<PointType> vg;
            vg.setInputCloud (m_IntermediateCloud);
            vg.setLeafSize (0.005f, 0.005f, 0.005f);
            vg.filter (subsampled_cloud);

            m_IntermediateCloud->clear();
        }

        m_IntermediateCloud->header = subsampled_cloud.header;
        m_IntermediateCloud->height = subsampled_cloud.height;
        m_IntermediateCloud->width  = subsampled_cloud.width;
        m_IntermediateCloud->is_dense = subsampled_cloud.is_dense;
        *m_IntermediateCloud+=subsampled_cloud;

        return m_IntermediateCloud;
    }

    CloudPtr subsampleIntermediateCloudFromImages()
    {
        // Images are added m_IntermediateDepthImages and m_IntermediateRGBImages when the camera stops at one position during the sweep
        Cloud subsampled_cloud;
        if (m_IntermediateDepthImages.size() != 0)
        {
            // convert images into point cloud
            subsampled_cloud.header = pcl_conversions::toPCL(m_IntermediateDepthImages.back()->header);
            subsampled_cloud.height = m_IntermediateDepthImages.back()->height;
            subsampled_cloud.width  = m_IntermediateDepthImages.back()->width;
            subsampled_cloud.is_dense = false;
            subsampled_cloud.points.resize(subsampled_cloud.height * subsampled_cloud.width);

            image_geometry::PinholeCameraModel aCameraModel;
            aCameraModel.fromCameraInfo(m_IntermediateCameraInfo[0]);

            float center_x = aCameraModel.cx();
            float center_y = aCameraModel.cy();

            // Supported color encodings: RGB8, BGR8, MONO8
            int red_offset, green_offset, blue_offset, color_step;
            if (m_IntermediateRGBImages[0]->encoding == sensor_msgs::image_encodings::RGB8)
            {
                red_offset   = 0;
                green_offset = 1;
                blue_offset  = 2;
                color_step   = 3;
            }
            else if (m_IntermediateRGBImages[0]->encoding == sensor_msgs::image_encodings::BGR8)
            {
                red_offset   = 2;
                green_offset = 1;
                blue_offset  = 0;
                color_step   = 3;
            }
            else if (m_IntermediateRGBImages[0]->encoding == sensor_msgs::image_encodings::MONO8)
            {
                red_offset   = 0;
                green_offset = 0;
                blue_offset  = 0;
                color_step   = 1;
            } else {
                ROS_INFO_STREAM("Color encoding not supported"<< m_IntermediateRGBImages[0]->encoding);
                red_offset   = 0;
                green_offset = 0;
                blue_offset  = 0;
                color_step   = 1;
            }

            // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
            float cx = 0.001 / aCameraModel.fx();
            float cy = 0.001 / aCameraModel.fy();

            // create new intermediate filtered images

            // depth
            m_IntermediateFilteredDepthImage = sensor_msgs::ImagePtr(new sensor_msgs::Image);
            m_IntermediateFilteredDepthImage->header = m_IntermediateDepthImages[0]->header;
            m_IntermediateFilteredDepthImage->height = m_IntermediateDepthImages[0]->height;
            m_IntermediateFilteredDepthImage->width  = m_IntermediateDepthImages[0]->width;
            m_IntermediateFilteredDepthImage->encoding  = m_IntermediateDepthImages[0]->encoding;
            m_IntermediateFilteredDepthImage->step  = m_IntermediateDepthImages[0]->step;
            m_IntermediateFilteredDepthImage->data.assign(m_IntermediateDepthImages[0]->data.begin(), m_IntermediateDepthImages[0]->data.end());

            // rgb
            m_IntermediateFilteredRGBImage = sensor_msgs::ImagePtr(new sensor_msgs::Image);
            m_IntermediateFilteredRGBImage->header = m_IntermediateRGBImages[0]->header;
            m_IntermediateFilteredRGBImage->height = m_IntermediateRGBImages[0]->height;
            m_IntermediateFilteredRGBImage->width  = m_IntermediateRGBImages[0]->width;
            m_IntermediateFilteredRGBImage->encoding  = m_IntermediateRGBImages[0]->encoding;
            m_IntermediateFilteredRGBImage->step  = m_IntermediateRGBImages[0]->step;
            m_IntermediateFilteredRGBImage->data.assign(m_IntermediateRGBImages[0]->data.begin(), m_IntermediateRGBImages[0]->data.end());

            m_IntermediateFilteredDepthCamInfo = m_IntermediateCameraInfo[0];

            CloudIterator pt_iter = subsampled_cloud.begin();

            const uint8_t* rgb_buffer = &m_IntermediateRGBImages[0]->data[0];
            int row_step = m_IntermediateDepthImages[0]->step / sizeof(uint16_t);
            int row_index = 0;
            int color_index = 0;
            for (int v = 0; v < (int)subsampled_cloud.height; ++v, row_index += row_step)
            {
                for (int u = 0; u < (int)subsampled_cloud.width; ++u)
                {
                    PointType& pt = *pt_iter++;

                    int temp_depth = 0; // use for mean filter
                    std::vector<uint16_t> depths; // use for median filter

                    for (int k=0; k<m_IntermediateDepthImages.size();k++)
                    {
                        const uint16_t* current_depth_index = reinterpret_cast<const uint16_t*>(&m_IntermediateDepthImages[k]->data[0])+row_index;
                        if (current_depth_index[u] == 0)
                        {
                            continue;
                        }
                        depths.push_back(current_depth_index[u]);
                        temp_depth+=current_depth_index[u];
                    }

                    uint16_t mean_depth = 0;
                    uint16_t median_depth = 0;
                    if (depths.size() != 0)
                    {
                        // mean filter
                        mean_depth = (uint16_t)(temp_depth/depths.size());
                        // median filter
                        std::sort(depths.begin(), depths.end());
                        int median_pos = (int)(depths.size()/2);
                        median_depth = depths[median_pos];
                    }

                    uint16_t point_depth = median_depth; // use median filter

                    uint16_t* filtered_image_depth_index = reinterpret_cast<uint16_t*>(&m_IntermediateFilteredDepthImage->data[0])+row_index;
                    filtered_image_depth_index[u] = point_depth;

                    // Missing points denoted by NaNs
                    if (!(point_depth != 0))
                    {
                        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                    } else {
                        // Fill in XYZ
                        pt.x = (u - center_x) * point_depth * cx;
                        pt.y = (v - center_y) * point_depth * cy;
                        pt.z = point_depth * 0.001f; // convert uint16 to meters
                    }

                    uint32_t rgb = ((uint8_t)rgb_buffer[color_index + red_offset] << 16 | (uint8_t)rgb_buffer[color_index + green_offset] << 8 | (uint8_t)rgb_buffer[color_index + blue_offset]);
                    pt.rgb = *reinterpret_cast<float*>(&rgb);

                    color_index += color_step;
                }
            }

            m_IntermediateDepthImages.clear();
            m_IntermediateRGBImages.clear();
            m_IntermediateCameraInfo.clear();
        }

        m_IntermediateCloud->header = subsampled_cloud.header;
        m_IntermediateCloud->height = subsampled_cloud.height;
        m_IntermediateCloud->width  = subsampled_cloud.width;
        m_IntermediateCloud->is_dense = subsampled_cloud.is_dense;

        *m_IntermediateCloud+=subsampled_cloud;

        return m_IntermediateCloud;
    }
};


#endif
