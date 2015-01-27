#ifndef __OCCLUSION_CHECKER__H
#define __OCCLUSION_CHECKER__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/segment_differences.h>

#include "ros/time.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include "roombase.h"

template <class PointType>
class OcclusionChecker {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    struct occluded_points
    {
        CloudPtr toBeAdded;
        CloudPtr toBeRemoved;
    };

    OcclusionChecker() : m_SensorOrigin(Eigen::Vector3f(0.0,0.0,0.0))
    {

    }

    ~OcclusionChecker()
    {

    }

    void setSensorOrigin(Eigen::Vector3f origin)
    {
        m_SensorOrigin = origin;
    }

    void setSensorOrigin(tf::Vector3 origin)
    {
        m_SensorOrigin = Eigen::Vector3f(origin.x(), origin.y(), origin.z());
    }

    std::vector<CloudPtr>    checkOcclusions(std::vector<CloudPtr>& differenceMetaRoomToRoomClusters,
                            std::vector<CloudPtr>& differenceRoomToMetaRoomClusters, int numberOfBins = 360)
    {
        ROS_INFO_STREAM("Checking occlusions");
        // Transformation to origin -> needed for spherical projection
        Eigen::Matrix4f transformFromOrigin = Eigen::Affine3f(Eigen::Translation3f(m_SensorOrigin)).matrix();
        Eigen::Matrix4f transformToOrigin = transformFromOrigin.inverse();

//        ROS_INFO_STREAM("Transform from origin "<< transformFromOrigin);
//        ROS_INFO_STREAM("Transform to origin "<< transformToOrigin);

        const double pi = std::acos(-1.0);
        std::vector<CloudPtr> toBeAdded;
        // for each cluster in the room to metaroom vector, check if it occludes anything
        // if yes, remove the occluded cluster so that it is not deleted from the metaroom
        bool clusterAdded = false;
        /*********** CLUSTERS TO BE REMOVED ************************************/
        for (size_t i=0; i<differenceRoomToMetaRoomClusters.size(); i++)
        {
            bool clusterAdded = false;
            // project clusters on sphere and check occlusions
            double thetaphi[numberOfBins][numberOfBins];
            for (size_t j=0; j<numberOfBins; j++)
            {
                for(size_t k=0; k<numberOfBins; k++)
                {
                    thetaphi[j][k] = 0.0;
                }
            }

            int occluded_clusters = 0;

            // transform cluster to Origin
            CloudPtr transformedRoomCluster(new Cloud);
            pcl::transformPointCloud (*differenceRoomToMetaRoomClusters[i], *transformedRoomCluster, transformToOrigin);

            for (size_t j=0; j<transformedRoomCluster->points.size(); j++)
            {
                // convert to spherical coordinates
                double r = sqrt(pow(transformedRoomCluster->points[j].x,2) + pow(transformedRoomCluster->points[j].y,2) + pow(transformedRoomCluster->points[j].z,2));
                double theta = pi+acos(transformedRoomCluster->points[j].z/r);
                double phi = pi+atan2(transformedRoomCluster->points[j].y,transformedRoomCluster->points[j].x);

//                ROS_INFO_STREAM("Theta: "<<theta<<";  Phi: "<<phi<<"; r: "<<r<<"; current point: "<<transformedRoomCluster->points[j].x<<" "<<transformedRoomCluster->points[j].y<<" "<<differenceRoomToMetaRoomClusters[i]->points[j].z);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                thetaphi[thetabin][phibin] = r;
            }

//            ROS_INFO_STREAM("Spherical projection computed");
            // check occlusion with other clusters
            typename std::vector<CloudPtr>::iterator metaRoomClusterIterator = differenceMetaRoomToRoomClusters.begin();
            while(metaRoomClusterIterator != differenceMetaRoomToRoomClusters.end())
            {
                int occluded = 0;
                int behind = 0;
                int infront = 0;
                // transform cluster to Origin
                CloudPtr transformedMetaRoomCluster(new Cloud);
                pcl::transformPointCloud (*(*metaRoomClusterIterator), *transformedMetaRoomCluster, transformToOrigin);

                CloudPtr pointsFront(new Cloud);
                CloudPtr pointsBehind(new Cloud);

                for (size_t k=0; k < transformedMetaRoomCluster->points.size(); k++)
                {
//                    if (k==j) continue;
                    // take spherical projection
                    double r = sqrt(pow(transformedMetaRoomCluster->points[k].x,2) + pow(transformedMetaRoomCluster->points[k].y,2) + pow(transformedMetaRoomCluster->points[k].z,2));
                    double theta = pi+acos(transformedMetaRoomCluster->points[k].z/r);
                    double phi = pi+atan2(transformedMetaRoomCluster->points[k].y,transformedMetaRoomCluster->points[k].x);

                    int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                    int phibin = (int)((phi/(2*pi)) * numberOfBins);
                    if (thetaphi[thetabin][phibin] != 0.0)
                    {
                        occluded++;
                        if (thetaphi[thetabin][phibin] > r)
                        {
                            infront++;
                            pointsFront->points.push_back((*metaRoomClusterIterator)->points[k]);
                        } else {
                            behind++;
                            pointsBehind->points.push_back((*metaRoomClusterIterator)->points[k]);
                        }
                    }
                }

//                ROS_INFO_STREAM("Cluster "<<i<<" occluded "<<occluded<<" behind "<<behind<<" infront "<<infront<<"  total points  "<<transformedMetaRoomCluster->points.size());
                if (occluded != 0)
                {
                    if (/*(behind > infront)*/(behind > 0))
                    {
                        // remove this cluster so that it's not deleted from the metaroom
                        // only remove it if it's occluded by a large enough cluster
                        //                        if (transformedMetaRoomCluster->points.size() > 0.5 *(*metaRoomClusterIterator)->points.size() )
                        //                        {
                        ROS_INFO_STREAM("Removing occluded points : "<<behind<<" and adding "<<infront);
                        // segment out the occluded points which shouldn't be removed, and add the ones which are not occluded
                        CloudPtr remainingPoints(new Cloud());
                        pcl::SegmentDifferences<PointType> segment;
                        segment.setDistanceThreshold(0.001);
                        segment.setInputCloud(*metaRoomClusterIterator);
                        segment.setTargetCloud(pointsBehind);
                        segment.segment(*remainingPoints);
                        if (remainingPoints->points.size() != 0)
                        {
//                            ROS_INFO_STREAM("Removing a cluster, no points: "<<(*metaRoomClusterIterator)->points.size());
                            metaRoomClusterIterator = differenceMetaRoomToRoomClusters.erase(metaRoomClusterIterator);
                        } else {
                            metaRoomClusterIterator++;
                        }
                    } else {
                        if (!clusterAdded)
                        {
                            // only add this cluster if it's occluded by a large enough cluster
                            if ((*metaRoomClusterIterator)->points.size() > 0.5 * differenceRoomToMetaRoomClusters[i]->points.size())
                            {
//                                ROS_INFO_STREAM("Found a cluster that needs to be added to the metaroom, no points "<<differenceRoomToMetaRoomClusters[i]->points.size());
                                toBeAdded.push_back(differenceRoomToMetaRoomClusters[i]);                                
                                clusterAdded = true;
                            }
                        }
                        metaRoomClusterIterator++;
                    }
                }
                else {
                    metaRoomClusterIterator++;
                }
            }
        }

        return toBeAdded;
    }


    occluded_points   checkOcclusions(CloudPtr differenceMetaRoomToRoom,
                                      CloudPtr differenceRoomToMetaRoom, int numberOfBins = 360)
    {
        ROS_INFO_STREAM("Checking occlusions");
        occluded_points toRet;
        toRet.toBeAdded = CloudPtr(new Cloud());
        toRet.toBeRemoved = CloudPtr(new Cloud());

        // Transformation to origin -> needed for spherical projection
        Eigen::Matrix4f transformFromOrigin = Eigen::Affine3f(Eigen::Translation3f(m_SensorOrigin)).matrix();
        Eigen::Matrix4f transformToOrigin = transformFromOrigin.inverse();


        const double pi = std::acos(-1.0);
        /*********** CLUSTERS TO BE REMOVED ************************************/
        {
            double thetaphi[numberOfBins][numberOfBins];
            for (size_t j=0; j<numberOfBins; j++)
            {
                for(size_t k=0; k<numberOfBins; k++)
                {
                    thetaphi[j][k] = 0.0;
                }
            }

            // transform cluster to Origin
            CloudPtr transformedRoomCloud(new Cloud);
            pcl::transformPointCloud (*differenceRoomToMetaRoom, *transformedRoomCloud, transformToOrigin);

            for (size_t j=0; j<transformedRoomCloud->points.size(); j++)
            {
                // convert to spherical coordinates
                double r = sqrt(pow(transformedRoomCloud->points[j].x,2) + pow(transformedRoomCloud->points[j].y,2) + pow(transformedRoomCloud->points[j].z,2));
                double theta = pi+acos(transformedRoomCloud->points[j].z/r);
                double phi = pi+atan2(transformedRoomCloud->points[j].y,transformedRoomCloud->points[j].x);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                thetaphi[thetabin][phibin] = r;
            }


            // check occlusion with other clusters
            int occluded = 0;
            int behind = 0;
            int infront = 0;
            // transform cluster to Origin
            CloudPtr transformedMetaRoomCloud(new Cloud);
            pcl::transformPointCloud (*differenceMetaRoomToRoom, *transformedMetaRoomCloud, transformToOrigin);

            CloudPtr pointsFront(new Cloud);
            CloudPtr pointsBehind(new Cloud);

            for (size_t k=0; k < transformedMetaRoomCloud->points.size(); k++)
            {
                //                    if (k==j) continue;
                // take spherical projection
                double r = sqrt(pow(transformedMetaRoomCloud->points[k].x,2) + pow(transformedMetaRoomCloud->points[k].y,2) + pow(transformedMetaRoomCloud->points[k].z,2));
                double theta = pi+acos(transformedMetaRoomCloud->points[k].z/r);
                double phi = pi+atan2(transformedMetaRoomCloud->points[k].y,transformedMetaRoomCloud->points[k].x);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                if (thetaphi[thetabin][phibin] != 0.0)
                {
                    occluded++;
                    if (thetaphi[thetabin][phibin] > r)
                    {
                        infront++;
                        pointsFront->points.push_back(transformedMetaRoomCloud->points[k]);
                    } else {
                        behind++;
                        pointsBehind->points.push_back(transformedMetaRoomCloud->points[k]);
                    }
                }
            }

            ROS_INFO_STREAM("Removing occluded points : "<<behind<<" and adding "<<infront);
             CloudPtr transformedPointsFront(new Cloud);
             pcl::transformPointCloud (*pointsFront, *transformedPointsFront, transformFromOrigin);
            *toRet.toBeRemoved += *transformedPointsFront;

        }

        /*********** CLUSTERS TO BE ADDED ************************************/
        {
            bool clusterAdded = false;
            // project clusters on sphere and check occlusions
            double thetaphi[numberOfBins][numberOfBins];
            for (size_t j=0; j<numberOfBins; j++)
            {
                for(size_t k=0; k<numberOfBins; k++)
                {
                    thetaphi[j][k] = 0.0;
                }
            }

            // transform cluster to Origin
            CloudPtr transformedMetaRoomCloud(new Cloud);
            pcl::transformPointCloud (*differenceMetaRoomToRoom, *transformedMetaRoomCloud, transformToOrigin);


            for (size_t j=0; j<transformedMetaRoomCloud->points.size(); j++)
            {
                // convert to spherical coordinates
                double r = sqrt(pow(transformedMetaRoomCloud->points[j].x,2) + pow(transformedMetaRoomCloud->points[j].y,2) + pow(transformedMetaRoomCloud->points[j].z,2));
                double theta = pi+acos(transformedMetaRoomCloud->points[j].z/r);
                double phi = pi+atan2(transformedMetaRoomCloud->points[j].y,transformedMetaRoomCloud->points[j].x);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                thetaphi[thetabin][phibin] = r;
            }

            // check occlusion with other clusters
            int occluded = 0;
            int behind = 0;
            int infront = 0;
            // transform cluster to Origin
            CloudPtr transformedRoomCloud(new Cloud);
            pcl::transformPointCloud (*differenceRoomToMetaRoom, *transformedRoomCloud, transformToOrigin);

            CloudPtr pointsFront(new Cloud);
            CloudPtr pointsBehind(new Cloud);

            for (size_t k=0; k < transformedRoomCloud->points.size(); k++)
            {
                //                    if (k==j) continue;
                // take spherical projection
                double r = sqrt(pow(transformedRoomCloud->points[k].x,2) + pow(transformedRoomCloud->points[k].y,2) + pow(transformedRoomCloud->points[k].z,2));
                double theta = pi+acos(transformedRoomCloud->points[k].z/r);
                double phi = pi+atan2(transformedRoomCloud->points[k].y,transformedRoomCloud->points[k].x);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                if (thetaphi[thetabin][phibin] != 0.0)
                {
                    occluded++;
                    if (thetaphi[thetabin][phibin] > r)
                    {
                        infront++;
                        pointsFront->points.push_back(transformedRoomCloud->points[k]);
                    } else {
                        behind++;
                        pointsBehind->points.push_back(transformedRoomCloud->points[k]);
                    }
                }
            }

            ROS_INFO_STREAM("Adding non-occluded points : "<<behind<<" and discarding "<<infront);
            CloudPtr transformedPointsBehind(new Cloud);
            pcl::transformPointCloud (*pointsBehind, *transformedPointsBehind, transformFromOrigin);
            *toRet.toBeAdded = *transformedPointsBehind;
        }

        return toRet;
    }

private:
    Eigen::Vector3f         m_SensorOrigin;

};
#endif
