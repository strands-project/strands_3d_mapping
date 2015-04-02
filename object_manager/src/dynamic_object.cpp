#include "dynamic_object.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <flann/flann.h>

using namespace std;

DynamicObject::DynamicObject(bool verbose) : m_points(new Cloud()), m_normals(new NormalCloud), m_label("unknown"), m_bVerbose(verbose)
{
    m_roomLogString = "";
    m_roomStringId = "";
    m_noAdditionalViews = 0;
}

DynamicObject::~DynamicObject()
{

}

void DynamicObject::setTime(boost::posix_time::ptime time)
{
    m_time = time;
}

void DynamicObject::setCloud(CloudPtr cloud)
{
    m_points = cloud->makeShared();
    computeCentroid();
    computeBbox();
}

auto DynamicObject::getAdditionalViews() -> decltype(m_vAdditionalViews)
{
    return m_vAdditionalViews;
}

void DynamicObject::addAdditionalView(CloudPtr view)
{
    m_vAdditionalViews.push_back(view);
    m_noAdditionalViews++;
}

void DynamicObject::updateAllData()
{
    computeCentroid();
    computeBbox();
    computeNormals(0.1);

}


void DynamicObject::computeCentroid()
{
    pcl::compute3DCentroid(*m_points, m_centroid);
}

void DynamicObject::computeBbox()
{
    double max_z = std::numeric_limits<double>::max();
    m_bboxHighest.z = -max_z;m_bboxHighest.y = -max_z;m_bboxHighest.x = -max_z;
    m_bboxLowest.z = max_z;m_bboxLowest.y = max_z;m_bboxLowest.x = max_z;

    for (size_t i=0; i<m_points->points.size(); i++)
    {
        if (m_points->points[i].z > m_bboxHighest.z)
        {
            m_bboxHighest.z = m_points->points[i].z;
        }

        if (m_points->points[i].z < m_bboxLowest.z)
        {
            m_bboxLowest.z = m_points->points[i].z;
        }

        if (m_points->points[i].y > m_bboxHighest.y)
        {
            m_bboxHighest.y = m_points->points[i].y;
        }

        if (m_points->points[i].y < m_bboxLowest.y)
        {
            m_bboxLowest.y = m_points->points[i].y;
        }

        if (m_points->points[i].x > m_bboxHighest.x)
        {
            m_bboxHighest.x = m_points->points[i].x;
        }

        if (m_points->points[i].x < m_bboxLowest.x)
        {
            m_bboxLowest.x = m_points->points[i].x;
        }
    }
}

void DynamicObject::computeNormals(double radius_search, int noThreads)
{
    pcl::NormalEstimationOMP<PointType, NormalType> normalEstimation;
    normalEstimation.setNumberOfThreads(noThreads);
    normalEstimation.setInputCloud(m_points);
    normalEstimation.setRadiusSearch(radius_search);
    Tree::Ptr kdtree(new Tree);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*m_normals);
}

void DynamicObject::computeFitness(CloudPtr one, CloudPtr other, double& fitnessScore, int& noPoints, double max_range)
{
    noPoints = 0;
    fitnessScore = 0;

    TreePtr tree(new Tree);
    tree->setInputCloud(other);


    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    for (size_t i = 0; i < one->points.size (); ++i)
    {
        Eigen::Vector4f p1 = Eigen::Vector4f (one->points[i].x,
                                              one->points[i].y,
                                              one->points[i].z, 0);
        // Find its nearest neighbor in the target
        tree->nearestKSearch (one->points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] > max_range)
            continue;

        Eigen::Vector4f p2 = Eigen::Vector4f (other->points[nn_indices[0]].x,
                other->points[nn_indices[0]].y,
                other->points[nn_indices[0]].z, 0);
        // Calculate the fitness score
        fitnessScore += fabs ((p1-p2).squaredNorm ());
        noPoints++;
    }

    if (noPoints > 0)
        fitnessScore /= noPoints;
    else
        fitnessScore = std::numeric_limits<double>::max ();


}

bool DynamicObject::operator==(const DynamicObject& rhs)
{
    if (m_roomLogString != rhs.m_roomLogString)
    {
        if (m_bVerbose)
        {
            cout<<"Room log string not the same "<<endl;
        }
        return false;
    }
    if (m_roomStringId != rhs.m_roomStringId)
    {
        if (m_bVerbose)
        {
            cout<<"Room string id not the same "<<endl;
        }
        return false;
    }

    if (m_label != rhs.m_label)
    {
        if (m_bVerbose)
        {
            cout<<"Label not the same "<<endl;
        }
        return false;
    }

    if (m_points->points.size() != rhs.m_points->points.size())
    {
        if (m_bVerbose)
        {
            cout<<"No points not the same "<<endl;
        }
        return false;
    }

    for (size_t i=0; i<m_points->points.size(); i++)
    {
        if ((m_points->points[i].x != rhs.m_points->points[i].x) ||
                (m_points->points[i].y != rhs.m_points->points[i].y) ||
                (m_points->points[i].z != rhs.m_points->points[i].z))
        {
            if (m_bVerbose)
            {
                cout<<"Point not the same "<<endl;
            }
            return false;
        }
    }

    return true;
}

bool DynamicObject::operator!=(const DynamicObject& rhs)
{
    return ! (*this==rhs);
}


void DynamicObject::computeVFHFeatures()
{
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (m_points);
    vfh.setInputNormals (m_normals);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    vfh.setSearchMethod (tree);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    // Compute the features
    vfh.compute (*vfhs);

    m_vfh.resize(308);
    for (size_t i = 0; i < 308; ++i)
    {
        m_vfh[i] = vfhs->points[0].histogram[i];
    }

    if (m_bVerbose)
    {
        cout<<"Computed VFH descriptor "<<endl;
    }
}
