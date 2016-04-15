#include "object_3d_benchmark/benchmark_overlap.h"

#include <pcl/octree/octree.h>

namespace benchmark_retrieval {

template<typename PointT,
         typename LeafContainerT = pcl::octree::OctreeContainerPointIndices,
         typename BranchContainerT = pcl::octree::OctreeContainerEmpty >
class OctreePointCloudOverlap : public pcl::octree::OctreePointCloud<PointT,
        LeafContainerT, BranchContainerT, pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT> >

{

public:

    /** \brief Constructor.
         *  \param resolution_arg:  octree resolution at lowest octree level
         * */
    OctreePointCloudOverlap(const double resolution_arg) :
        pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT,
        pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT> >(resolution_arg)
    {
    }

    /** \brief Empty class constructor. */
    virtual ~OctreePointCloudOverlap()
    {
    }

    /** \brief Get a indices from all leaf nodes that did not exist in previous buffer.
         * \param indicesVector_arg: results are written to this vector of int indices
         * \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to become serialized.
         * \return number of point indices
         */
    std::size_t getNewOccupiedVoxelCenters()
    {

        std::vector<pcl::octree::OctreeContainerPointIndices*> leaf_containers;
        this->serializeNewLeafs(leaf_containers); // this method is actually public, so actually no need to subclass
        return leaf_containers.size();
    }
};

double compute_overlap(CloudT::Ptr& A, CloudT::Ptr& B, float resolution)
{
    // Octree resolution - side length of octree voxels

    // Instantiate octree-based point cloud change detection class
    OctreePointCloudOverlap<PointT> octree(resolution);

    // Add points from cloudA to octree
    octree.setInputCloud(A);
    octree.addPointsFromInputCloud();

    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    double nbr_total_A = octree.getOccupiedVoxelCenters(dummy);

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    // Add points from cloudB to octree
    octree.setInputCloud(B);
    octree.addPointsFromInputCloud();

    double nbr_total_B = octree.getOccupiedVoxelCenters(dummy);

    // Get vector of point indices from octree voxels which did not exist in previous buffer

    double nbr_not_A = octree.getNewOccupiedVoxelCenters();

    if (nbr_not_A == nbr_total_B) {
        return 0.0;
    }

    /*
    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    // Important for this second part is that it seems like it cleans up empty branches first
    octree.switchBuffers();

    // Add points from cloudB to octree
    octree.setInputCloud(A);
    octree.addPointsFromInputCloud();

    double nbr_not_B = octree.getNewOccupiedVoxelCenters();

    double nbr_both = nbr_total_A - nbr_not_B;
    double nbr_total = nbr_total_A + nbr_not_A;
    */

    double nbr_both = nbr_total_B - nbr_not_A;
    double nbr_total = nbr_total_A + nbr_not_A;

    double overlap_fraction = nbr_both / nbr_total;

    return overlap_fraction;
}

} // namespace benchmark_retrieval
