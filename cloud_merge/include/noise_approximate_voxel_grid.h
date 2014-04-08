/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 * Class more or less copied from the PCL approximate voxel grid class,
 * see www.pointclouds.org for details
 *
 */

#ifndef NOISE_APPROXIMATE_VOXEL_GRID_H
#define NOISE_APPROXIMATE_VOXEL_GRID_H

#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <boost/mpl/size.hpp>

/** \brief ApproximateVoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
*
* \author James Bowman, Radu B. Rusu
* \ingroup filters
*/
class noise_approximate_voxel_grid : public pcl::Filter<pcl::PointXYZRGB>
{
  typedef pcl::PointXYZRGB PointT;

  using pcl::Filter<PointT>::filter_name_;
  using pcl::Filter<PointT>::getClassName;
  using pcl::Filter<PointT>::input_;
  using pcl::Filter<PointT>::indices_;

  typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

private:
  struct he
  {
    he () : ix (), iy (), iz (), count (0), centroid () {}
    int ix, iy, iz;
    int count;
    Eigen::VectorXf centroid;
  };

public:
  /** \brief Empty constructor. */
  noise_approximate_voxel_grid (int min_points, int skip_points) :
    pcl::Filter<PointT> (),
    leaf_size_ (Eigen::Vector3f::Ones ()),
    inverse_leaf_size_ (Eigen::Array3f::Ones ()),
    downsample_all_data_ (true), histsize_ (512),
    history_ (new he[histsize_]),
    min_points(min_points),
    skip_points(skip_points)
  {
    filter_name_ = "ApproximateVoxelGrid";
  }

  /** \brief Copy constructor.
    * \param[in] src the approximate voxel grid to copy into this.
    */
  noise_approximate_voxel_grid (const noise_approximate_voxel_grid &src) :
    pcl::Filter<PointT> (),
    leaf_size_ (src.leaf_size_),
    inverse_leaf_size_ (src.inverse_leaf_size_),
    downsample_all_data_ (src.downsample_all_data_),
    histsize_ (src.histsize_),
    history_ (),
    min_points(src.min_points),
    skip_points(src.skip_points)
  {
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
  }

  /** \brief Copy operator.
    * \param[in] src the approximate voxel grid to copy into this.
    */
  inline noise_approximate_voxel_grid&
  operator = (const noise_approximate_voxel_grid &src)
  {
    leaf_size_ = src.leaf_size_;
    inverse_leaf_size_ = src.inverse_leaf_size_;
    downsample_all_data_ = src.downsample_all_data_;
    histsize_ = src.histsize_;
    history_ = new he[histsize_];
    for (size_t i = 0; i < histsize_; i++)
      history_[i] = src.history_[i];
    min_points = src.min_points;
    skip_points = src.skip_points;
    return (*this);
  }

  /** \brief Set the voxel grid leaf size.
    * \param[in] leaf_size the voxel grid leaf size
    */
  inline void
  setLeafSize (const Eigen::Vector3f &leaf_size)
  {
    leaf_size_ = leaf_size;
    inverse_leaf_size_ = Eigen::Array3f::Ones () / leaf_size_.array ();
  }

  /** \brief Set the voxel grid leaf size.
    * \param[in] lx the leaf size for X
    * \param[in] ly the leaf size for Y
    * \param[in] lz the leaf size for Z
    */
  inline void
  setLeafSize (float lx, float ly, float lz)
  {
    setLeafSize (Eigen::Vector3f (lx, ly, lz));
  }

  /** \brief Get the voxel grid leaf size. */
  inline Eigen::Vector3f
  getLeafSize () const { return (leaf_size_); }

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
    * \param downsample the new value (true/false)
    */
  inline void
  setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

  /** \brief Get the state of the internal downsampling parameter (true if
    * all fields need to be downsampled, false if just XYZ).
    */
  inline bool
  getDownsampleAllData () const { return (downsample_all_data_); }

protected:
  /** \brief The size of a leaf. */
  Eigen::Vector3f leaf_size_;

  /** \brief Compute 1/leaf_size_ to avoid division later */
  Eigen::Array3f inverse_leaf_size_;

  /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
  bool downsample_all_data_;

  /** \brief history buffer size, power of 2 */
  size_t histsize_;

  /** \brief history buffer */
  struct he* history_;

  int min_points;

  int skip_points;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param output the resultant point cloud message
    */
  void
  applyFilter (PointCloud &output);

  /** \brief Write a single point from the hash to the output cloud
    */
  void
  flush(PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size);
};
#endif // NOISE_APPROXIMATE_VOXEL_GRID
