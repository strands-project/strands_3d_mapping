#ifndef reglibUtil_H
#define reglibUtil_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>
#include <time.h>
#include <sys/time.h>
#include <Eigen/Dense>

#include "../registration/ICP.h"
#include "nanoflann2.hpp"

namespace reglib
{
namespace RigidMotionEstimator5 {
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	/// @param Right hand side
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
	Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
								   Eigen::MatrixBase<Derived2>& Xn,
								   Eigen::MatrixBase<Derived3>& Y,
								   Eigen::MatrixBase<Derived4>& Yn,
								   const Eigen::MatrixBase<Derived5>& w,
								   const Eigen::MatrixBase<Derived6>& u,
								   bool dox,
								   bool doy);

	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
										  Eigen::MatrixBase<Derived2>& Xn,
										  Eigen::MatrixBase<Derived3>& Yp,
										  Eigen::MatrixBase<Derived4>& Yn,
										  const Eigen::MatrixBase<Derived5>& w,
										  bool dox = true,
										  bool doy = false);

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X, Eigen::MatrixBase<Derived2>& Y, const Eigen::MatrixBase<Derived3>& w);
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X, Eigen::MatrixBase<Derived2>& Y);
}



	template <typename T> struct ArrayData {
		int rows;
		int cols;
		T * data;

		inline size_t kdtree_get_point_count() const { return rows; }

		inline T kdtree_distance(const T *w1, const T *p1, const size_t idx,size_t /*size*/) const {
			T sum = 0;
			for(int i = 0; i < cols; i++){
				const T d0=p1[i]-data[cols*idx+i];
                //sum += w1[i]*d0*d0;
                sum += d0*d0;
			}
			return sum;
		}

		inline T kdtree_get_pt(const size_t idx, int dim) const {return data[cols*idx+dim];}
		template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

        template <typename X, typename U, typename V> inline T accum_dist(const X w, const U a, const V b, int ) const{
            //return w * (a-b) * (a-b);
            return (a-b) * (a-b);
        }
	};
	//typedef nanoflann2::KDTreeEigenMatrixAdaptor< Eigen::Matrix<double,-1,-1>, SAMPLES_DIM,nanoflann2::metric_L2_Simple> KDTreed;
	//typedef nanoflann2::KDTreeEigenMatrixAdaptor< Eigen::Matrix<float,-1,-1>, SAMPLES_DIM,nanoflann2::metric_L2_Simple> KDTreef;
	double getTime();
}

#endif
