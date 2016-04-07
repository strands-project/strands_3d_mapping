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

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"

using ceres::NumericDiffCostFunction;
using ceres::SizedCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::Solve;

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



template <typename T> struct ArrayData3D {
	int rows;
	T * data;

	inline size_t kdtree_get_point_count() const { return rows; }

	inline T kdtree_distance(const T *p1, const size_t idx,size_t /*size*/) const {
		const T d0=p1[0]-data[3*idx+0];
		const T d1=p1[1]-data[3*idx+1];
		const T d2=p1[2]-data[3*idx+2];
		return d0*d0 + d1*d1 + d2*d2;
	}

	inline T kdtree_get_pt(const size_t idx, int dim) const {return data[3*idx+dim];}
	template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	template <typename U, typename V> inline T accum_dist(const U a, const V b, int ) const{
		return (a-b) * (a-b);
	}
};

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


	struct PointCostFunctor {
		double m1 [3];
		double m2 [3];
		double w;

		PointCostFunctor(double m1x, double m1y, double m1z, double m2x, float m2y, double m2z, double w_){
			m1[0] = m1x;
			m1[1] = m1y;
			m1[2] = m1z;
			m2[0] = m2x;
			m2[1] = m2y;
			m2[2] = m2z;
			w = w_;
		}

		bool operator()(const double* const camera1, const double* camera2, double* residuals) const {
			double p1[3];
			ceres::AngleAxisRotatePoint(camera1, m1, p1);
			p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

			double p2[3];
			ceres::AngleAxisRotatePoint(camera2, m2, p2);
			p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

			residuals[0] = w*(p1[0]-p2[0]);
			residuals[1] = w*(p1[1]-p2[1]);
			residuals[2] = w*(p1[2]-p2[2]);
			return true;
		}
	};

	struct PointNormalCostFunctor {
		double m1 [3];
		double n1 [3];
		double m2 [3];
		double n2 [3];
		double w;

		PointNormalCostFunctor(double m1x, double m1y, double m1z, double n1x, double n1y, double n1z, double m2x, float m2y, double m2z, double n2x, float n2y, double n2z, double w_){
			m1[0] = m1x;
			m1[1] = m1y;
			m1[2] = m1z;
			m2[0] = m2x;
			m2[1] = m2y;
			m2[2] = m2z;

			n1[0] = n1x;
			n1[1] = n1y;
			n1[2] = n1z;
			n2[0] = n2x;
			n2[1] = n2y;
			n2[2] = n2z;
			w = w_;
		}

		bool operator()(const double* const camera1, const double* camera2, double* residuals) const {
			double p1[3];
			double pn1[3];
			ceres::AngleAxisRotatePoint(camera1, m1, p1);
			ceres::AngleAxisRotatePoint(camera1, n1, pn1);
			p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

			double p2[3];
			double pn2[3];
			ceres::AngleAxisRotatePoint(camera2, m2, p2);
			ceres::AngleAxisRotatePoint(camera2, n2, pn2);
			p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

			double dx = p1[0]-p2[0];
			double dy = p1[1]-p2[1];
			double dz = p1[2]-p2[2];
			residuals[0] = 0;
			residuals[1] = 0;
			residuals[0] = w*(dx*pn1[0]+dy*pn1[1]+dz*pn1[2]);
			residuals[1] = w*(dx*pn2[0]+dy*pn2[1]+dz*pn2[2]);

			return true;
		}
	};

	Eigen::Matrix4d getMatTest(const double * const camera, int mode = 0);
	double * getCamera(Eigen::Matrix4d mat, int mode = 0);
}

#endif
