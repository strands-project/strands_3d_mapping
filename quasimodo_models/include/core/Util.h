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

//#include "../registration/ICP.h"
#include "nanoflann.hpp"
#include <Eigen/Dense>

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

//namespace nanoflann {
//	/// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
//	/// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
//	template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
//	struct KDTreeAdaptor {
//		typedef KDTreeAdaptor<MatrixType,DIM,Distance> self_t;
//		typedef typename MatrixType::Scalar              num_t;
//		typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
//		typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;
//		index_t* index;
//		KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
//			const size_t dims = mat.rows();
//			index = new index_t( dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
//			index->buildIndex();
//		}
//		~KDTreeAdaptor() {delete index;}
//		const MatrixType &m_data_matrix;
//		/// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
//		inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
//			nanoflann::KNNResultSet<typename MatrixType::Scalar,IndexType> resultSet(num_closest);
//			resultSet.init(out_indices, out_distances_sq);
//			index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
//		}
//		/// Query for the closest points to a given point (entered as query_point[0:dim-1]).
//		inline IndexType closest(const num_t *query_point) const {
//			IndexType out_indices;
//			num_t out_distances_sq;
//			query(query_point, 1, &out_indices, &out_distances_sq);
//			return out_indices;
//		}
//		const self_t & derived() const {return *this;}
//		self_t & derived() {return *this;}
//		inline size_t kdtree_get_point_count() const {return m_data_matrix.cols();}
//		/// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
//		inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const {
//			num_t s=0;
//			for (size_t i=0; i<size; i++) {
//				const num_t d= p1[i]-m_data_matrix.coeff(i,idx_p2);
//				s+=d*d;
//			}
//			return s;
//		}
//		/// Returns the dim'th component of the idx'th point in the class:
//		inline num_t kdtree_get_pt(const size_t idx, int dim) const {
//			return m_data_matrix.coeff(dim,idx);
//		}
//		/// Optional bounding-box computation: return false to default to a standard bbox computation loop.
//		template <class BBOX> bool kdtree_get_bbox(BBOX&) const {return false;}
//	};
//}

// This is an exampleof a custom data set class


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

	typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, ArrayData3D<double> > , ArrayData3D<double>, 3 > KDTREE;
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

	Eigen::Matrix4d constructTransformationMatrix (const double & alpha, const double & beta, const double & gamma, const double & tx,    const double & ty,   const double & tz);

	void point_to_plane2(		Eigen::Matrix<double, 3, Eigen::Dynamic> & X,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Y,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Yn,
								Eigen::VectorXd & W);
	bool isconverged(std::vector<Eigen::Matrix4d> before, std::vector<Eigen::Matrix4d> after, double stopvalr = 0.001, double stopvalt = 0.001);
	void setFirstIdentity(std::vector<Eigen::Matrix4d> & v);
}

namespace nanoflann{
template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};
}

typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, reglib::ArrayData3D<double> > , reglib::ArrayData3D<double>,3> Tree3d;
typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, nanoflann::PointCloud<double> > , nanoflann::PointCloud<double>,3> my_kd_tree_t;

#endif
