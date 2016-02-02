///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
#ifndef ICP_H
#define ICP_H
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <iostream>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#pragma once
///////////////////////////////////////////////////////////////////////////////
namespace nanoflann {
    /// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
    /// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
    template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
    struct KDTreeAdaptor {
        typedef KDTreeAdaptor<MatrixType,DIM,Distance> self_t;
        typedef typename MatrixType::Scalar              num_t;
        typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
        typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;
        index_t* index;
        KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
            const size_t dims = mat.rows();
            index = new index_t( dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
            index->buildIndex();
        }
        ~KDTreeAdaptor() {delete index;}
        const MatrixType &m_data_matrix;
        /// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
        inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
            nanoflann::KNNResultSet<typename MatrixType::Scalar,IndexType> resultSet(num_closest);
            resultSet.init(out_indices, out_distances_sq);
            index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
        }
        /// Query for the closest points to a given point (entered as query_point[0:dim-1]).
        inline IndexType closest(const num_t *query_point) const {
            IndexType out_indices;
            num_t out_distances_sq;
            query(query_point, 1, &out_indices, &out_distances_sq);
            return out_indices;
        }
        const self_t & derived() const {return *this;}
        self_t & derived() {return *this;}
        inline size_t kdtree_get_point_count() const {return m_data_matrix.cols();}
        /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const {
            num_t s=0;
            for (size_t i=0; i<size; i++) {
                const num_t d= p1[i]-m_data_matrix.coeff(i,idx_p2);
                s+=d*d;
            }
            return s;
        }
        /// Returns the dim'th component of the idx'th point in the class:
        inline num_t kdtree_get_pt(const size_t idx, int dim) const {
            return m_data_matrix.coeff(dim,idx);
        }
        /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const {return false;}
    };
}
///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   const Eigen::MatrixBase<Derived3>& w) {
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean, Y_mean;
        for(int i=0; i<3; ++i) {
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
        }
        X.colwise() -= X_mean;
        Y.colwise() -= Y_mean;
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
            transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        } else {
            transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
        }
        transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += Y_mean;
        /// Return transformation
        return transformation;
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Y) {
        return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   Eigen::MatrixBase<Derived3>& N,
                                   const Eigen::MatrixBase<Derived4>& w,
                                   const Eigen::MatrixBase<Derived5>& u) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for(int i=0; i<3; ++i){X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();}
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
        /// Prepare LHS and RHS
        Matrix66 LHS = Matrix66::Zero();
        Vector6 RHS = Vector6::Zero();
        Block33 TL = LHS.topLeftCorner<3,3>();
        Block33 TR = LHS.topRightCorner<3,3>();
        Block33 BR = LHS.bottomRightCorner<3,3>();
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
        #pragma omp parallel
        {
            #pragma omp for
            for(int i=0; i<X.cols(); i++) {
                C.col(i) = X.col(i).cross(N.col(i));
            }
            #pragma omp sections nowait
            {
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
                #pragma omp section
                for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
                #pragma omp section
                for(int i=0; i<C.cols(); i++) {
                    double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
                    RHS.head<3>() += C.col(i)*dist_to_plane;
                    RHS.tail<3>() += N.col(i)*dist_to_plane;
                }
            }
        }
        LHS = LHS.selfadjointView<Eigen::Upper>();
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::LDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += X_mean;
        /// Return transformation
        return transformation;
    }
    
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Yp,
                                          Eigen::MatrixBase<Derived3>& Yn,
                                          const Eigen::MatrixBase<Derived4>& w) {
        return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
    }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
    struct Parameters {
        bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
        double p = 1.0;           /// p norm
        double mu = 10.0;         /// penalty weight
        double alpha = 1.2;       /// penalty increase factor
        double max_mu = 1e5;      /// max penalty
        int max_icp = 100;        /// max ICP iteration
        int max_outer = 100;      /// max outer iteration
        int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
        double stop = 1e-5;       /// stopping criteria
        bool print_icpn = false;  /// (debug) print ICP iteration 
    };
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline double shrinkage(double mu, double n, double p, double s);
    template<>
    inline double shrinkage<0>(double, double, double, double s);
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p);
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXd& y, double mu, double p);
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    void point_to_point(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Parameters par = Parameters());
    
    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters

    Eigen::Matrix4d point_to_plane(	Eigen::Matrix3Xd& X,
                        			Eigen::Matrix3Xd& Y,
                        			Eigen::Matrix3Xd& N,
                        			Parameters par = Parameters());
}

///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {
    enum Function {
        PNORM,
        TUKEY,
        FAIR,
        LOGISTIC,
        TRIMMED,
        NONE
    };
    
    class Parameters {
    public:
        Parameters() : f(NONE),
                       p(0.1),
                       max_icp(100),
                       max_outer(100),
                       stop(1e-5) {}
        /// Parameters
        Function f;     /// robust function type
        double p;       /// paramter of the robust function
        int max_icp;    /// max ICP iteration
        int max_outer;  /// max outer iteration
        double stop;    /// stopping criteria
    };
    /// Weight functions
    /// @param Residuals
    /// @param Parameter
    void uniform_weight(Eigen::VectorXd& r);
    
    /// @param Residuals
    /// @param Parameter
    void pnorm_weight(Eigen::VectorXd& r, double p, double reg=1e-8);
    
    /// @param Residuals
    /// @param Parameter
    void tukey_weight(Eigen::VectorXd& r, double p);
    
    /// @param Residuals
    /// @param Parameter
    void fair_weight(Eigen::VectorXd& r, double p);
    
    /// @param Residuals
    /// @param Parameter
    void logistic_weight(Eigen::VectorXd& r, double p);

    /// @param Residuals
    /// @param Parameter
    void trimmed_weight(Eigen::VectorXd& r, double p);
    
    /// @param Function type
    /// @param Residuals
    /// @param Parameter
    void robust_weight(Function f, Eigen::VectorXd& r, double p);
    
    /// Reweighted ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    void point_to_point(Eigen::Matrix3Xd& X,
                        Eigen::Matrix3Xd& Y,
                        Parameters par = Parameters());
    
    /// Reweighted ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Matrix4d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Eigen::MatrixBase<Derived3>& N,
                        Parameters par = Parameters());
}
///////////////////////////////////////////////////////////////////////////////
#endif
