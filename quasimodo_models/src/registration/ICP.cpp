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
//#ifndef ICP_H
//#define ICP_H
//#pragma once
#include "ICP.h"
#include "nanoflann.hpp"
#include <Eigen/Dense>
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline double shrinkage(double mu, double n, double p, double s) {
        return shrinkage<I-1>(mu, n, p, 1.0 - (p/mu)*std::pow(n, p-2.0)*std::pow(s, p-1.0));
    }
    template<>
    inline double shrinkage<0>(double, double, double, double s) {return s;}
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p) {
        double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
        double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
        #pragma omp parallel for
        for(int i=0; i<Q.cols(); ++i) {
            double n = Q.col(i).norm();
            double w = 0.0;
            if(n > ha) w = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
            Q.col(i) *= w;
        }
    }
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXd& y, double mu, double p) {
        double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
        double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
        #pragma omp parallel for
        for(int i=0; i<y.rows(); ++i) {
            double n = std::abs(y(i));
            double s = 0.0;
            if(n > ha) s = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
            y(i) *= s;
        }
    }
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    void point_to_point(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Parameters par) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
            }
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = X-Q+C/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::Matrix3Xd U = Q+Z-C/mu;
                    RigidMotionEstimator::point_to_point(X, U);
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::Matrix3Xd P = X-Q-Z;
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop) break;
        }
    }
    
    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters

    Eigen::Matrix4d point_to_plane(	Eigen::Matrix3Xd& X,
                        			Eigen::Matrix3Xd& Y,
                        			Eigen::Matrix3Xd& N,
                        			Parameters par) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd Z = Eigen::VectorXd::Zero(X.cols());
        Eigen::VectorXd C = Eigen::VectorXd::Zero(X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("point_to_plane"));
		viewer->addCoordinateSystem();
		viewer->setBackgroundColor(0.5,0.5,0.5);

		double primal_prev = 999999999;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn){ std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;}

            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                int id = kdtree.closest(X.col(i).data());
                Qp.col(i) = Y.col(id);
                Qn.col(i) = N.col(id);
            }

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Qp.cols();
	scloud->points.clear();
	dcloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i); p.y = X(1,i); p.z = X(2,i); p.b = 0;p.g = 255;p.r = 0;  scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Qp(0,i);p.y = Qp(1,i);p.z = Qp(2,i);p.b = 0;p.g = 0;	 p.r = 255;dcloud->points.push_back(p);}		
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();
	
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                bool run_inner = true;
                for(int inner=0; inner<par.max_inner && run_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()+C.array()/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::VectorXd U = Z-C/mu;
                    Eigen::Affine3d transformation = RigidMotionEstimator::point_to_plane(X, Qp, Qn, Eigen::VectorXd::Ones(X.cols()), U);
                    T *= transformation.matrix();
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    //printf("inner %i %i -> %f < %f = %i\n",outer,inner,dual,par.stop,dual < par.stop);
                    Xo1 = X;
                    if(dual < par.stop){		run_inner = false;}
                    if(inner >= par.max_inner){	run_inner = false;}
                }
                /// C update (lagrange multipliers)
                Eigen::VectorXd P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
                
                if(!par.use_penalty){ C.noalias() += mu*P; }
                /// mu update (penalty)
                if(mu < par.max_mu){ mu *= par.alpha; }
                /// Stopping criteria
                double primal = P.array().abs().maxCoeff();
                if(primal < par.stop && dual < par.stop){ break; }
                double pdiff = primal_prev-primal;
                primal_prev = primal;
                //if(pdiff < par.stop){ break;}  
            }
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop){ break; }
        }
    	return T;
    }
}

///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {

    /// Weight functions
    /// @param Residuals
    /// @param Parameter
    void uniform_weight(Eigen::VectorXd& r) {
        r = Eigen::VectorXd::Ones(r.rows());
    }
    /// @param Residuals
    /// @param Parameter
    void pnorm_weight(Eigen::VectorXd& r, double p, double reg) {
        for(int i=0; i<r.rows(); ++i) {
            r(i) = p/(std::pow(r(i),2-p) + reg);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void tukey_weight(Eigen::VectorXd& r, double p) {
        for(int i=0; i<r.rows(); ++i) {
            if(r(i) > p) r(i) = 0.0;
            else r(i) = std::pow((1.0 - std::pow(r(i)/p,2.0)), 2.0);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void fair_weight(Eigen::VectorXd& r, double p) {
        for(int i=0; i<r.rows(); ++i) {
            r(i) = 1.0/(1.0 + r(i)/p);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void logistic_weight(Eigen::VectorXd& r, double p) {
        for(int i=0; i<r.rows(); ++i) {
            r(i) = (p/r(i))*std::tanh(r(i)/p);
        }
    }
    struct sort_pred {
        bool operator()(const std::pair<int,double> &left,
                        const std::pair<int,double> &right) {
            return left.second < right.second;
        }
    };
    /// @param Residuals
    /// @param Parameter
    void trimmed_weight(Eigen::VectorXd& r, double p) {
        std::vector<std::pair<int, double> > sortedDist(r.rows());
        for(int i=0; i<r.rows(); ++i) {
            sortedDist[i] = std::pair<int, double>(i,r(i));
        }
        std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
        r.setZero();
        int nbV = r.rows()*p;
        for(int i=0; i<nbV; ++i) {
            r(sortedDist[i].first) = 1.0;
        }
    }
    /// @param Function type
    /// @param Residuals
    /// @param Parameter
    void robust_weight(Function f, Eigen::VectorXd& r, double p) {
        switch(f) {
            case PNORM: pnorm_weight(r,p); break;
            case TUKEY: tukey_weight(r,p); break;
            case FAIR: fair_weight(r,p); break;
            case LOGISTIC: logistic_weight(r,p); break;
            case TRIMMED: trimmed_weight(r,p); break;
            case NONE: uniform_weight(r); break;
            default: uniform_weight(r); break;
        }
    }
    
    /// Reweighted ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    void point_to_point(Eigen::Matrix3Xd& X,
                        Eigen::Matrix3Xd& Y,
                        Parameters par) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
            }
            /// Computer rotation and translation
            for(int outer=0; outer<par.max_outer; ++outer) {
                /// Compute weights
                W = (X-Q).colwise().norm();
                robust_weight(par.f, W, par.p);
                /// Rotation and translation update
                RigidMotionEstimator::point_to_point(X, Q, W);
                /// Stopping criteria
                double stop1 = (X-Xo1).colwise().norm().maxCoeff();
                Xo1 = X;
                if(stop1 < par.stop) break;
            }
            /// Stopping criteria
            double stop2 = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop2 < par.stop) break;
        }
    }
    
    /// Reweighted ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Matrix4d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Eigen::MatrixBase<Derived3>& N,
						Parameters par = Parameters()) {
                        
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("point_to_plane"));
		viewer->addCoordinateSystem();
		viewer->setBackgroundColor(0.5,0.5,0.5);
	
                        
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd  W  = Eigen::VectorXd::Zero(X.cols());
        //Eigen::Matrix3Xd Xo0 = X;
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                int id = kdtree.closest(X.col(i).data());
                Qp.col(i) = Y.col(id);
                Qn.col(i) = N.col(id);
            }
            
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();
	scloud->points.clear();
	dcloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}		
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();
	
            /// Computer rotation and translation
            for(int outer=0; outer<par.max_outer; ++outer) {
                /// Compute weights
                W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
                robust_weight(par.f, W, par.p);
                /// Rotation and translation update
                Eigen::Affine3d transformation = RigidMotionEstimator::point_to_plane(X, Qp, Qn, W);
                /// Stopping criteria
                double stop1 = (X-Xo1).colwise().norm().maxCoeff();
                Xo1 = X;
                if(stop1 < par.stop) break;
            }
            /// Stopping criteria
            double stop2 = (X-Xo2).colwise().norm().maxCoeff() ;
            Xo2 = X;
            if(stop2 < par.stop) break;
        }
        return Eigen::Matrix4d::Identity();//tr.matrix().inverse();//T;
    }
}
///////////////////////////////////////////////////////////////////////////////
//#endif
