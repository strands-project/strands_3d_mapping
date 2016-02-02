#include "MassRegistrationPPR.h"

#include "ICP.h"

#include <iostream>
#include <fstream>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

#include "g2o/core/factory.h"

//#include "g2o/config.h"
//#include "g2o/core/base_vertex.h"
//#include "g2o/core/hyper_graph_action.h"
//#include "g2o/types/slam3d/isometry3d_mappings.h"
//#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "g2o/core/eigen_types.h"

namespace reglib
{

MassRegistrationPPR::MassRegistrationPPR(bool visualize){
	type					= PointToPlane;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;
	func					= new DistanceWeightFunction2PPR();

	if(visualize){
		visualizationLvl = 1;

		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}else{
		visualizationLvl = 0;
	}

}
MassRegistrationPPR::~MassRegistrationPPR(){}

namespace RigidMotionEstimator3 {
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
								   bool doy) {

		if(!dox && !doy){return Eigen::Affine3d::Identity();}

		typedef Eigen::Matrix<double, 6, 6> Matrix66;
		typedef Eigen::Matrix<double, 6, 1> Vector6;
		typedef Eigen::Block<Matrix66, 3, 3> Block33;

        //Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
        //for(int i=0; i<4; ++i){}

		/// Normalize weight vector
		Eigen::VectorXd w_normalized = w/w.sum();
		/// De-mean
		Eigen::Vector3d X_mean;
		for(int i=0; i<3; ++i){X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();}
		X.colwise() -= X_mean;
		Y.colwise() -= X_mean;
		/// Prepare LHS and RHS


		Matrix66 LHS1 = Matrix66::Zero();
		Vector6 RHS1 = Vector6::Zero();
		if(dox){
			Block33 TL = LHS1.topLeftCorner<3,3>();
			Block33 TR = LHS1.topRightCorner<3,3>();
			Block33 BR = LHS1.bottomRightCorner<3,3>();
			Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
			#pragma omp parallel
			{
				#pragma omp for
				for(int i=0; i<X.cols(); i++) {
					C.col(i) = X.col(i).cross(Yn.col(i));
				}
				#pragma omp sections nowait
				{
					#pragma omp section
					for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
					#pragma omp section
					for(int i=0; i<X.cols(); i++) TR += (C.col(i)*Yn.col(i).transpose())*w(i);
					#pragma omp section
					for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(Yn.col(i), w(i));
					#pragma omp section
					for(int i=0; i<C.cols(); i++) {
						double dist_to_plane = -((X.col(i) - Y.col(i)).dot(Yn.col(i)) - u(i))*w(i);
						RHS1.head<3>() += C.col(i)*dist_to_plane;
						RHS1.tail<3>() += Yn.col(i)*dist_to_plane;
					}
				}
			}
			LHS1 = LHS1.selfadjointView<Eigen::Upper>();
		}


		Matrix66 LHS2 = Matrix66::Zero();
		Vector6 RHS2 = Vector6::Zero();
		if(doy){
			Block33 TL = LHS2.topLeftCorner<3,3>();
			Block33 TR = LHS2.topRightCorner<3,3>();
			Block33 BR = LHS2.bottomRightCorner<3,3>();
			Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,Y.cols());
			#pragma omp parallel
			{
				#pragma omp for
				for(int i=0; i<Y.cols(); i++) {
					C.col(i) = Y.col(i).cross(Xn.col(i));
				}
				#pragma omp sections nowait
				{
					#pragma omp section
					for(int i=0; i<Y.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
					#pragma omp section
					for(int i=0; i<Y.cols(); i++) TR += (C.col(i)*Xn.col(i).transpose())*w(i);
					#pragma omp section
					for(int i=0; i<Y.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(Xn.col(i), w(i));
					#pragma omp section
					for(int i=0; i<C.cols(); i++) {
						double dist_to_plane = -((Y.col(i) - X.col(i)).dot(Xn.col(i)) - u(i))*w(i);
						RHS2.head<3>() += C.col(i)*dist_to_plane;
						RHS2.tail<3>() += Xn.col(i)*dist_to_plane;
					}
				}
			}
			LHS2 = LHS2.selfadjointView<Eigen::Upper>();
		}
/*
		std::cout << LHS1 << std::endl << std::endl;
		std::cout << LHS2 << std::endl << std::endl;
		std::cout << RHS1 << std::endl << std::endl;
		std::cout << -RHS2 << std::endl << std::endl;
*/
		Matrix66 LHS = LHS1 + LHS2;
		Vector6 RHS = RHS1 - RHS2;
		/// Compute transformation
		Eigen::Affine3d transformation;
		Eigen::LDLT<Matrix66> ldlt(LHS);
		RHS = ldlt.solve(RHS);
		transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
						  Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
						  Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
/*
		//BIDIRECTIONAL
		/// Prepare LHS and RHS
		Matrix66 LHS2 = Matrix66::Zero();
		Vector6 RHS2 = Vector6::Zero();
		Block33 TL2 = LHS2.topLeftCorner<3,3>();
		Block33 TR2 = LHS2.topRightCorner<3,3>();
		Block33 BR2 = LHS2.bottomRightCorner<3,3>();
		Eigen::MatrixXd C2 = Eigen::MatrixXd::Zero(3,Y.cols());
		#pragma omp parallel
		{
			#pragma omp for
			for(int i=0; i<Y.cols(); i++) {
				C2.col(i) = Y.col(i).cross(Xn.col(i));
			}
			#pragma omp sections nowait
			{
				#pragma omp section
				for(int i=0; i<Y.cols(); i++) TL2.selfadjointView<Eigen::Upper>().rankUpdate(C2.col(i), w(i));
				#pragma omp section
				for(int i=0; i<Y.cols(); i++) TR2 += (C2.col(i)*Xn.col(i).transpose())*w(i);
				#pragma omp section
				for(int i=0; i<Y.cols(); i++) BR2.selfadjointView<Eigen::Upper>().rankUpdate(Xn.col(i), w(i));
				#pragma omp section
				for(int i=0; i<C2.cols(); i++) {
					double dist_to_plane = -((Y.col(i) - X.col(i)).dot(Xn.col(i)) - u(i))*w(i);
					RHS2.head<3>() += C2.col(i)*dist_to_plane;
					RHS2.tail<3>() += Xn.col(i)*dist_to_plane;
				}
			}
		}
		LHS2 = LHS2.selfadjointView<Eigen::Upper>();
		/// Compute transformation
		Eigen::Affine3d transformation2;
		Eigen::LDLT<Matrix66> ldlt2(LHS2);
		RHS2 = ldlt.solve(RHS2);

		std::cout << LHS << std::endl << std::endl;

		std::cout << LHS2 << std::endl << std::endl;


		std::cout << RHS << std::endl << std::endl;

		std::cout << -RHS2 << std::endl << std::endl;
		exit(0);
*/
		Xn = transformation*Xn;

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
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
										  Eigen::MatrixBase<Derived2>& Xn,
										  Eigen::MatrixBase<Derived3>& Yp,
										  Eigen::MatrixBase<Derived4>& Yn,
										  const Eigen::MatrixBase<Derived5>& w,
										  bool dox = true,
										  bool doy = false) {
		return point_to_plane(X,Xn,Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()),dox,doy);
	}
}


/*

        */
double getError0(){

}

namespace RigidMotionEstimator4 {

template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
double getError0(              Eigen::MatrixBase<Derived1> X,
                               Eigen::MatrixBase<Derived2> Xn,
                               Eigen::MatrixBase<Derived3> Y,
                               Eigen::MatrixBase<Derived4> Yn,
                               const Eigen::MatrixBase<Derived5> w){
    MatrixXd R = Yn.array()*(X-Y).array();
    double R_sum1 = 0;
    int nr_pts = R.cols();
    for(int i = 0; i < nr_pts; i++){float norm = R.col(i).norm();R_sum1 += w(i)*norm*norm;}
    printf("R_sum1: %15.15f\n",R_sum1);
    return R_sum1;
}
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
                                   bool doy) {

        if(!dox && !doy){return Eigen::Affine3d::Identity();}
        double step = 0.00001;

        Eigen::Affine3d xr_transformation;
        xr_transformation  = Eigen::AngleAxisd(step, Eigen::Vector3d::UnitX());

        Eigen::Affine3d yr_transformation;
        yr_transformation  = Eigen::AngleAxisd(step, Eigen::Vector3d::UnitY());

        Eigen::Affine3d zr_transformation;
        zr_transformation  = Eigen::AngleAxisd(step, Eigen::Vector3d::UnitZ());

        double emid = getError0(X,Xn,Y,Yn,w);
        double exr = getError0(xr_transformation*X,xr_transformation*Xn,Y,Yn,w);
        double eyr = getError0(yr_transformation*X,yr_transformation*Xn,Y,Yn,w);
        double ezr = getError0(zr_transformation*X,zr_transformation*Xn,Y,Yn,w);
/*
        Eigen::Affine3d transformation;
        Eigen::LDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());

        Xn = transformation*Xn;

        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
*/
        return Eigen::Affine3d::Identity();
    }

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
                                          bool doy = false) {
        return point_to_plane(X,Xn,Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()),dox,doy);
    }
}


using namespace g2o;

MassFusionResults MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses){
	printf("start MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses)\n");
//testMinimizer();
	unsigned int nr_frames = frames.size();
	if(poses.size() != nr_frames){
		printf("ERROR: poses.size() != nr_frames\n");
		return MassFusionResults();
	}

	nr_matches.resize(nr_frames);
	matchids.resize(nr_frames);
	nr_datas.resize(nr_frames);
	points.resize(nr_frames);
	colors.resize(nr_frames);
	normals.resize(nr_frames);
	transformed_points.resize(nr_frames);
	transformed_normals.resize(nr_frames);
/*
boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("3D Viewer"));
view->setBackgroundColor (255, 255, 255);
view->addCoordinateSystem (1.0);
view->initCameraParameters ();
*/
	for(unsigned int i = 0; i < nr_frames; i++){
		//printf("start local : data loaded successfully\n");
		printf("loading data for %i\n",i);

		unsigned char  * maskdata		= (unsigned char	*)(masks[i].data);
		unsigned char  * rgbdata		= (unsigned char	*)(frames[i]->rgb.data);
		unsigned short * depthdata		= (unsigned short	*)(frames[i]->depth.data);
		float		   * normalsdata	= (float			*)(frames[i]->normals.data);

		Eigen::Matrix4d p = poses[i];
		float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
		float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
		float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

		Camera * camera				= frames[i]->camera;
		const unsigned int width	= camera->width;
		const unsigned int height	= camera->height;
		const float idepth			= camera->idepth_scale;
		const float cx				= camera->cx;
		const float cy				= camera->cy;
		const float ifx				= 1.0/camera->fx;
		const float ify				= 1.0/camera->fy;

		int count = 0;

		//for(unsigned int ind = 0; ind < width*height; ind++){count += (maskdata[ind] == 255);}
		const unsigned steps = 4;
		for(unsigned int w = 0; w < width; w+=steps){
			for(unsigned int h = 0; h < height;h+=steps){
				int ind = h*width+w;
				if(maskdata[ind] == 255){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];

					if(z > 0.2 && xn != 2){
						count++;
					}
				}
			}
		}

		nr_datas[i] = count;
		matchids[i].resize(nr_frames);
		points[i].resize(Eigen::NoChange,count);
		colors[i].resize(Eigen::NoChange,count);
		normals[i].resize(Eigen::NoChange,count);
		transformed_points[i].resize(Eigen::NoChange,count);
		transformed_normals[i].resize(Eigen::NoChange,count);


		Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & C	= colors[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= normals[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= transformed_points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= transformed_normals[i];
		Eigen::VectorXd information (count);

		int c = 0;
		for(unsigned int w = 0; w < width; w+=steps){
			for(unsigned int h = 0; h < height;h+=steps){
				int ind = h*width+w;
				if(maskdata[ind] == 255){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];

					if(z > 0.2 && xn != 2){
						float yn = normalsdata[3*ind+1];
						float zn = normalsdata[3*ind+2];

						float x = (w - cx) * z * ifx;
						float y = (h - cy) * z * ify;

						X(0,c)	= x;
						X(1,c)	= y;
						X(2,c)	= z;
						Xn(0,c)	= xn;
						Xn(1,c)	= yn;
						Xn(2,c)	= zn;

						tX(0,c)		= m00*x + m01*y + m02*z + m03;
						tX(1,c)		= m10*x + m11*y + m12*z + m13;
						tX(2,c)		= m20*x + m21*y + m22*z + m23;
						tXn(0,c)	= m00*xn + m01*yn + m02*zn;
						tXn(1,c)	= m10*xn + m11*yn + m12*zn;
						tXn(2,c)	= m20*xn + m21*yn + m22*zn;

						information(c) = 1.0/(z*z);

						//if(c%100 == 0){printf("information(i): %15.15f\n",information(c));}

						C(0,c) = rgbdata[3*ind+0];
						C(1,c) = rgbdata[3*ind+1];
						C(2,c) = rgbdata[3*ind+2];
						c++;
					}
				}
			}
		}
		informations.push_back(information);
		trees.push_back(new nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>(X));
/*
		printf("--------------------------------\n");
		for(unsigned int i = 0; i < count; i++){if(i%100 == 0){printf("X: %f %f %f\n",X(0,i),X(1,i),X(2,i));}}
		for(unsigned int i = 0; i < count; i++){if(i%100 == 0){printf("tX: %f %f %f\n",tX(0,i),tX(1,i),tX(2,i));}}
		printf("--------------------------------\n");

		char buf [1024];
		sprintf(buf,"cloud%i",i);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		cloud->points.clear();
		for(unsigned int c = 0; c < count; c++){
			pcl::PointXYZRGBNormal p;
			p.x = X(0,c);
			p.y = X(1,c);
			p.z = X(2,c);
			p.b = 0;
			p.g = 255;
			p.r = 0;
			cloud->points.push_back(p);
		}
		view->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf );
		view->spin();

		printf("end local : data loaded successfully\n");
*/
	}
	printf("data loaded successfully\n");
	func->reset();
	func->regularization = 0.01;

    Eigen::MatrixXd Xo1;

	int imgcount = 0;

	for(int funcupdate=0; funcupdate < 50; ++funcupdate) {
		printf("funcupdate: %i\n",funcupdate);
		for(int rematching=0; rematching < 3; ++rematching) {
			printf("funcupdate: %i rematching: %i\n",funcupdate,rematching);
			
			if(visualizationLvl > 0){
				std::vector<Eigen::MatrixXd> Xv;
				for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
				char buf [1024];
				sprintf(buf,"image%5.5i.png",imgcount++);
				show(Xv,true,std::string(buf),imgcount);
			}
			
			for(unsigned int i = 0; i < nr_frames; i++){
				nr_matches[i] = 0;
				for(unsigned int j = 0; j < nr_frames; j++){
					if(i == j){continue;}
					Eigen::Affine3d relative_pose = Eigen::Affine3d(poses[j].inverse()*poses[i]);

					Eigen::Matrix<double, 3, Eigen::Dynamic> tX	= relative_pose*points[i];
					nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * tree = trees[j];

					unsigned int nr_data = nr_datas[i];
					std::vector<int> & matchid = matchids[i][j];
					matchid.resize(nr_data);
					for(unsigned int k = 0; k < nr_data; ++k) {
						matchid[k] = tree->closest(tX.col(k).data());
					}
					nr_matches[i] += matchid.size();
				}
			}

			for(unsigned int i = 0; i < nr_frames; i++){
				nr_matches[i] = 0;
				for(unsigned int j = 0; j < nr_frames; j++){
					nr_matches[i] += matchids[i][j].size()+matchids[j][i].size();
				}
			}

			int total_matches = 0;
			for(unsigned int i = 0; i < nr_frames; i++){
				for(unsigned int j = 0; j < nr_frames; j++){
					total_matches += matchids[i][j].size();
				}
			}

			printf("total_matches: %i\n",total_matches);

			Eigen::MatrixXd all_residuals = Eigen::Matrix3Xd::Zero(3,total_matches);
			int count = 0;

			for(unsigned int i = 0; i < nr_frames; i++){
				Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
				Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
				Eigen::VectorXd & informationi					= informations[i];

				for(unsigned int j = 0; j < nr_frames; j++){
					if(i == j){continue;}
					std::vector<int> & matchidi = matchids[i][j];
					unsigned int matchesi = matchidi.size();

					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
					Eigen::VectorXd & informationj					= informations[j];

					Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
					Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	matchesi);

					Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
					Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	matchesi);
					Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	matchesi);

					for(unsigned int ki = 0; ki < matchesi; ki++){
						int kj = matchidi[ki];

						Qp.col(ki) = tXj.col(kj);
						Qn.col(ki) = tXnj.col(kj);

						Xp.col(ki) = tXi.col(ki);
						Xn.col(ki) = tXni.col(ki);
						rangeW(ki) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
					}

					Eigen::MatrixXd residuals;
					switch(type) {
						case PointToPoint:	{residuals = Xp-Qp;} 						break;
						case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
						default:			{printf("type not set\n");}					break;
					}

					for(unsigned int k=0; k < matchesi; ++k) {residuals.col(k) *= rangeW(k);}

					all_residuals.block(0,count,residuals.rows(),residuals.cols()) = residuals;
					count += residuals.cols();
				}
			}

			func->debugg_print = true;
			switch(type) {
				case PointToPoint:	{func->computeModel(all_residuals); 				} 	break;
				case PointToPlane:	{func->computeModel(all_residuals.colwise().norm());}	break;
				default:  			{printf("type not set\n");} break;
			}
			func->debugg_print = false;



			for(int outer=0; outer < 10; ++outer) {
				printf("funcupdate: %i rematching: %i outer: %i\n",funcupdate,rematching,outer);
				for(unsigned int i = 0; i < nr_frames; i++){
					//printf("funcupdate: %i rematching: %i outer: %i frame: %i\n",funcupdate,rematching,outer,i);

					unsigned int nr_match = 0;
					for(unsigned int j = 0; j < nr_frames; j++){
						std::vector<int> & matchidj = matchids[j][i];
						unsigned int matchesj = matchidj.size();
						std::vector<int> & matchidi = matchids[i][j];
						unsigned int matchesi = matchidi.size();

						for(unsigned int ki = 0; ki < matchesi; ki++){
							int kj = matchidi[ki];
							if(matchidj[kj] == ki){nr_match++;}
						}
					}

					Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
					Eigen::Matrix3Xd Xp_ori	= Eigen::Matrix3Xd::Zero(3,	nr_match);
					Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	nr_match);

					Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
					Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	nr_match);
					Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	nr_match);

					int count = 0;

					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & Xi	= points[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & Xni	= normals[i];
					Eigen::VectorXd & informationi					= informations[i];

					for(unsigned int j = 0; j < nr_frames; j++){
						Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
						Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
						Eigen::VectorXd & informationj					= informations[j];

						std::vector<int> & matchidj = matchids[j][i];
						unsigned int matchesj = matchidj.size();
						std::vector<int> & matchidi = matchids[i][j];
						unsigned int matchesi = matchidi.size();

						for(unsigned int ki = 0; ki < matchesi; ki++){
							int kj = matchidi[ki];
							if(matchidj[kj] == ki){
								Qp.col(count) = tXj.col(kj);
								Qn.col(count) = tXnj.col(kj);

								Xp_ori.col(count) = Xi.col(ki);
								Xp.col(count) = tXi.col(ki);

								Xn.col(count) = tXni.col(ki);
								rangeW(count) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
								count++;
							}
						}
					}
/*
					Eigen::MatrixXd residuals;
					switch(type) {
						case PointToPoint:	{residuals = Xp-Qp;} 						break;
						case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
						default:			{printf("type not set\n");}					break;
					}
					for(unsigned int k=0; k < nr_match; ++k) {residuals.col(k) *= rangeW(k);}
					switch(type) {
						case PointToPoint:	{func->computeModel(residuals); 				} 	break;
						case PointToPlane:	{func->computeModel(residuals.colwise().norm());}	break;
						default:  			{printf("type not set\n");} break;
					}

*/
					for(int inner=0; inner < 5; ++inner) {
						/*
						if(inner != 0){
							switch(type) {
								case PointToPoint:	{residuals = Xp-Qp;} 						break;
								case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
								default:			{printf("type not set\n");}					break;
							}
							for(int k=0; k<nr_match; ++k) {residuals.col(k) *= rangeW(k);}
						}
						*/
						Eigen::MatrixXd residuals;
						switch(type) {
							case PointToPoint:	{residuals = Xp-Qp;} 						break;
							case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
							default:			{printf("type not set\n");}					break;
						}
						for(unsigned int k=0; k < nr_match; ++k) {residuals.col(k) *= rangeW(k);}

						Eigen::VectorXd  W;

						switch(type) {
							case PointToPoint:	{W = func->getProbs(residuals); } 					break;
							case PointToPlane:	{
								W = func->getProbs(residuals.colwise().norm());
								for(int k=0; k<nr_match; ++k) {W(k) = W(k)*float((Xn(0,k)*Qn(0,k) + Xn(1,k)*Qn(1,k) + Xn(2,k)*Qn(2,k)) > 0.0);}
							}	break;
							default:			{printf("type not set\n");} break;
						}

						W = W.array()*rangeW.array()*rangeW.array();
                        Xo1 = Xp;
						switch(type) {
							case PointToPoint:	{RigidMotionEstimator::point_to_point(Xp, Qp, W);}		break;
							case PointToPlane:	{RigidMotionEstimator3::point_to_plane(Xp, Xn, Qp, Qn, W);}	break;
							default:  			{printf("type not set\n"); } break;
						}

                        double stop1 = (Xp-Xo1).colwise().norm().maxCoeff();
                        Xo1 = Xp;
						//printf("funcupdate: %i rematching: %i outer: %i frame: %i inner: %i ",funcupdate,rematching,outer,i,inner);
						//printf("stop1: %10.10f\n",stop1);

						//if(stop1 < stop) break;
					}

					pcl::TransformationFromCorrespondences tfc;
					for(unsigned int c = 0; c < nr_match; c++){
						Eigen::Vector3f a (Xp(0,c),				Xp(1,c),			Xp(2,c));
                        Eigen::Vector3f b (Xp_ori(0,c),         Xp_ori(1,c),        Xp_ori(2,c));
						tfc.add(b,a);
					}

					poses[i] = tfc.getTransformation().cast<double>().matrix();
					Eigen::Matrix4d p = poses[i];
					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

                    for(int c = 0; c < Xi.cols(); c++){
						float x = Xi(0,c);
						float y = Xi(1,c);
						float z = Xi(2,c);

						float nx = Xni(0,c);
						float ny = Xni(1,c);
						float nz = Xni(2,c);

						tXi(0,c)		= m00*x + m01*y + m02*z + m03;
						tXi(1,c)		= m10*x + m11*y + m12*z + m13;
						tXi(2,c)		= m20*x + m21*y + m22*z + m23;

						tXni(0,c)		= m00*nx + m01*ny + m02*nz;
						tXni(1,c)		= m10*nx + m11*ny + m12*nz;
						tXni(2,c)		= m20*nx + m21*ny + m22*nz;
					}
				}

				Eigen::Matrix4d p0inv = poses[0].inverse();
				for(unsigned int j = 0; j < nr_frames; j++){
					poses[j] = p0inv*poses[j];

					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[j];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & Xi	= points[j];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[j];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & Xni	= normals[j];

					Eigen::Matrix4d p = poses[j];
					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

					for(int c = 0; c < Xi.cols(); c++){
						float x = Xi(0,c);
						float y = Xi(1,c);
						float z = Xi(2,c);

						float nx = Xni(0,c);
						float ny = Xni(1,c);
						float nz = Xni(2,c);

						tXi(0,c)		= m00*x + m01*y + m02*z + m03;
						tXi(1,c)		= m10*x + m11*y + m12*z + m13;
						tXi(2,c)		= m20*x + m21*y + m22*z + m23;

						tXni(0,c)		= m00*nx + m01*ny + m02*nz;
						tXni(1,c)		= m10*nx + m11*ny + m12*nz;
						tXni(2,c)		= m20*nx + m21*ny + m22*nz;
					}
				}
			}		
		}

		//std::vector<Eigen::MatrixXd> Xv;
		//for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
		//char buf [1024];
		//sprintf(buf,"image%5.5i.png",imgcount++);
		//show(Xv,true,std::string(buf),imgcount);

		func->debugg_print = true;
		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		func->debugg_print = false;
		printf("before: %5.5f after: %5.5f relative size: %5.5f\n",noise_before,noise_after,noise_after/noise_before);
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}



	printf("stop MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> guess)\n");
	//exit(0);
	return MassFusionResults(poses,-1);
}

/*

FusionResults RegistrationGOICP::getTransform(Eigen::MatrixXd guess){
	double stop		= 0.00001;
	/// Buffers

	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	X.cols());

	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;
	Eigen::Matrix3Xd Xo4 = X;
	Eigen::VectorXd angles;

	std::vector<int> matchid;
	matchid.resize(s_nr_data);

	std::vector<double> total_dweight;
	total_dweight.resize(d_nr_data);
	//if(visualizationLvl >= 1){show(X,Xn,Y,N);}

	/// ICP
	for(int funcupdate=0; funcupdate < 50; ++funcupdate) {
		for(int rematching=0; rematching < 1; ++rematching) {
			//printf("funcupdate: %i rematching: %i\n",funcupdate,rematching);

			#pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {matchid[i] = kdtree.closest(X.col(i).data());}

			/// Find closest point
			#pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {
				int id = matchid[i];
				Qn.col(i) = N.col(id);
				Qp.col(i) = Y.col(id);
				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			}

			if(visualizationLvl >= 2){show(X,Xn,Y,N);}

			for(int outer=0; outer< 1; ++outer) {

				for(int inner=0; inner< 1; ++inner) {
					//printf("icp: %i outer: %i inner: %i ",icp,outer,inner);
					if(inner != 0){
						switch(type) {
							case PointToPoint:	{residuals = X-Qp;} 						break;
							case PointToPlane:	{residuals	= Qn.array()*(X-Qp).array();}	break;
							default:			{printf("type not set\n");}					break;
						}
						for(int i=0; i<X.cols(); ++i) {residuals.col(i) *= rangeW(i);}
					}

					switch(type) {
						case PointToPoint:	{W = func->getProbs(residuals); } 					break;
						case PointToPlane:	{
							W = func->getProbs(residuals.colwise().norm());
							for(int i=0; i<X.cols(); ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
						}	break;
						default:			{printf("type not set\n");} break;
					}

					Wold = W;

					float score1 = Wold.sum()/(pow(func->getNoise(),2)*float(s_nr_data));
					//printf("sum: %f noise: %f score: %f\n",Wold.sum(),func->getNoise(),score1);

					//Normalizing weights has an effect simmilar to one to one matching
					//in that it reduces the effect of border points
					if(normalize_matchweights){
						for(unsigned int i=0; i < d_nr_data; ++i) {	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
						for(unsigned int i=0; i< s_nr_data; ++i) {	total_dweight[matchid[i]] += W(i);}
						for(unsigned int i=0; i< s_nr_data; ++i) {	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
					}
					//W = fwt.array()*W.array();
					//if(visualizationLvl >= 3){show(X,Qp,W);}
					W = W.array()*rangeW.array()*rangeW.array();

					switch(type) {
						case PointToPoint:	{RigidMotionEstimator::point_to_point(X, Qp, W);}		break;
						case PointToPlane:	{RigidMotionEstimator2::point_to_plane(X, Xn, Qp, Qn, W);}	break;
						default:  			{printf("type not set\n"); } break;
					}

					double stop1 = (X-Xo1).colwise().norm().maxCoeff();
					Xo1 = X;
					if(stop1 < stop) break;
				}
				double stop2 = (X-Xo2).colwise().norm().maxCoeff();
				Xo2 = X;
				if(stop2 < stop) break;
			}
			//func->update();
			//for(unsigned int f = 0; f < nr_features; f++){feature_func[f]->update();}
			//if(visualizationLvl >= 3){show(X,Y);}
			double stop3 = (X-Xo3).colwise().norm().maxCoeff();
			Xo3 = X;
			if(stop3 < stop) break;
		}

		func->update();

		double stop4 = (X-Xo4).colwise().norm().maxCoeff();
		Xo4 = X;
		if(stop4 < stop && funcupdate > 25) break;
	}

	float score = Wold.sum()*pow(func->getNoise(),-1)/float(s_nr_data);
	//printf("sum: %f noise: %f score: %f\n",Wold.sum(),func->getNoise(),score);

	pcl::TransformationFromCorrespondences tfc;
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	Eigen::Matrix4d np = tfc.getTransformation().matrix().cast<double>();

	//show(X,Xn,Y,N);
	//return FusionResults(best_transform,best_score);

	//clock_t clockEnd3 = clock();
	//std::cout << (double)(clockEnd3 - clockBegin3)/CLOCKS_PER_SEC << "s (CPU)" << std::endl;

	return FusionResults(np,score);
}
*/
}
