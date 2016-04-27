#include "registration/MassRegistrationPPR.h"

#include "registration/ICP.h"

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reglib
{

MassRegistrationPPR::MassRegistrationPPR(double startreg, bool visualize){
	type					= PointToPlane;
	//type					= PointToPoint;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;

	DistanceWeightFunction2PPR2 * dwf = new DistanceWeightFunction2PPR2();
	dwf->update_size		= true;
	dwf->startreg			= startreg;
	dwf->debugg_print		= false;
	func					= dwf;
	
	fast_opt				= false;

	nomask = true;
	maskstep = 1;
	nomaskstep = 100000;

	stopval = 0.001;
	steps = 4;

	timeout = 6000;

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

	Matrix66 LHS = LHS1 + LHS2;
	Vector6 RHS = RHS1 - RHS2;
	/// Compute transformation
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

double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

bool isconverged(std::vector<Eigen::Matrix4d> before, std::vector<Eigen::Matrix4d> after, double stopvalr = 0.001, double stopvalt = 0.001){
	double change_trans = 0;
	double change_rot = 0;
	unsigned int nr_frames = after.size();
	for(unsigned int i = 0; i < nr_frames; i++){
		for(unsigned int j = i+1; j < nr_frames; j++){
			Eigen::Matrix4d diff_before = after[i].inverse()*after[j];
			Eigen::Matrix4d diff_after	= before[i].inverse()*before[j];
			Eigen::Matrix4d diff = diff_before.inverse()*diff_after;
			double dt = 0;
			for(unsigned int k = 0; k < 3; k++){
				dt += diff(k,3)*diff(k,3);
				for(unsigned int l = 0; l < 3; l++){
					if(k == l){ change_rot += fabs(1-diff(k,l));}
					else{		change_rot += fabs(diff(k,l));}
				}
			}
			change_trans += sqrt(dt);
		}
	}

	change_trans /= double(nr_frames*(nr_frames-1));
	change_rot	 /= double(nr_frames*(nr_frames-1));

	if(change_trans < stopvalt && change_rot < stopvalr){return true;}
	else{return false;}
}

Eigen::Matrix4d constructTransformationMatrix (const double & alpha, const double & beta, const double & gamma, const double & tx,    const double & ty,   const double & tz){
	// Construct the transformation matrix from rotation and translation
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero ();
	transformation_matrix (0, 0) =  cos (gamma) * cos (beta);
	transformation_matrix (0, 1) = -sin (gamma) * cos (alpha) + cos (gamma) * sin (beta) * sin (alpha);
	transformation_matrix (0, 2) =  sin (gamma) * sin (alpha) + cos (gamma) * sin (beta) * cos (alpha);
	transformation_matrix (1, 0) =  sin (gamma) * cos (beta);
	transformation_matrix (1, 1) =  cos (gamma) * cos (alpha) + sin (gamma) * sin (beta) * sin (alpha);
	transformation_matrix (1, 2) = -cos (gamma) * sin (alpha) + sin (gamma) * sin (beta) * cos (alpha);
	transformation_matrix (2, 0) = -sin (beta);
	transformation_matrix (2, 1) =  cos (beta) * sin (alpha);
	transformation_matrix (2, 2) =  cos (beta) * cos (alpha);

	transformation_matrix (0, 3) = tx;
	transformation_matrix (1, 3) = ty;
	transformation_matrix (2, 3) = tz;
	transformation_matrix (3, 3) = 1;
	return transformation_matrix;
}

MassFusionResults MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses){
	printf("start MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses)\n");

	unsigned int nr_frames = frames.size();
	if(poses.size() != nr_frames){
		printf("ERROR: poses.size() != nr_frames\n");
		return MassFusionResults();
	}

	if(fast_opt){
		printf("debugging... setting nr frames to 3: %s :: %i\n",__FILE__,__LINE__);
		nr_frames = 3;
	}

	std::vector<Eigen::Matrix4d> poses1; poses1.resize(poses.size());
	std::vector<Eigen::Matrix4d> poses2; poses2.resize(poses.size());
	std::vector<Eigen::Matrix4d> poses2b; poses2b.resize(poses.size());
	std::vector<Eigen::Matrix4d> poses3; poses3.resize(poses.size());
	std::vector<Eigen::Matrix4d> poses4; poses4.resize(poses.size());

	nr_matches.resize(nr_frames);
	matchids.resize(nr_frames);
	nr_datas.resize(nr_frames);
	points.resize(nr_frames);
	colors.resize(nr_frames);
	normals.resize(nr_frames);
	transformed_points.resize(nr_frames);
	transformed_normals.resize(nr_frames);
	informations.resize(nr_frames);

	for(unsigned int i = 0; i < nr_frames; i++){
		printf("loading data for %i\n",i);
		bool * maskvec		= mmasks[i]->maskvec;
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
		for(unsigned int w = 0; w < width; w+=maskstep){
			for(unsigned int h = 0; h < height; h+=maskstep){
				int ind = h*width+w;
				if(maskvec[ind]){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];
					if(z > 0.2 && xn != 2){count++;}
				}
			}
		}

		if(count < 10){
			is_ok.push_back(false);
			continue;
		}else{
			is_ok.push_back(true);
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
		for(unsigned int w = 0; w < width; w+=maskstep){
			for(unsigned int h = 0; h < height;h+=maskstep){
				if(c == count){continue;}
				int ind = h*width+w;
				if(maskvec[ind]){
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
						C(0,c) = rgbdata[3*ind+0];
						C(1,c) = rgbdata[3*ind+1];
						C(2,c) = rgbdata[3*ind+2];
						c++;
					}
				}
			}
		}
		informations[i] = information;
		printf("%i %i\n",X.rows(),X.cols());

		cv::namedWindow( "rgb", cv::WINDOW_AUTOSIZE );			cv::imshow( "rgb", frames[i]->rgb);
		cv::namedWindow( "normals", cv::WINDOW_AUTOSIZE );		cv::imshow( "normals", frames[i]->normals);
		cv::namedWindow( "depth", cv::WINDOW_AUTOSIZE );		cv::imshow( "depth", frames[i]->depth);
		cv::namedWindow( "depthedges", cv::WINDOW_AUTOSIZE );	cv::imshow( "depthedges", frames[i]->depthedges);
		cv::namedWindow( "mask", cv::WINDOW_AUTOSIZE );			cv::imshow( "mask", mmasks[i]->getMask());
		cv::waitKey(0);

		//trees.push_back(new nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>(X));
	}

	//	background_nr_matches.resize(nr_frames);
	//	background_matchids.resize(nr_frames);
	//	background_nr_datas.resize(nr_frames);
	//	background_points.resize(nr_frames);
	//	background_colors.resize(nr_frames);
	//	background_normals.resize(nr_frames);
	//	background_transformed_points.resize(nr_frames);
	//	background_transformed_normals.resize(nr_frames);
	//	background_informations.resize(nr_frames);

	//	for(unsigned int i = 0; i < nr_frames; i++){
	//		//printf("start local : data loaded successfully\n");
	//		printf("background loading data for %i\n",i);

	//		bool * maskvec		= mmasks[i]->maskvec;
	//		unsigned char  * rgbdata		= (unsigned char	*)(frames[i]->rgb.data);
	//		unsigned short * depthdata		= (unsigned short	*)(frames[i]->depth.data);
	//		float		   * normalsdata	= (float			*)(frames[i]->normals.data);

	//		Eigen::Matrix4d p = poses[i];
	//		float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	//		float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	//		float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	//		Camera * camera				= frames[i]->camera;
	//		const unsigned int width	= camera->width;
	//		const unsigned int height	= camera->height;
	//		const float idepth			= camera->idepth_scale;
	//		const float cx				= camera->cx;
	//		const float cy				= camera->cy;
	//		const float ifx				= 1.0/camera->fx;
	//		const float ify				= 1.0/camera->fy;

	//		int count = 0;
	//		for(unsigned int w = 0; w < width; w+=nomaskstep){
	//			for(unsigned int h = 0; h < height; h+=nomaskstep){
	//				int ind = h*width+w;
	//				if(maskvec[ind]){
	//					float z = idepth*float(depthdata[ind]);
	//					float xn = normalsdata[3*ind+0];
	//					if(z > 0.2 && xn != 2){count++;}
	//				}
	//			}
	//		}

	//		background_nr_datas[i] = count;
	//		background_matchids[i].resize(nr_frames);
	//		background_points[i].resize(Eigen::NoChange,count);
	//		background_colors[i].resize(Eigen::NoChange,count);
	//		background_normals[i].resize(Eigen::NoChange,count);
	//		background_transformed_points[i].resize(Eigen::NoChange,count);
	//		background_transformed_normals[i].resize(Eigen::NoChange,count);


	//		Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= background_points[i];
	//		Eigen::Matrix<double, 3, Eigen::Dynamic> & C	= background_colors[i];
	//		Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= background_normals[i];
	//		Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= background_transformed_points[i];
	//		Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= background_transformed_normals[i];
	//		Eigen::VectorXd information (count);

	//		int c = 0;
	//		for(unsigned int w = 0; w < width; w+=nomaskstep){
	//			for(unsigned int h = 0; h < height; h+=nomaskstep){
	//				if(c == count){continue;}
	//				int ind = h*width+w;
	//				if(maskvec[ind]){
	//					float z = idepth*float(depthdata[ind]);
	//					float xn = normalsdata[3*ind+0];

	//					if(z > 0.2 && xn != 2){
	//						float yn = normalsdata[3*ind+1];
	//						float zn = normalsdata[3*ind+2];

	//						float x = (w - cx) * z * ifx;
	//						float y = (h - cy) * z * ify;

	//						X(0,c)	= x;
	//						X(1,c)	= y;
	//						X(2,c)	= z;
	//						Xn(0,c)	= xn;
	//						Xn(1,c)	= yn;
	//						Xn(2,c)	= zn;

	//						tX(0,c)		= m00*x + m01*y + m02*z + m03;
	//						tX(1,c)		= m10*x + m11*y + m12*z + m13;
	//						tX(2,c)		= m20*x + m21*y + m22*z + m23;
	//						tXn(0,c)	= m00*xn + m01*yn + m02*zn;
	//						tXn(1,c)	= m10*xn + m11*yn + m12*zn;
	//						tXn(2,c)	= m20*xn + m21*yn + m22*zn;

	//						//printf("c: %i count: %i\n",c,count);
	//						information(c) = 1.0/(z*z);

	//						C(0,c) = rgbdata[3*ind+0];
	//						C(1,c) = rgbdata[3*ind+1];
	//						C(2,c) = rgbdata[3*ind+2];
	//						c++;
	//					}
	//				}
	//			}
	//		}
	//		background_informations[i] = information;
	//		background_trees.push_back(new nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>(X));
	//	}


	func->reset();

	Eigen::MatrixXd Xo1;

	int imgcount = 0;

	double good_rematches = 0;
	double total_rematches = 0;

	double good_opt = 0;
	double bad_opt = 0;

	double rematch_time = 0;
	double residuals_time = 0;
	double computeModel_time = 0;
	double setup_matches_time = 0;
	double setup_equation_time = 0;
	double setup_equation_time2 = 0;
	double solve_equation_time = 0;
	double total_time_start = getTime();

	int savecounter = 0;

	bool onetoone = true;

	char buf [1024];
	if(visualizationLvl > 0){
		std::vector<Eigen::MatrixXd> Xv;
		for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
		sprintf(buf,"image%5.5i.png",imgcount++);
		show(Xv,false,std::string(buf),imgcount);
	}

	for(int funcupdate=0; funcupdate < 100; ++funcupdate) {
		if(getTime()-total_time_start > timeout){break;}
		if(visualizationLvl == 2){
			std::vector<Eigen::MatrixXd> Xv;
			for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
			sprintf(buf,"image%5.5i.png",imgcount++);
			show(Xv,false,std::string(buf),imgcount);
		}

		for(int rematching=0; rematching < 10; ++rematching) {
			if(visualizationLvl == 3){
				std::vector<Eigen::MatrixXd> Xv;
				for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
				sprintf(buf,"image%5.5i.png",imgcount++);
				show(Xv,false,std::string(buf),imgcount);
			}

			for(unsigned int i = 0; i < nr_frames; i++){poses1[i] = poses[i];}

			double rematch_time_start = getTime();
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				nr_matches[i] = 0;

				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					if(i == j){continue;}
					Eigen::Affine3d rp = Eigen::Affine3d(poses[j].inverse()*poses[i]);
					Eigen::Affine3d irp = rp.inverse();

					Eigen::Matrix<double, 3, Eigen::Dynamic> tX	= rp*points[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> tY	= irp*points[j];

					nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * treex = trees[j];


					unsigned int nr_data = nr_datas[i];
					std::vector<int> & matchid = matchids[i][j];
					matchid.resize(nr_data);
					for(unsigned int k = 0; k < nr_data; ++k) {
						int prev = matchid[k];
						int current =  treex->closest(tX.col(k).data());
						good_rematches += prev != current;
						total_rematches++;
						matchid[k] = current;
					}
					nr_matches[i] += matchid.size();
				}
			}
			rematch_time += getTime()-rematch_time_start;
			printf("rematch_time: %f\n",rematch_time);

			printf("percentage: %5.5f (good_rematches: %f total_rematches: %f)\n",good_rematches/total_rematches,good_rematches,total_rematches);

			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				nr_matches[i] = 0;
				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					nr_matches[i] += matchids[i][j].size()+matchids[j][i].size();
				}
			}

			int total_matches = 0;
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					total_matches += matchids[i][j].size();
				}
			}

			for(int lala = 0; lala < 1; lala++){
				if(visualizationLvl == 4){
					std::vector<Eigen::MatrixXd> Xv;
					for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
					sprintf(buf,"image%5.5i.png",imgcount++);
					show(Xv,false,std::string(buf),imgcount);

				}
				for(unsigned int i = 0; i < nr_frames; i++){poses2b[i] = poses[i];}

				double residuals_time_start = getTime();
				Eigen::MatrixXd all_residuals;
				switch(type) {
				case PointToPoint:	{all_residuals = Eigen::Matrix3Xd::Zero(3,total_matches);}break;
				case PointToPlane:	{all_residuals = Eigen::MatrixXd::Zero(1,total_matches);}break;
				default:			{printf("type not set\n");}					break;
				}

				int count = 0;
				for(unsigned int i = 0; i < nr_frames; i++){
					if(!is_ok[i]){continue;}
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
					Eigen::VectorXd & informationi					= informations[i];
					for(unsigned int j = 0; j < nr_frames; j++){
						if(!is_ok[j]){continue;}
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
							if( ki >= Qp.cols() || kj < 0 || kj >= tXj.cols() ){continue;}
							Qp.col(ki) = tXj.col(kj);
							Qn.col(ki) = tXnj.col(kj);
							Xp.col(ki) = tXi.col(ki);
							Xn.col(ki) = tXni.col(ki);
							rangeW(ki) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
						}
						Eigen::MatrixXd residuals;
						switch(type) {
						case PointToPoint:	{residuals = Xp-Qp;} 						break;
						case PointToPlane:	{
							residuals		= Eigen::MatrixXd::Zero(1,	Xp.cols());
							for(int i=0; i<Xp.cols(); ++i) {
								float dx = Xp(0,i)-Qp(0,i);
								float dy = Xp(1,i)-Qp(1,i);
								float dz = Xp(2,i)-Qp(2,i);
								float qx = Qn(0,i);
								float qy = Qn(1,i);
								float qz = Qn(2,i);
								float di = qx*dx + qy*dy + qz*dz;
								residuals(0,i) = di;
							}
						}break;
						default:			{printf("type not set\n");}					break;
						}
						for(unsigned int k=0; k < matchesi; ++k) {residuals.col(k) *= rangeW(k);}
						all_residuals.block(0,count,residuals.rows(),residuals.cols()) = residuals;
						count += residuals.cols();
					}
				}
				residuals_time += getTime()-residuals_time_start;

				double computeModel_time_start = getTime();
				func->computeModel(all_residuals);
				computeModel_time += getTime()-computeModel_time_start;
				
				if(fast_opt){
					double setup_matches_time_start = getTime();

					std::vector< Eigen::Matrix4d > localposes = poses;
					std::vector< std::vector < std::vector< std::pair<int,int	> > > > current_matches;
					std::vector< std::vector < std::vector<			double		  > > > rangeW;

					current_matches.resize(nr_frames);
					rangeW.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						current_matches[i].resize(nr_frames);
						rangeW[i].resize(nr_frames);
						if(!is_ok[i]){continue;}

						Eigen::VectorXd & informationi					= informations[i];
						for(unsigned int j = 0; j < nr_frames; j++){
							if(!is_ok[j]){continue;}
							if(i == j){continue;}
							Eigen::VectorXd & informationj					= informations[j];

							std::vector<int> & matchidj = matchids[j][i];
							unsigned int matchesj = matchidj.size();
							std::vector<int> & matchidi = matchids[i][j];
							unsigned int matchesi = matchidi.size();

							std::vector<std::pair<int,int> > & cm = current_matches[i][j];
							std::vector<double > & rw = rangeW[i][j];

							for(unsigned int ki = 0; ki < matchesi; ki++){
								int kj = matchidi[ki];
								if( kj < 0 || kj >=  matchesj){continue;} //Make sure that failed searches dont screw things up
								if(matchidj[kj] != ki){continue;}//Only 1-to-1 matching

								cm.push_back(std::make_pair(ki,kj));
								rw.push_back(1.0/(1.0/informationi(ki)+1.0/informationj(kj)));
							}
						}
					}

					setup_matches_time += getTime()-setup_matches_time_start;

					typedef Eigen::Matrix<double, 6, 1> Vector6d;
					typedef Eigen::Matrix<double, 6, 6> Matrix6d;

					std::vector<std::vector<Matrix6d>> A;
					std::vector<std::vector<Vector6d>> b;
					A.resize(nr_frames);
					b.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						A[i].resize(nr_frames);
						b[i].resize(nr_frames);
					}

					std::vector<std::vector<Matrix6d>> A2;
					std::vector<std::vector<Vector6d>> b2;
					A2.resize(nr_frames);
					b2.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						A2[i].resize(nr_frames);
						b2[i].resize(nr_frames);
						for(unsigned int j = 0; j < nr_frames; j++){
							Matrix6d & ATA = A[i][j];
							Vector6d & ATb = b[i][j];
							ATA.setZero ();
							ATb.setZero ();
						}
					}

					for(int iteration = 0; iteration < 5; iteration++){
						printf("iteration: %i\n",iteration);

						double total_score = 0;

						double setup_equation_time_start = getTime();
						for(unsigned int i = 0; i < nr_frames; i++){
							if(!is_ok[i]){continue;}
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
								if(i == j){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];

								std::vector<std::pair<int,int> > & cm = current_matches[i][j];
								std::vector<double > & rw = rangeW[i][j];
								unsigned int current_nr_matches = cm.size();

								Matrix6d & ATA = A[i][j];
								Vector6d & ATb = b[i][j];
								ATA.setZero ();
								ATb.setZero ();

								//Matches from ki to kj
								for(unsigned int k = 0; k < current_nr_matches; k++){
									unsigned int ki = cm[k].first;
									unsigned int kj = cm[k].second;
									double rwij = rw[k];

									const float & sx = tXi(0,ki);
									const float & sy = tXi(1,ki);
									const float & sz = tXi(2,ki);

									const float & dx = tXj(0,kj);
									const float & dy = tXj(1,kj);
									const float & dz = tXj(2,kj);

									const float & nx = tXnj(0,kj);
									const float & ny = tXnj(1,kj);
									const float & nz = tXnj(2,kj);


									double a = nz*sy - ny*sz;
									double b = nx*sz - nz*sx;
									double c = ny*sx - nx*sy;

									//    0  1  2  3  4  5
									//    6  7  8  9 10 11
									//   12 13 14 15 16 17
									//   18 19 20 21 22 23
									//   24 25 26 27 28 29
									//   30 31 32 33 34 35

									ATA.coeffRef (0) += a * a;
									ATA.coeffRef (1) += a * b;
									ATA.coeffRef (2) += a * c;
									ATA.coeffRef (3) += a * nx;
									ATA.coeffRef (4) += a * ny;
									ATA.coeffRef (5) += a * nz;
									ATA.coeffRef (7) += b * b;
									ATA.coeffRef (8) += b * c;
									ATA.coeffRef (9) += b * nx;
									ATA.coeffRef (10) += b * ny;
									ATA.coeffRef (11) += b * nz;
									ATA.coeffRef (14) += c * c;
									ATA.coeffRef (15) += c * nx;
									ATA.coeffRef (16) += c * ny;
									ATA.coeffRef (17) += c * nz;
									ATA.coeffRef (21) += nx * nx;
									ATA.coeffRef (22) += nx * ny;
									ATA.coeffRef (23) += nx * nz;
									ATA.coeffRef (28) += ny * ny;
									ATA.coeffRef (29) += ny * nz;
									ATA.coeffRef (35) += nz * nz;

									double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
									total_score += d*d;
									ATb.coeffRef (0) += a * d;
									ATb.coeffRef (1) += b * d;
									ATb.coeffRef (2) += c * d;
									ATb.coeffRef (3) += nx * d;
									ATb.coeffRef (4) += ny * d;
									ATb.coeffRef (5) += nz * d;

								}
								ATA.coeffRef (6) = ATA.coeff (1);
								ATA.coeffRef (12) = ATA.coeff (2);
								ATA.coeffRef (13) = ATA.coeff (8);
								ATA.coeffRef (18) = ATA.coeff (3);
								ATA.coeffRef (19) = ATA.coeff (9);
								ATA.coeffRef (20) = ATA.coeff (15);
								ATA.coeffRef (24) = ATA.coeff (4);
								ATA.coeffRef (25) = ATA.coeff (10);
								ATA.coeffRef (26) = ATA.coeff (16);
								ATA.coeffRef (27) = ATA.coeff (22);
								ATA.coeffRef (30) = ATA.coeff (5);
								ATA.coeffRef (31) = ATA.coeff (11);
								ATA.coeffRef (32) = ATA.coeff (17);
								ATA.coeffRef (33) = ATA.coeff (23);
								ATA.coeffRef (34) = ATA.coeff (29);
							}
						}
						setup_equation_time += getTime()-setup_equation_time_start;

						double setup_equation_time_start2 = getTime();
						for(unsigned int i = 0; i < nr_frames; i++){
							if(!is_ok[i]){continue;}
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
								if(i == j){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];

								std::vector<std::pair<int,int> > & cm = current_matches[i][j];
								std::vector<double > & rw = rangeW[i][j];
								unsigned int current_nr_matches = cm.size();

								Matrix6d & ATA = A2[i][j];
								Vector6d & ATb = b2[i][j];
								ATA.setZero ();
								ATb.setZero ();

								//Matches from ki to kj
								for(unsigned int k = 0; k < current_nr_matches; k++){
									unsigned int ki = cm[k].first;
									unsigned int kj = cm[k].second;
									double rwij = rw[k];



									const float & sx = tXi(0,ki);
									const float & sy = tXi(1,ki);
									const float & sz = tXi(2,ki);

									const float & snx = tXni(0,ki);
									const float & sny = tXni(1,ki);
									const float & snz = tXni(2,ki);

									const float & dx = tXj(0,kj);
									const float & dy = tXj(1,kj);
									const float & dz = tXj(2,kj);

									const float & nx = tXnj(0,kj);
									const float & ny = tXnj(1,kj);
									const float & nz = tXnj(2,kj);


									double a = nz*sy - ny*sz;
									double b = nx*sz - nz*sx;
									double c = ny*sx - nx*sy;


									double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;

									double angle = nx*snx + ny*sny + nz*snz;
									if(angle < 0){continue;}

									double prob = func->getProb(d);
									double weight = prob*rwij;
									//    0  1  2  3  4  5
									//    6  7  8  9 10 11
									//   12 13 14 15 16 17
									//   18 19 20 21 22 23
									//   24 25 26 27 28 29
									//   30 31 32 33 34 35

									ATA.coeffRef (0) += weight * a * a;
									ATA.coeffRef (1) += weight * a * b;
									ATA.coeffRef (2) += weight * a * c;
									ATA.coeffRef (3) += weight * a * nx;
									ATA.coeffRef (4) += weight * a * ny;
									ATA.coeffRef (5) += weight * a * nz;
									ATA.coeffRef (7) += weight * b * b;
									ATA.coeffRef (8) += weight * b * c;
									ATA.coeffRef (9) += weight * b * nx;
									ATA.coeffRef (10) += weight * b * ny;
									ATA.coeffRef (11) += weight * b * nz;
									ATA.coeffRef (14) += weight * c * c;
									ATA.coeffRef (15) += weight * c * nx;
									ATA.coeffRef (16) += weight * c * ny;
									ATA.coeffRef (17) += weight * c * nz;
									ATA.coeffRef (21) += weight * nx * nx;
									ATA.coeffRef (22) += weight * nx * ny;
									ATA.coeffRef (23) += weight * nx * nz;
									ATA.coeffRef (28) += weight * ny * ny;
									ATA.coeffRef (29) += weight * ny * nz;
									ATA.coeffRef (35) += weight * nz * nz;

									ATb.coeffRef (0) += weight * a * d;
									ATb.coeffRef (1) += weight * b * d;
									ATb.coeffRef (2) += weight * c * d;
									ATb.coeffRef (3) += weight * nx * d;
									ATb.coeffRef (4) += weight * ny * d;
									ATb.coeffRef (5) += weight * nz * d;

								}
								ATA.coeffRef (6) = ATA.coeff (1);
								ATA.coeffRef (12) = ATA.coeff (2);
								ATA.coeffRef (13) = ATA.coeff (8);
								ATA.coeffRef (18) = ATA.coeff (3);
								ATA.coeffRef (19) = ATA.coeff (9);
								ATA.coeffRef (20) = ATA.coeff (15);
								ATA.coeffRef (24) = ATA.coeff (4);
								ATA.coeffRef (25) = ATA.coeff (10);
								ATA.coeffRef (26) = ATA.coeff (16);
								ATA.coeffRef (27) = ATA.coeff (22);
								ATA.coeffRef (30) = ATA.coeff (5);
								ATA.coeffRef (31) = ATA.coeff (11);
								ATA.coeffRef (32) = ATA.coeff (17);
								ATA.coeffRef (33) = ATA.coeff (23);
								ATA.coeffRef (34) = ATA.coeff (29);
							}
						}
						setup_equation_time2 += getTime()-setup_equation_time_start2;

						printf("total_score: %f\n",total_score);

						if(false){
							Eigen::Matrix4d p0inv = poses[0].inverse();
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
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

							std::vector<Eigen::MatrixXd> Xv;
							for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
							sprintf(buf,"image%5.5i.png",imgcount++);
							show(Xv,false,std::string(buf),imgcount);
						}

						double solve_equation_time_start = getTime();
						Eigen::MatrixXd fullA = Eigen::MatrixXd::Identity (6 * (nr_frames - 1), 6 * (nr_frames - 1));
						fullA *= 0.00001;
						Eigen::VectorXd fullB = Eigen::VectorXd::Zero (6 * (nr_frames - 1));

						for(unsigned int i = 0; i < nr_frames; i++){
							for(unsigned int j = 0; j == 0 && j < nr_frames; j++){
								if(i == j){continue;}
								Matrix6d & ATA = A[i][j];
								Vector6d & ATb = b[i][j];
								Vector6d x = ATA.inverse () * ATb;
								//printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
								//								Eigen::Matrix4d m = constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
								//								std::cout << m << std::endl << std::endl;

								//								printf("%i %i ->%i %i\n",i,j,6*(i-1), 6*j);
								if (i > 0){
									//fullA.block (6*(i-1), 6*j,6,6) -= ATA;
									fullA.block (6*(i-1),6*(i-1),6,6) += ATA;
									fullB.segment (6*(i-1), 6) += ATb;
								}
								//G.block (6 * (vi - 1), 6 * (vi - 1), 6, 6) += (*slam_graph_)[e].cinv_;
								//B.segment (6 * (vi - 1), 6) += (present1 ? 1 : -1) * (*slam_graph_)[e].cinvd_;
							}
						}
						//						std::cout << fullA << std::endl << std::endl;
						//						std::cout << fullB << std::endl << std::endl;
						//printf("--------------------------------------------------\n");
						Eigen::VectorXd fullX = fullA.inverse()*fullB;
						for(unsigned int i = 0; i < nr_frames; i++){
							Eigen::Matrix4d m;
							if(i == 0){
								m = Eigen::Matrix4d::Identity();
							}else{
								m = constructTransformationMatrix(fullX(6*(i-1)+0),fullX(6*(i-1)+1),fullX(6*(i-1)+2),fullX(6*(i-1)+3),fullX(6*(i-1)+4),fullX(6*(i-1)+5));
							}
							//std::cout << m << std::endl << std::endl;
						}

						//					    // Start at 1 because 0 is the reference pose
						//					    for (int vi = 1; vi != n; ++vi)
						//					    {
						//					      for (int vj = 0; vj != n; ++vj)
						//					      {
						//					        // Attempt to use the forward edge, otherwise use backward edge, otherwise there was no edge
						//					        Edge e;
						//					        bool present1, present2;
						//					        boost::tuples::tie (e, present1) = edge (vi, vj, *slam_graph_);
						//					        if (!present1)
						//					        {
						//					          boost::tuples::tie (e, present2) = edge (vj, vi, *slam_graph_);
						//					          if (!present2)
						//					            continue;
						//					        }

						//					        // Fill in elements of G and B
						//					        if (vj > 0)
						//					          G.block (6 * (vi - 1), 6 * (vj - 1), 6, 6) = -(*slam_graph_)[e].cinv_;
						//					        G.block (6 * (vi - 1), 6 * (vi - 1), 6, 6) += (*slam_graph_)[e].cinv_;
						//					        B.segment (6 * (vi - 1), 6) += (present1 ? 1 : -1) * (*slam_graph_)[e].cinvd_;
						//					      }
						//					    }

						//					    // Computation of the linear equation system: GX = B
						//					    // TODO investigate accuracy vs. speed tradeoff and find the best solving method for our type of linear equation (sparse)
						//					    Eigen::VectorXf X = G.colPivHouseholderQr ().solve (B);

						//					    // Update the poses
						//					    float sum = 0.0;
						//					    for (int vi = 1; vi != n; ++vi)
						//					    {
						//					      Eigen::Vector6f difference_pose = static_cast<Eigen::Vector6f> (-incidenceCorrection (getPose (vi)).inverse () * X.segment (6 * (vi - 1), 6));
						//					      sum += difference_pose.norm ();
						//					      setPose (vi, getPose (vi) + difference_pose);
						//					    }

						//						for(unsigned int i = 0; i < nr_frames; i++){
						//							for(unsigned int j = 0; j < nr_frames; j++){
						//								if(i == j){continue;}
						//								Matrix6d & ATA = A[i][j];
						//								Vector6d & ATb = b[i][j];
						//								Vector6d x = ATA.inverse () * ATb;
						//								printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));

						//								Matrix6d & ATA2 = A2[i][j];
						//								Vector6d & ATb2 = b2[i][j];
						//								Vector6d x2 = ATA2.inverse () * ATb2;
						//								printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x2(0,0),x2(1,0),x2(2,0),x2(3,0),x2(4,0),x2(5,0));

						//								Eigen::Matrix4d m = constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
						//								std::cout << m << std::endl << std::endl;
						//							}
						//						}

						solve_equation_time += getTime()-solve_equation_time_start;

						if(false){
							Eigen::Matrix4d p0inv = poses[0].inverse();
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
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

							std::vector<Eigen::MatrixXd> Xv;
							for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
							sprintf(buf,"image%5.5i.png",imgcount++);
							show(Xv,false,std::string(buf),imgcount);
						}
						//Recover poses from solution
						//Recompute points

						//if(isconverged(poses, localposes, stopval, stopval)){break;}
					}

					printf("total_time:           %5.5f\n",getTime()-total_time_start);
					printf("rematch_time:         %5.5f\n",rematch_time);
					printf("computeModel:         %5.5f\n",computeModel_time);
					printf("setup_matches_time:   %5.5f\n",setup_matches_time);
					printf("setup_equation_time:  %5.5f\n",setup_equation_time);
					printf("setup_equation_time2: %5.5f\n",setup_equation_time2);
					printf("solve_equation_time:  %5.5f\n",solve_equation_time);
					exit(0);
				}else{
					for(int outer=0; outer < 30; ++outer) {
						if(getTime()-total_time_start > timeout){break;}
						printf("funcupdate: %i rematching: %i lala: %i outer: %i\n",funcupdate,rematching,lala,outer);
						for(unsigned int i = 0; i < nr_frames; i++){poses2[i] = poses[i];}
						for(unsigned int i = 0; i < nr_frames; i++){
							if(getTime()-total_time_start > timeout){break;}
							if(!is_ok[i]){continue;}
							unsigned int nr_match = 0;
							{
								for(unsigned int j = 0; j < nr_frames; j++){
									if(!is_ok[j]){continue;}
									std::vector<int> & matchidj = matchids[j][i];
									unsigned int matchesj = matchidj.size();
									std::vector<int> & matchidi = matchids[i][j];
									unsigned int matchesi = matchidi.size();

									for(unsigned int ki = 0; ki < matchesi; ki++){
										int kj = matchidi[ki];
										if( kj == -1 ){continue;}
										if( kj >=  matchesj){continue;}
										if(!onetoone || matchidj[kj] == ki){	nr_match++;}
									}
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
								if(!is_ok[j]){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
								Eigen::VectorXd & informationj					= informations[j];

								std::vector<int> & matchidj = matchids[j][i];
								unsigned int matchesj = matchidj.size();
								std::vector<int> & matchidi = matchids[i][j];
								unsigned int matchesi = matchidi.size();

								for(unsigned int ki = 0; ki < matchesi; ki++){
									int kj = matchidi[ki];
									if( kj == -1 ){continue;}
									if( kj >=  matchesj){continue;}
									if(!onetoone || matchidj[kj] == ki){
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

							if(count == 0){break;}

							//showMatches(Xp,Qp);
							for(int inner=0; inner < 5; ++inner) {
								Eigen::MatrixXd residuals;
								switch(type) {
								case PointToPoint:	{residuals = Xp-Qp;} 						break;
								case PointToPlane:	{
									residuals		= Eigen::MatrixXd::Zero(1,	Xp.cols());
									for(int i=0; i<Xp.cols(); ++i) {
										float dx = Xp(0,i)-Qp(0,i);
										float dy = Xp(1,i)-Qp(1,i);
										float dz = Xp(2,i)-Qp(2,i);
										float qx = Qn(0,i);
										float qy = Qn(1,i);
										float qz = Qn(2,i);
										float di = qx*dx + qy*dy + qz*dz;
										residuals(0,i) = di;
									}
								}break;
								default:			{printf("type not set\n");}					break;
								}
								for(unsigned int k=0; k < nr_match; ++k) {residuals.col(k) *= rangeW(k);}

								Eigen::VectorXd  W;
								switch(type) {
								case PointToPoint:	{W = func->getProbs(residuals); } 					break;
								case PointToPlane:	{
									W = func->getProbs(residuals);
									for(int k=0; k<nr_match; ++k) {W(k) = W(k)*float((Xn(0,k)*Qn(0,k) + Xn(1,k)*Qn(1,k) + Xn(2,k)*Qn(2,k)) > 0.0);}
								}	break;
								default:			{printf("type not set\n");} break;
								}


								for(int k=0; k<nr_match; ++k) {
									if(W(k) > 0.1){good_opt++;}
									else{bad_opt++;}
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
								if(stop1 < 0.001){break; }
							}

							pcl::TransformationFromCorrespondences tfc;
							for(unsigned int c = 0; c < nr_match; c++){
								Eigen::Vector3f a (Xp(0,c),				Xp(1,c),			Xp(2,c));
								Eigen::Vector3f b (Xp_ori(0,c),         Xp_ori(1,c),        Xp_ori(2,c));
								tfc.add(b,a);
							}
							poses[i] = tfc.getTransformation().cast<double>().matrix();
						}

						double change = 0;
						Eigen::Matrix4d p0inv = poses[0].inverse();
						for(unsigned int j = 0; j < nr_frames; j++){
							if(!is_ok[j]){continue;}
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

								double tXi0c	= tXi(0,c);
								double tXi1c	= tXi(1,c);
								double tXi2c	= tXi(2,c);

								tXi(0,c)		= m00*x + m01*y + m02*z + m03;
								tXi(1,c)		= m10*x + m11*y + m12*z + m13;
								tXi(2,c)		= m20*x + m21*y + m22*z + m23;

								tXni(0,c)		= m00*nx + m01*ny + m02*nz;
								tXni(1,c)		= m10*nx + m11*ny + m12*nz;
								tXni(2,c)		= m20*nx + m21*ny + m22*nz;

								double dx = tXi0c-tXi(0,c);
								double dy = tXi1c-tXi(1,c);
								double dz = tXi2c-tXi(2,c);
								double d = dx*dx+dy*dy+dz*dz;
								change = std::max(d,change);
							}
						}
						change = sqrt(change);

						double change_trans = 0;
						double change_rot = 0;
						for(unsigned int i = 0; i < nr_frames; i++){
							if(!is_ok[i]){continue;}
							for(unsigned int j = i+1; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
								Eigen::Matrix4d diff_before = poses2[i].inverse()*poses2[j];
								Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
								Eigen::Matrix4d diff = diff_before.inverse()*diff_after;
								double dt = 0;
								for(unsigned int k = 0; k < 3; k++){
									dt += diff(k,3)*diff(k,3);
									for(unsigned int l = 0; l < 3; l++){
										if(k == l){ change_rot += fabs(1-diff(k,l));}
										else{		change_rot += fabs(diff(k,l));}
									}
								}
								change_trans += sqrt(dt);
							}
						}
						change_trans /= double(nr_frames*(nr_frames-1));
						change_rot	 /= double(nr_frames*(nr_frames-1));
						if(change_trans < stopval && change_rot < stopval){break;}
					}
				}
				double change_trans = 0;
				double change_rot = 0;
				for(unsigned int i = 0; i < nr_frames; i++){
					if(!is_ok[i]){continue;}
					for(unsigned int j = i+1; j < nr_frames; j++){
						if(!is_ok[j]){continue;}
						Eigen::Matrix4d diff_before = poses2b[i].inverse()*poses2b[j];
						Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
						Eigen::Matrix4d diff = diff_before.inverse()*diff_after;

						double dt = 0;
						for(unsigned int k = 0; k < 3; k++){
							dt += diff(k,3)*diff(k,3);
							for(unsigned int l = 0; l < 3; l++){
								if(k == l){ change_rot += fabs(1-diff(k,l));}
								else{		change_rot += fabs(diff(k,l));}
							}
						}
						change_trans += sqrt(dt);
					}
				}

				change_trans /= double(nr_frames*(nr_frames-1));
				change_rot	 /= double(nr_frames*(nr_frames-1));

				if(change_trans < stopval && change_rot < stopval){break;}
			}

			double change_trans = 0;
			double change_rot = 0;
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				for(unsigned int j = i+1; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					Eigen::Matrix4d diff_before = poses1[i].inverse()*poses1[j];
					Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
					Eigen::Matrix4d diff = diff_before.inverse()*diff_after;

					double dt = 0;
					for(unsigned int k = 0; k < 3; k++){
						dt += diff(k,3)*diff(k,3);
						for(unsigned int l = 0; l < 3; l++){
							if(k == l){ change_rot += fabs(1-diff(k,l));}
							else{		change_rot += fabs(diff(k,l));}
						}
					}
					change_trans += sqrt(dt);
				}
			}

			change_trans /= double(nr_frames*(nr_frames-1));
			change_rot	 /= double(nr_frames*(nr_frames-1));
			if(change_trans < stopval && change_rot < stopval){break;}
		}

		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}

	printf("total_time:          %5.5f\n",getTime()-total_time_start);
	printf("rematch_time:        %5.5f\n",rematch_time);
	printf("computeModel:        %5.5f\n",computeModel_time);
	printf("setup_matches_time:  %5.5f\n",setup_matches_time);
	printf("setup_equation_time: %5.5f\n",setup_equation_time);
	printf("solve_equation_time: %5.5f\n",solve_equation_time);
	printf("good opt: %f bad opt: %f ratio: %f\n",good_opt,bad_opt,good_opt/(good_opt+bad_opt));

	if(visualizationLvl > 0){
		std::vector<Eigen::MatrixXd> Xv;
		for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
		sprintf(buf,"image%5.5i.png",imgcount++);
		show(Xv,false,std::string(buf),imgcount);
	}

	Eigen::Matrix4d firstinv = poses.front().inverse();
	for(int i = 0; i < nr_frames; i++){poses[i] = firstinv*poses[i];}

	printf("stop MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> guess)\n");
	return MassFusionResults(poses,-1);
}

}
