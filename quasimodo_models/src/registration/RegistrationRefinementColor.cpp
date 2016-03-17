#include "RegistrationRefinementColor.h"
#include <iostream>
#include <fstream>

#include "myhull.h"

//#include <pcl/surface/convex_hull.h>
#include <pcl/registration/lum.h>

using namespace std;
using namespace nanoflann;



namespace reglib
{

RegistrationRefinementColor::RegistrationRefinementColor(){
	only_initial_guess		= false;

	type					= PointToPlane;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;
	//DistanceWeightFunction2PPR * fu = new DistanceWeightFunction2PPR();
	DistanceWeightFunction2PPR2 * fu = new DistanceWeightFunction2PPR2();
	fu->startreg			= 0.001;
	fu->debugg_print		= false;
	func					= fu;

    funcR = new DistanceWeightFunction2PPR2();
    funcR->fixed_histogram_size = true;
    funcR->maxd                 = 255;
    funcR->histogram_size       = 255;
	funcR->startreg             = 5.0;
    funcR->debugg_print         = true;

    funcG = new DistanceWeightFunction2PPR2();
    funcG->fixed_histogram_size     = true;
    funcG->maxd                     = 255;
    funcG->histogram_size           = 255;
	funcG->startreg                 = 5.0;
    funcG->debugg_print             = false;

    funcB = new DistanceWeightFunction2PPR2();
    funcB->fixed_histogram_size     = true;
    funcB->maxd                     = 255;
    funcB->histogram_size           = 255;
	funcB->startreg                 = 5.0;
    funcB->debugg_print             = false;

	visualizationLvl = 1;

	for(unsigned int i= 3; i < 6; i++){
		feature_start.push_back(i);
		feature_end.push_back(i);
		DistanceWeightFunction2PPR * f = new DistanceWeightFunction2PPR();
		f->regularization	= 0.0;
		f->maxd 			= 255;
		f->histogram_size	= 255;
		feature_func.push_back(f);
	}

    target_points = 250;
    allow_regularization = true;
    maxtime = 9999999;
}
RegistrationRefinementColor::~RegistrationRefinementColor(){}

namespace RigidMotionEstimatorRefinementColor {
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Confidence weights
/// @param Right hand side
template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
							   Eigen::MatrixBase<Derived2>& Xn,
							   Eigen::MatrixBase<Derived3>& Y,
							   Eigen::MatrixBase<Derived4>& N,
							   const Eigen::MatrixBase<Derived5>& w,
							   const Eigen::MatrixBase<Derived6>& u) {
	typedef Eigen::Matrix<double, 6, 6> Matrix66;
	typedef Eigen::Matrix<double, 6, 1> Vector6;
	typedef Eigen::Block<Matrix66, 3, 3> Block33;

	/// Normalize weight vector
	Eigen::VectorXd w_normalized = w/w.sum();
	/// De-mean
	Eigen::Vector3d X_mean;
	for(int i=0; i<3; ++i){X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();}

	Eigen::Vector3d Y_mean;
	for(int i=0; i<3; ++i){Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();}

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

	Xn = transformation*Xn;

	transformation.translation() = RHS.tail<3>();
	/// Apply transformation
	X = transformation*X;
	/// Re-apply mean
	X.colwise() += X_mean;
	Y.colwise() += X_mean;
	//Eigen::Vector3d mean_diff;
	//for(int i=0; i<3; ++i){mean_diff(i) = ((X-Y).row(i).array()*w_normalized.transpose().array()).sum();}

	//std::cout << mean_diff << std::endl << std::endl;
	//std::cout << Y_mean << std::endl << std::endl;
	//std::cout << Y_mean-X_mean << std::endl << std::endl;
	//std::cout << transformation.translation() << std::endl << std::endl;
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
									  const Eigen::MatrixBase<Derived5>& w) {
	return point_to_plane(X,Xn,Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
}
}

void dump_mem_usage()
{
	FILE* f=fopen("/proc/self/statm","rt");
	if (!f) return;
	char str[300];
	size_t n=fread(str,1,200,f);
	str[n]=0;
	printf("MEM: %s\n",str);
	fclose(f);
}

void RegistrationRefinementColor::setDst(CloudData * dst_){
	dst = dst_;
	unsigned int d_nr_data = dst->data.cols();
	int stepy = std::max(1,int(d_nr_data)/100000);
	Y.resize(Eigen::NoChange,d_nr_data/stepy);
    C.resize(Eigen::NoChange,d_nr_data/stepy);
	N.resize(Eigen::NoChange,d_nr_data/stepy);
	ycols = Y.cols();
	total_dweight.resize(ycols);

	printf("stepy: %i\n",stepy);

    for(unsigned int i = 0; i < d_nr_data/stepy; i++){
        Y(0,i)	= dst->data(0,i*stepy);
        Y(1,i)	= dst->data(1,i*stepy);
        Y(2,i)	= dst->data(2,i*stepy);
        C(0,i)	= dst->data(3,i*stepy);
        C(1,i)	= dst->data(4,i*stepy);
        C(2,i)	= dst->data(5,i*stepy);
		N(0,i)	= dst->normals(0,i*stepy);
		N(1,i)	= dst->normals(1,i*stepy);
		N(2,i)	= dst->normals(2,i*stepy);
	}

//	cloud.pts.resize(d_nr_data);
//	for(unsigned int i = 0; i < d_nr_data; i++){
//		cloud.pts[i].x = dst->data(0,i);
//		cloud.pts[i].y = dst->data(1,i);
//		cloud.pts[i].z = dst->data(2,i);
//		cloud.pts[i].r = dst->data(3,i);
//		cloud.pts[i].g = dst->data(4,i);
//		cloud.pts[i].b = dst->data(5,i);
//	}
//	cloud.count = 0;
//	cloud.w0 = 1;
//	cloud.w1 = 1;
//	cloud.w2 = 1;
//	cloud.w3 = 0;
//	cloud.w4 = 0;
//	cloud.w5 = 0;
//	xyzrgb_tree = new nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, PointCloudXYZRGB<double> > , PointCloudXYZRGB<double>, 6 > (6 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
//	xyzrgb_tree->buildIndex();
//	dump_mem_usage();

//	double query_pt[6] = {0,0,0,0,0,0};

//	const size_t num_results = 1;
//	size_t ret_index [num_results];
//	double out_dist_sqr  [num_results];

//	nanoflann::KNNResultSet<double> resultSet(num_results);
//	resultSet.init( ret_index,  out_dist_sqr );

//	nanoflann::SearchParams sp (10);
//	sp.eps = 0.0;
//	xyzrgb_tree->findNeighbors(resultSet, &query_pt[0], sp);


//	std::cout << "knnSearch(nn="<<num_results<<"): \n";
//	for(int j = 0; j < 1; j++){
//		std::cout << "ret_index=" << ret_index[j] << " out_dist_sqr=" << out_dist_sqr[j] << endl;
//	}

	/// Build kd-tree
	tree = new nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>(Y);

	DST_INORMATION = Eigen::VectorXd::Zero(ycols);
	for(unsigned int i = 0; i < d_nr_data/stepy; i++){DST_INORMATION(i) = dst->information(0,i*stepy);}

}
using namespace Eigen;
using namespace std;
Eigen::MatrixXd align(Eigen::VectorXd W, Eigen::MatrixXd X, Eigen::MatrixXd Y, Eigen::MatrixXd bias){
    Eigen::MatrixXd WX = X;
    for(int i = 0; i < WX.rows(); i++){
        for(int j = 0; j < WX.cols(); j++){
             WX(i,j) = W(j)*X(i,j);
        }
    }
    Eigen::MatrixXd ret =   (WX.transpose()*X + bias).ldlt().solve(WX.transpose() * Y);
    return ret;
}

double test(Eigen::VectorXd W, Eigen::MatrixXd X, Eigen::MatrixXd Y){
    printf("X %i %i\n",X.rows(),X.cols());
    printf("Y %i %i\n",Y.rows(),Y.cols());

   Eigen::MatrixXd err = X-Y;
   double sum = 0;
   double sumWi = 0;

   printf("%i %i\n",err.rows(),err.cols());
   for(int j = 0; j < err.cols(); j++){
       for(int i = 0; i < err.rows(); i++){
           double Wi = W(i);
           sumWi += Wi;
           sum += Wi*err(i,j)*err(i,j);
       }
   }
   sum = sqrt(sum/sumWi);

   printf("sum: %f sumWi %f \n",sum,sumWi);
    return sum;
}

FusionResults RegistrationRefinementColor::getTransform(Eigen::MatrixXd guess){

	double query_pt[6] = {0,0,0,0,0,0};
	const size_t num_results = 1;
	size_t ret_index [num_results];
	double out_dist_sqr  [num_results];

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init( ret_index,  out_dist_sqr );

	nanoflann::SearchParams sp (10);
	sp.eps = 0.0;
/*
	const size_t num_results = 1;
	size_t ret_index [num_results];
	double out_dist_sqr  [num_results];

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init( ret_index,  out_dist_sqr );

	nanoflann::SearchParams sp (10);
	sp.eps = 0.0;
	xyzrgb_tree->findNeighbors(resultSet, &query_pt[0], sp);


	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	for(int j = 0; j < 1; j++){
		std::cout << "ret_index=" << ret_index[j] << " out_dist_sqr=" << out_dist_sqr[j] << endl;
	}
exit(0);
*/
	unsigned int s_nr_data = src->data.cols();
	int stepx = std::max(1,int(s_nr_data)/target_points);

	double stop		= 0.00001;

	func->reset();
	if(!allow_regularization){func->regularization = 0;}

	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);

	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Xn;
	X.resize(Eigen::NoChange,s_nr_data/stepx);
	Xn.resize(Eigen::NoChange,s_nr_data/stepx);
	unsigned int xcols = X.cols();
	printf("xcols: %i\n",xcols);

	/// Buffers
	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	xcols);
	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	xcols);
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	xcols);
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	xcols);
	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	xcols);

	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(xcols);
	for(unsigned int i = 0; i < s_nr_data/stepx; i++){
		SRC_INORMATION(i) = src->information(0,i*stepx);
		float x		= src->data(0,i*stepx);
		float y		= src->data(1,i*stepx);
		float z		= src->data(2,i*stepx);
		float xn	= src->normals(0,i*stepx);
		float yn	= src->normals(1,i*stepx);
		float zn	= src->normals(2,i*stepx);
		X(0,i)	= m00*x + m01*y + m02*z + m03;
		X(1,i)	= m10*x + m11*y + m12*z + m13;
		X(2,i)	= m20*x + m21*y + m22*z + m23;
		Xn(0,i)	= m00*xn + m01*yn + m02*zn;
		Xn(1,i)	= m10*xn + m11*yn + m12*zn;
		Xn(2,i)	= m20*xn + m21*yn + m22*zn;
	}

	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;
	Eigen::Matrix3Xd Xo4 = X;
	Eigen::MatrixXd residuals;

	std::vector<int> matchid;
	matchid.resize(xcols);

	double score = 0;
	stop = 99999;

	double start = getTime();

	//double query_pt [6];

	/// ICP
	for(int funcupdate=0; funcupdate < 10; ++funcupdate) {
		if( (getTime()-start) > maxtime ){break;}
		for(int rematching=0; rematching < 10; ++rematching) {
			if( (getTime()-start) > maxtime ){break;}

#pragma omp parallel for
			for(unsigned int i=0; i< xcols; ++i) {
					matchid[i] = tree->closest(X.col(i).data());
//					printf("%f %f %f\n",X(0,i),X(1,i),X(2,i));
//					query_pt[0] = X(0,i);
//					query_pt[1] = X(1,i);
//					query_pt[2] = X(2,i);
//					query_pt[3] = 0;
//					query_pt[4] = 0;
//					query_pt[5] = 0;
//					xyzrgb_tree->findNeighbors(resultSet, &query_pt[0], sp);

//					printf("index old: %i index new: %i\n",matchid[i],ret_index[0]);
					//std::cout << "ret_index=" << ret_index[j] << " out_dist_sqr=" << out_dist_sqr[j] << endl;
			}
exit(0);
			/// Find closest point
#pragma omp parallel for
			for(unsigned int i=0; i< xcols; ++i) {
				int id = matchid[i];
				Qn.col(i) = N.col(id);
				Qp.col(i) = Y.col(id);
				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			}

			for(int outer=0; outer< 1; ++outer) {
				if( (getTime()-start) > maxtime ){break;}
				/// Compute weights
				switch(type) {
					case PointToPoint:	{residuals = X-Qp;} 						break;
					case PointToPlane:	{
						residuals		= Eigen::MatrixXd::Zero(1,	xcols);
						for(int i=0; i<xcols; ++i) {
							float dx = X(0,i)-Qp(0,i);
							float dy = X(1,i)-Qp(1,i);
							float dz = X(2,i)-Qp(2,i);
							float qx = Qn(0,i);
							float qy = Qn(1,i);
							float qz = Qn(2,i);
							float di = qx*dx + qy*dy + qz*dz;
							residuals(0,i) = di;
						}
					}break;
					default:			{printf("type not set\n");}					break;
				}
				for(int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}

				switch(type) {
					case PointToPoint:	{func->computeModel(residuals);} 	break;
					case PointToPlane:	{func->computeModel(residuals);}	break;
					default:  			{printf("type not set\n");} break;
				}

				for(int rematching2=0; rematching2 < 3; ++rematching2) {
					if( (getTime()-start) > maxtime ){break;}
					if(rematching2 != 0){
#pragma omp parallel for
						for(unsigned int i=0; i< xcols; ++i) {matchid[i] = tree->closest(X.col(i).data());}

						/// Find closest point
#pragma omp parallel for
						for(unsigned int i=0; i< xcols; ++i) {
							int id = matchid[i];
							Qn.col(i) = N.col(id);
							Qp.col(i) = Y.col(id);
							rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
						}
					}

					for(int inner=0; inner< 40; ++inner) {
						if( (getTime()-start) > maxtime ){break;}
						if(inner != 0){
							switch(type) {
								case PointToPoint:	{residuals = X-Qp;} 						break;
								case PointToPlane:	{
									residuals		= Eigen::MatrixXd::Zero(1,	xcols);
									for(int i=0; i< xcols; ++i) {
										float dx = X(0,i)-Qp(0,i);
										float dy = X(1,i)-Qp(1,i);
										float dz = X(2,i)-Qp(2,i);
										float qx = Qn(0,i);
										float qy = Qn(1,i);
										float qz = Qn(2,i);
										float di = qx*dx + qy*dy + qz*dz;
										residuals(0,i) = di;
									}
								}break;
								default:			{printf("type not set\n");}					break;
							}
							for(int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}
						}

						switch(type) {
							case PointToPoint:	{W = func->getProbs(residuals); } 					break;
							case PointToPlane:	{
								W = func->getProbs(residuals);
								for(int i=0; i<xcols; ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
							}	break;
							default:			{printf("type not set\n");} break;
						}
						Wold = W;

						//Normalizing weights has an effect simmilar to one to one matching
						//in that it reduces the effect of border points
						if(normalize_matchweights){
							for(unsigned int i=0; i < ycols; ++i)	{	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
							for(unsigned int i=0; i< xcols; ++i)	{	total_dweight[matchid[i]] += W(i);}
							for(unsigned int i=0; i< xcols; ++i)	{	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
						}

						W = W.array()*rangeW.array()*rangeW.array();
						Eigen::Affine3d change;
						switch(type) {
							case PointToPoint:	{RigidMotionEstimator::point_to_point(X, Qp, W);}		break;
							case PointToPlane:	{change = RigidMotionEstimatorRefinementColor::point_to_plane(X, Xn, Qp, Qn, W);}	break;
							default:  			{printf("type not set\n"); } break;
						}

						stop = 100*func->getConvergenceThreshold();
						score = Wold.sum()/(pow(func->getNoise(),2)*float(xcols));
						//score = Wold.sum()/float(xcols);
						double stop1 = (X-Xo1).colwise().norm().mean();
						Xo1 = X;
						if(stop1 < stop) break;
					}
					double stop2 = (X-Xo2).colwise().norm().mean();
					Xo2 = X;
					if(stop2 < stop) break;
				}
				double stop3 = (X-Xo3).colwise().norm().mean();
				Xo3 = X;
				if(stop3 < stop) break;
			}
			double stop4 = (X-Xo4).colwise().norm().mean();
			Xo4 = X;
			if(stop4 < stop) break;
		}
		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}

	if(visualizationLvl >= 2){show(X,Y);}

	pcl::TransformationFromCorrespondences tfc;
	tfc.reset();
	for(unsigned int i = 0; i < xcols; i++){
		Eigen::Vector3f a (X(0,i),						X(1,i),					X(2,i));
		Eigen::Vector3f b (src->data(0,i*stepx),		src->data(1,i*stepx),	src->data(2,i*stepx));
		tfc.add(b,a);
	}
	guess = tfc.getTransformation().matrix().cast<double>();
	FusionResults fr = FusionResults(guess,stop);
	return fr;

//	unsigned int s_nr_data = src->data.cols();
//    int stepx = std::max(1,int(s_nr_data)/target_points);

//	double stop		= 0.00001;

//	func->reset();
//    if(!allow_regularization){func->regularization = 0;}

//	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
//	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
//	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);

//	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
//    Eigen::Matrix<double, 3, Eigen::Dynamic> Xc;
//	Eigen::Matrix<double, 3, Eigen::Dynamic> Xn;
//	X.resize(Eigen::NoChange,s_nr_data/stepx);
//    Xc.resize(Eigen::NoChange,s_nr_data/stepx);
//	Xn.resize(Eigen::NoChange,s_nr_data/stepx);
//	unsigned int xcols = X.cols();

//    Eigen::MatrixXd colorTransform;

//	/// Buffers
//	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	xcols);
//    Eigen::Matrix3Xd Qc		= Eigen::Matrix3Xd::Zero(3,	xcols);
//	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	xcols);
//	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	xcols);
//	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	xcols);
//	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	xcols);

//    Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(7,7);
//    for(unsigned int i = 0; i < 7; i++){bias(i,i) = 1;}
//    Eigen::MatrixXd Xfull = Eigen::MatrixXd::Zero(xcols,7);

//    Eigen::MatrixXd ct = Eigen::MatrixXd::Zero(7,3);
//    ct(3,0) = 1;
//    ct(4,1) = 1;
//    ct(5,2) = 1;

//    Eigen::MatrixXd Xcol = Eigen::MatrixXd::Zero(xcols,3);
//    Eigen::MatrixXd Xcol2 = Eigen::MatrixXd::Zero(xcols,3);

//	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(xcols);
//	for(unsigned int i = 0; i < s_nr_data/stepx; i++){
//		SRC_INORMATION(i) = src->information(0,i*stepx);
//		float x		= src->data(0,i*stepx);
//		float y		= src->data(1,i*stepx);
//		float z		= src->data(2,i*stepx);

//        float r		= src->data(3,i*stepx);
//        float g		= src->data(4,i*stepx);
//        float b		= src->data(5,i*stepx);

//		float xn	= src->normals(0,i*stepx);
//		float yn	= src->normals(1,i*stepx);
//		float zn	= src->normals(2,i*stepx);
//        X(0,i)	= m00*x + m01*y + m02*z + m03;
//		X(1,i)	= m10*x + m11*y + m12*z + m13;
//		X(2,i)	= m20*x + m21*y + m22*z + m23;

//        Xc(0,i)	= r;
//        Xc(1,i)	= g;
//        Xc(2,i)	= b;


//        Xcol2(i,0)	= r;
//        Xcol2(i,1)	= g;
//        Xcol2(i,2)	= b;



//        Xfull(i,0) = X(0,i);
//        Xfull(i,1) = X(1,i);
//        Xfull(i,2) = X(2,i);
//        Xfull(i,3) = r;
//        Xfull(i,4) = g;
//        Xfull(i,5) = b;
//        Xfull(i,6) = 1;
//        W(i) = 1;


//		Xn(0,i)	= m00*xn + m01*yn + m02*zn;
//		Xn(1,i)	= m10*xn + m11*yn + m12*zn;
//		Xn(2,i)	= m20*xn + m21*yn + m22*zn;
//	}

//    Eigen::Matrix3Xd Xo1 = X;
//	Eigen::Matrix3Xd Xo2 = X;
//	Eigen::Matrix3Xd Xo3 = X;
//	Eigen::Matrix3Xd Xo4 = X;
//	Eigen::MatrixXd residuals;
//    Eigen::MatrixXd residualsR;
//    Eigen::MatrixXd residualsG;
//    Eigen::MatrixXd residualsB;


//	std::vector<int> matchid;
//	matchid.resize(xcols);

//	double score = 0;
//	stop = 99999;

//    double start = getTime();

//	/// ICP
//	for(int funcupdate=0; funcupdate < 10; ++funcupdate) {
//        if( (getTime()-start) > maxtime ){break;}

//		for(int rematching=0; rematching < 10; ++rematching) {
//            if( (getTime()-start) > maxtime ){break;}
//#pragma omp parallel for
//			for(unsigned int i=0; i< xcols; ++i) {matchid[i] = tree->closest(X.col(i).data());}
//            printf("%i %i\n",funcupdate,rematching);

//			/// Find closest point
//#pragma omp parallel for
//			for(unsigned int i=0; i< xcols; ++i) {
//				int id = matchid[i];

//                Xcol(i,0)	= C(0,id);
//                Xcol(i,1)	= C(1,id);
//                Xcol(i,2)	= C(2,id);

//				Qn.col(i) = N.col(id);
//                Qc.col(i) = C.col(id);
//				Qp.col(i) = Y.col(id);
//				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
//			}

//			for(int outer=0; outer< 1; ++outer) {
//                if( (getTime()-start) > maxtime ){break;}

//                residualsR		= Eigen::MatrixXd::Zero(1,	xcols);
//                residualsG		= Eigen::MatrixXd::Zero(1,	xcols);
//                residualsB		= Eigen::MatrixXd::Zero(1,	xcols);

//                Eigen::MatrixXd  transformed_colors = Xfull*ct;

//                for(int i=0; i<xcols; ++i) {
//                    residualsR(0,i) = transformed_colors(i,0)-Qc(0,i);
//                    residualsG(0,i) = transformed_colors(i,1)-Qc(1,i);
//                    residualsB(0,i) = transformed_colors(i,2)-Qc(2,i);
//                }

//                //test(W, transformed_colors, Xcol);


//                //ct = align(W, Xfull, Xcol-Xcol2, xcols*xcols*bias);
//                //ct(3,0) += 1;
//                //ct(4,1) += 1;
//                //ct(5,2) += 1;
//                //cout << "The solution using normal equations is:\n" <<  ct << endl;
//                //test(W, Xfull*ct, Xcol);
//                //test(W, Xcol2, Xcol);
//                //exit(0);
//				/// Compute weights
//				switch(type) {
//					case PointToPoint:	{residuals = X-Qp;} 						break;
//					case PointToPlane:	{
//						residuals		= Eigen::MatrixXd::Zero(1,	xcols);
//						for(int i=0; i<xcols; ++i) {
//							float dx = X(0,i)-Qp(0,i);
//							float dy = X(1,i)-Qp(1,i);
//							float dz = X(2,i)-Qp(2,i);
//							float qx = Qn(0,i);
//							float qy = Qn(1,i);
//							float qz = Qn(2,i);
//							float di = qx*dx + qy*dy + qz*dz;
//							residuals(0,i) = di;
//						}
//					}break;
//					default:			{printf("type not set\n");}					break;
//				}
//				for(int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}
//func->computeModel(residuals);
//funcR->computeModel(residualsR);
//funcG->computeModel(residualsG);
//funcB->computeModel(residualsB);

//for(int inner=0; inner< 10; ++inner) {
//    if( (getTime()-start) > maxtime ){break;}

//    transformed_colors = Xfull*ct;

//    if(inner != 0){
//        for(int i=0; i<xcols; ++i) {
//            residualsR(0,i) = transformed_colors(i,0)-Qc(0,i);
//            residualsG(0,i) = transformed_colors(i,1)-Qc(1,i);
//            residualsB(0,i) = transformed_colors(i,2)-Qc(2,i);
//        }

//        switch(type) {
//            case PointToPoint:	{residuals = X-Qp;} 						break;
//            case PointToPlane:	{
//                residuals		= Eigen::MatrixXd::Zero(1,	xcols);
//                for(int i=0; i< xcols; ++i) {
//                    float dx = X(0,i)-Qp(0,i);
//                    float dy = X(1,i)-Qp(1,i);
//                    float dz = X(2,i)-Qp(2,i);
//                    float qx = Qn(0,i);
//                    float qy = Qn(1,i);
//                    float qz = Qn(2,i);
//                    float di = qx*dx + qy*dy + qz*dz;
//                    residuals(0,i) = di;
//                }
//            }break;
//            default:			{printf("type not set\n");}					break;
//        }
//        for(int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}

//        switch(type) {
//            case PointToPoint:	{W = func->getProbs(residuals); } 					break;
//            case PointToPlane:	{
//                W = func->getProbs(residuals);
//                for(int i=0; i<xcols; ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
//            }	break;
//            default:			{printf("type not set\n");} break;
//        }
//        Wold = W;

//        //Normalizing weights has an effect simmilar to one to one matching
//        //in that it reduces the effect of border points
//        if(normalize_matchweights){
//            for(unsigned int i=0; i < ycols; ++i)	{	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
//            for(unsigned int i=0; i< xcols; ++i)	{	total_dweight[matchid[i]] += W(i);}
//            for(unsigned int i=0; i< xcols; ++i)	{	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
//        }

//        printf("-----------\n");
//        test(W, transformed_colors, Xcol);

//        ct = align(W, Xfull, Xcol-Xcol2, 5.0*xcols*xcols*bias);
//        ct(3,0) += 1;
//        ct(4,1) += 1;
//        ct(5,2) += 1;
//        transformed_colors = Xfull*ct;

//        test(W, transformed_colors, Xcol);

//        //cout << "The solution using normal equations is:\n" <<  ct << endl;
//    }
//}
//exit(0);


///*
//				for(int rematching2=0; rematching2 < 3; ++rematching2) {
//                    if( (getTime()-start) > maxtime ){break;}
//					if(rematching2 != 0){
//#pragma omp parallel for
//						for(unsigned int i=0; i< xcols; ++i) {matchid[i] = tree->closest(X.col(i).data());}
//						/// Find closest point
//#pragma omp parallel for
//						for(unsigned int i=0; i< xcols; ++i) {
//							int id = matchid[i];
//							Qn.col(i) = N.col(id);
//							Qp.col(i) = Y.col(id);
//							rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
//						}
//					}

//					for(int inner=0; inner< 40; ++inner) {
//                        if( (getTime()-start) > maxtime ){break;}
//						if(inner != 0){
//							switch(type) {
//								case PointToPoint:	{residuals = X-Qp;} 						break;
//								case PointToPlane:	{
//									residuals		= Eigen::MatrixXd::Zero(1,	xcols);
//									for(int i=0; i< xcols; ++i) {
//										float dx = X(0,i)-Qp(0,i);
//										float dy = X(1,i)-Qp(1,i);
//										float dz = X(2,i)-Qp(2,i);
//										float qx = Qn(0,i);
//										float qy = Qn(1,i);
//                                        float qz = Qn(2,i);
//										float di = qx*dx + qy*dy + qz*dz;
//										residuals(0,i) = di;
//									}
//								}break;
//								default:			{printf("type not set\n");}					break;
//							}
//							for(int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}
//						}

//						switch(type) {
//							case PointToPoint:	{W = func->getProbs(residuals); } 					break;
//							case PointToPlane:	{
//								W = func->getProbs(residuals);
//								for(int i=0; i<xcols; ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
//							}	break;
//							default:			{printf("type not set\n");} break;
//						}
//						Wold = W;

//						//Normalizing weights has an effect simmilar to one to one matching
//						//in that it reduces the effect of border points
//						if(normalize_matchweights){
//							for(unsigned int i=0; i < ycols; ++i)	{	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
//							for(unsigned int i=0; i< xcols; ++i)	{	total_dweight[matchid[i]] += W(i);}
//							for(unsigned int i=0; i< xcols; ++i)	{	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
//						}

//						W = W.array()*rangeW.array()*rangeW.array();
//						Eigen::Affine3d change;
//						switch(type) {
//							case PointToPoint:	{RigidMotionEstimator::point_to_point(X, Qp, W);}		break;
//                            case PointToPlane:	{change = RigidMotionEstimatorRefinementColor::point_to_plane(X, Xn, Qp, Qn, W);}	break;
//							default:  			{printf("type not set\n"); } break;
//						}

//						stop = 100*func->getConvergenceThreshold();
//						//score = Wold.sum()/(pow(func->getNoise(),2)*float(xcols));
//						score = Wold.sum()/float(xcols);
//						double stop1 = (X-Xo1).colwise().norm().mean();
//						Xo1 = X;
//						if(stop1 < stop) break;
//					}
//					double stop2 = (X-Xo2).colwise().norm().mean();
//					Xo2 = X;
//					if(stop2 < stop) break;
//				}
//				double stop3 = (X-Xo3).colwise().norm().mean();
//				Xo3 = X;
//                if(stop3 < stop) break;*/
//			}
//            /*
//			double stop4 = (X-Xo4).colwise().norm().mean();
//			Xo4 = X;
//			if(stop4 < stop) break;
//            */
//		}
//        /*
//		double noise_before = func->getNoise();
//		func->update();
//		double noise_after = func->getNoise();
//		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
//*/
//	}

//   // if(visualizationLvl >= 2){show(X,Y);}

//	pcl::TransformationFromCorrespondences tfc;
//	tfc.reset();
//	for(unsigned int i = 0; i < xcols; i++){
//		Eigen::Vector3f a (X(0,i),						X(1,i),					X(2,i));
//		Eigen::Vector3f b (src->data(0,i*stepx),		src->data(1,i*stepx),	src->data(2,i*stepx));
//		tfc.add(b,a);
//	}
//	guess = tfc.getTransformation().matrix().cast<double>();
//    FusionResults fr = FusionResults(guess,stop);
//	return fr;

}

}
