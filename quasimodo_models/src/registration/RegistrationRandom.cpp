#include "RegistrationRandom.h"

#include "ICP.h"
#include "GoICP_V1.3/src/jly_goicp.h"

#include <iostream>
#include <fstream>

#include "myhull.h"

//#include <pcl/surface/convex_hull.h>

namespace reglib
{

RegistrationRandom::RegistrationRandom(){
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
}
RegistrationRandom::~RegistrationRandom(){}

namespace RigidMotionEstimatorRand {
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

double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

FusionResults RegistrationRandom::getTransform(Eigen::MatrixXd guess){

	unsigned int s_nr_data = std::min(int(src->data.cols()),int(500000));
	unsigned int d_nr_data = dst->data.cols();
	
	int stepx = std::max(1,int(s_nr_data)/500);
	int stepy = std::max(1,int(d_nr_data)/10000);

	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Xn;

	X.resize(Eigen::NoChange,s_nr_data/stepx);
	Xn.resize(Eigen::NoChange,s_nr_data/stepx);
	unsigned int xcols = X.cols();


	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(xcols);

	Eigen::Matrix<double, 3, Eigen::Dynamic> Y;
	Eigen::Matrix<double, 3, Eigen::Dynamic> N;
	Y.resize(Eigen::NoChange,d_nr_data/stepy);
	N.resize(Eigen::NoChange,d_nr_data/stepy);
	unsigned int ycols = Y.cols();

	for(unsigned int i = 0; i < d_nr_data/stepy; i++){
		Y(0,i)	= dst->data(0,i*stepy);
		Y(1,i)	= dst->data(1,i*stepy);
		Y(2,i)	= dst->data(2,i*stepy);
		N(0,i)	= dst->normals(0,i*stepy);
		N(1,i)	= dst->normals(1,i*stepy);
		N(2,i)	= dst->normals(2,i*stepy);
	}

	/// Build kd-tree
	nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>                  kdtree(Y);

	Eigen::VectorXd DST_INORMATION = Eigen::VectorXd::Zero(Y.cols());
	for(unsigned int i = 0; i < d_nr_data/stepy; i++){DST_INORMATION(i) = dst->information(0,i*stepy);}

	double s_mean_x = 0;
	double s_mean_y = 0;
	double s_mean_z = 0;
	for(unsigned int i = 0; i < s_nr_data; i++){
		s_mean_x += src->data(0,i);
		s_mean_y += src->data(1,i);
		s_mean_z += src->data(2,i);
	}
	s_mean_x /= double(s_nr_data);
	s_mean_y /= double(s_nr_data);
	s_mean_z /= double(s_nr_data);

	double d_mean_x = 0;
	double d_mean_y = 0;
	double d_mean_z = 0;
	for(unsigned int i = 0; i < d_nr_data; i++){
        d_mean_x += dst->data(0,i);
		d_mean_y += dst->data(1,i);
		d_mean_z += dst->data(2,i);
	}
	d_mean_x /= double(d_nr_data);
	d_mean_y /= double(d_nr_data);
	d_mean_z /= double(d_nr_data);


	double stop		= 0.00001;

	Eigen::Affine3d Ymean = Eigen::Affine3d::Identity();
	Ymean(0,3) = d_mean_x;
	Ymean(1,3) = d_mean_y;
	Ymean(2,3) = d_mean_z;

	Eigen::Affine3d Xmean = Eigen::Affine3d::Identity();
	Xmean(0,3) = s_mean_x;
	Xmean(1,3) = s_mean_y;
	Xmean(2,3) = s_mean_z;

	/// Buffers
	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	X.cols());

	std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > all_X;
	std::vector< int > count_X;
    std::vector< std::vector< Eigen::VectorXd > > all_starts;

	double sumtime = 0;
    int r = 0;
    //for(int r = 0; r < 1000; r++){

    //double stop = 0;
    double step = 1.0;
    for(double rx = 0; rx < 2.0*M_PI; rx += step)
    for(double ry = 0; ry < 2.0*M_PI; ry += step)
        for(double rz = 0; rz < 2.0*M_PI; rz += step){

		double start = getTime();

		double meantime = 999999999999;
		if(r != 0){meantime = sumtime/double(r+1);}

        Eigen::VectorXd startparam = Eigen::VectorXd(3);
        startparam(0) = rx;
        startparam(1) = ry;
        startparam(2) = rz;

		Eigen::Affine3d randomrot = Eigen::Affine3d::Identity();
        //double rx = 2.0*M_PI*0.001*double(rand()%1000);
        //double ry = 2.0*M_PI*0.001*double(rand()%1000);
        //double rz = 2.0*M_PI*0.001*double(rand()%1000);
		randomrot =	Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
					Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
		Eigen::Affine3d current_guess = Ymean*randomrot*Xmean.inverse();//*Ymean;

		float m00 = current_guess(0,0); float m01 = current_guess(0,1); float m02 = current_guess(0,2); float m03 = current_guess(0,3);
		float m10 = current_guess(1,0); float m11 = current_guess(1,1); float m12 = current_guess(1,2); float m13 = current_guess(1,3);
		float m20 = current_guess(2,0); float m21 = current_guess(2,1); float m22 = current_guess(2,2); float m23 = current_guess(2,3);

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

        //printf("%f %f %f\n",rx,ry,rz);
        //if(visualizationLvl >= 2){show(X,Y);}

		func->reset();

		for(unsigned int f = 0; f < feature_func.size(); f++){feature_func[f]->regularization = 15;}

		Eigen::Matrix3Xd Xo1 = X;
		Eigen::Matrix3Xd Xo2 = X;
		Eigen::Matrix3Xd Xo3 = X;
		Eigen::Matrix3Xd Xo4 = X;
		Eigen::MatrixXd residuals;

		std::vector<int> matchid;
		matchid.resize(xcols);

		std::vector<double> total_dweight;
		total_dweight.resize(ycols);

		double score = 0;
		stop = 99999;

		/// ICP
		for(int funcupdate=0; funcupdate < 10; ++funcupdate) {
			if( (getTime()-start) > (3*meantime) ){break;}
			for(int rematching=0; rematching < 10; ++rematching) {
				if( (getTime()-start) > (3*meantime) ){break;}

				#pragma omp parallel for
				for(unsigned int i=0; i< xcols; ++i) {matchid[i] = kdtree.closest(X.col(i).data());}

				/// Find closest point
				#pragma omp parallel for
				for(unsigned int i=0; i< xcols; ++i) {
					int id = matchid[i];
					Qn.col(i) = N.col(id);
					Qp.col(i) = Y.col(id);
					rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
				}



				for(int outer=0; outer< 1; ++outer) {
					if( (getTime()-start) > (3*meantime) ){break;}
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
						if( (getTime()-start) > (3*meantime) ){break;}
						if(rematching2 != 0){
							#pragma omp parallel for
							for(unsigned int i=0; i< xcols; ++i) {matchid[i] = kdtree.closest(X.col(i).data());}

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
							if( (getTime()-start) > (3*meantime) ){break;}
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
								case PointToPlane:	{change = RigidMotionEstimatorRand::point_to_plane(X, Xn, Qp, Qn, W);}	break;
								default:  			{printf("type not set\n"); } break;
							}

							stop = 100*func->getConvergenceThreshold();
							//score = Wold.sum()/(pow(func->getNoise(),2)*float(xcols));
							score = Wold.sum()/float(xcols);

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
		double stoptime = getTime();
		sumtime += stoptime-start;
		printf("score: %5.5f time to align: %5.5fs meantime: %5.5fs\n",score,stoptime-start,sumtime/double(r+1));

		bool exists = false;
		for(unsigned int ax = 0; ax < all_X.size(); ax++){
			Eigen::Matrix<double, 3, Eigen::Dynamic> axX = all_X[ax];
			double diff = (X-axX).colwise().norm().mean();
            if(diff < 20*stop){
				count_X[ax]++;
                all_starts[ax].push_back(startparam);

				int count = count_X[ax];
                std::vector< Eigen::VectorXd > starts = all_starts[ax];
				for(int bx = ax-1; bx >= 0; bx--){
					if(count_X[bx] < count_X[bx+1]){
                        count_X[bx+1]	= count_X[bx];
                        all_X[bx+1]		= all_X[bx];
                        all_starts[bx+1]		= all_starts[bx];

                        all_X[bx] = axX;
                        count_X[bx] = count;
                        all_starts[bx] = starts;
					}else{
						break;
					}
				}
				exists = true;
				break;
			}
		}

		if(!exists){
			all_X.push_back(X);
			count_X.push_back(1);
            all_starts.push_back(std::vector< Eigen::VectorXd >());
            all_starts.back().push_back(startparam);
		}
		//for(unsigned int ax = 0; ax < all_X.size(); ax++){printf("%i -> %i\n",ax,count_X[ax]);}
		printf("iterations: %i solutions %i\n",r+1,all_X.size());
        r++;
        //if(visualizationLvl >= 2){show(X,Y);}
        //if(sumtime > 6){break;}
	}
	printf("sumtime: %f\n",sumtime);

    for(unsigned int ax = 0; ax < all_X.size() && ax < 10000; ax++){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(all_starts[ax].size());
        for(unsigned int i = 0; i < all_starts[ax].size(); i++){
            cloud->points[i].x = all_starts[ax].at(i)(0);
            cloud->points[i].y = all_starts[ax].at(i)(1);
            cloud->points[i].z = all_starts[ax].at(i)(2);
        }

        //pcl::ConvexHull<pcl::PointXYZ> chull;
        //chull.setInputCloud (cloud);
        //chull.reconstruct (*cloud_hull);

		printf("%i -> %i\n",ax,count_X[ax]);
        if(visualizationLvl >= 2){show(all_X[ax],Y);}
	}

	FusionResults fr = FusionResults();

	pcl::TransformationFromCorrespondences tfc;
	for(unsigned int ax = 0; ax < all_X.size(); ax++){
		Eigen::Matrix<double, 3, Eigen::Dynamic> X = all_X[ax];
		tfc.reset();
		for(unsigned int i = 0; i < xcols; i++){
			Eigen::Vector3f a (X(0,i),						X(1,i),					X(2,i));
			Eigen::Vector3f b (src->data(0,i*stepx),		src->data(1,i*stepx),	src->data(2,i*stepx));
			tfc.add(b,a);
		}
		Eigen::Matrix4d np = tfc.getTransformation().matrix().cast<double>();
		fr.candidates.push_back(np);
		fr.counts.push_back(count_X[ax]);
		fr.scores.push_back(0);
	}
	return fr;
/*

	float score = Wold.sum()*pow(func->getNoise(),-1)/float(s_nr_data);

	tfc.reset();
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	np = tfc.getTransformation().matrix().cast<double>();
//exit(0);
	return FusionResults(np,score);
	*/
}

}
