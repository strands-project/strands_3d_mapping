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

	refinement = new RegistrationRefinement();
}
RegistrationRandom::~RegistrationRandom(){}


void RegistrationRandom::setSrc(CloudData * src_){
	src = src_;
	refinement->setSrc(src_);
}
void RegistrationRandom::setDst(CloudData * dst_){
	dst = dst_;
	refinement->setDst(dst_);
}


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

double transformationdiff(Eigen::Affine3d & A, Eigen::Affine3d & B, double rotationweight){
	Eigen::Affine3d C = A.inverse()*B;
	double r = fabs(1-C(0,0))+fabs(C(0,1))+fabs(C(0,2))  +  fabs(C(1,0))+fabs(1-C(1,1))+fabs(C(1,2))  +  fabs(C(2,0))+fabs(C(2,1))+fabs(1-C(2,2));
	double t = sqrt(C(0,3)*C(0,3)+C(1,3)*C(1,3)+C(2,3)*C(2,3));
	return r*rotationweight+t;
}

FusionResults RegistrationRandom::getTransform(Eigen::MatrixXd guess){

	unsigned int s_nr_data = src->data.cols();//std::min(int(src->data.cols()),int(500000));
	unsigned int d_nr_data = dst->data.cols();
    refinement->allow_regularization = true;
	printf("s_nr_data: %i d_nr_data: %i\n",s_nr_data,d_nr_data);

	int stepy = std::max(1,int(d_nr_data)/100000);

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

	printf("%f %f %f\n",d_mean_x,d_mean_y,d_mean_z);

	double stop		= 0.00001;

	Eigen::Affine3d Ymean = Eigen::Affine3d::Identity();
	Ymean(0,3) = d_mean_x;
	Ymean(1,3) = d_mean_y;
	Ymean(2,3) = d_mean_z;

	Eigen::Affine3d Xmean = Eigen::Affine3d::Identity();
	Xmean(0,3) = s_mean_x;
	Xmean(1,3) = s_mean_y;
	Xmean(2,3) = s_mean_z;

	std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > all_X;
	std::vector< Eigen::Affine3d > all_res;
	std::vector< int > count_X;
    std::vector< std::vector< Eigen::VectorXd > > all_starts;
	int stepxsmall = std::max(1,int(s_nr_data)/250);
	Eigen::VectorXd Wsmall (s_nr_data/stepxsmall);
	for(unsigned int i = 0; i < s_nr_data/stepxsmall; i++){Wsmall(i) = src->information(0,i*stepxsmall);}


	double sumtime = 0;
	int r = 0;

    refinement->viewer = viewer;
    refinement->visualizationLvl = 1;
	//for(int r = 0; r < 1000; r++){
//	while(true){
//		double rx = 2.0*M_PI*0.0001*double(rand()%10000);
//		double ry = 2.0*M_PI*0.0001*double(rand()%10000);
//		double rz = 2.0*M_PI*0.0001*double(rand()%10000);
    //double stop = 0;
	double step = 2.0;
	for(double rx = 0; rx < 2.0*M_PI; rx += step){
	for(double ry = 0; ry < 2.0*M_PI; ry += step)
	for(double rz = 0; rz < 2.0*M_PI; rz += step){
		//printf("rx: %f\n",rx);

		double start = getTime();

		double meantime = 999999999999;
		if(r != 0){meantime = sumtime/double(r+1);}
        refinement->maxtime = 3*meantime;

        Eigen::VectorXd startparam = Eigen::VectorXd(3);
        startparam(0) = rx;
        startparam(1) = ry;
        startparam(2) = rz;

		Eigen::Affine3d randomrot = Eigen::Affine3d::Identity();

		randomrot =	Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
					Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());

		Eigen::Affine3d current_guess = Ymean*randomrot*Xmean.inverse();//*Ymean;
        refinement->target_points = 250;
        FusionResults fr = refinement->getTransform(current_guess.matrix());

        double stoptime = getTime();
        sumtime += stoptime-start;
        stop = fr.score;
        Eigen::Matrix4d m = fr.guess;
        current_guess = m;//fr.guess;
        float m00 = current_guess(0,0); float m01 = current_guess(0,1); float m02 = current_guess(0,2); float m03 = current_guess(0,3);
        float m10 = current_guess(1,0); float m11 = current_guess(1,1); float m12 = current_guess(1,2); float m13 = current_guess(1,3);
        float m20 = current_guess(2,0); float m21 = current_guess(2,1); float m22 = current_guess(2,2); float m23 = current_guess(2,3);

		Eigen::Matrix<double, 3, Eigen::Dynamic> Xsmall;
		Xsmall.resize(Eigen::NoChange,s_nr_data/stepxsmall);
		for(unsigned int i = 0; i < s_nr_data/stepxsmall; i++){
			float x		= src->data(0,i*stepxsmall);
			float y		= src->data(1,i*stepxsmall);
			float z		= src->data(2,i*stepxsmall);
			Xsmall(0,i)	= m00*x + m01*y + m02*z + m03;
			Xsmall(1,i)	= m10*x + m11*y + m12*z + m13;
			Xsmall(2,i)	= m20*x + m21*y + m22*z + m23;
		}

		bool exists = false;
		for(unsigned int ax = 0; ax < all_X.size(); ax++){
			Eigen::Matrix<double, 3, Eigen::Dynamic> axX = all_X[ax];
			Eigen::Affine3d axT = all_res[ax];
			double diff = (Xsmall-axX).colwise().norm().mean();
			if(diff < 20*stop){
				count_X[ax]++;
                all_starts[ax].push_back(startparam);
				int count = count_X[ax];
                std::vector< Eigen::VectorXd > starts = all_starts[ax];
				for(int bx = ax-1; bx >= 0; bx--){
					if(count_X[bx] < count_X[bx+1]){
						count_X[bx+1]		= count_X[bx];
						all_X[bx+1]			= all_X[bx];
						all_starts[bx+1]	= all_starts[bx];
						all_res[bx+1]		= all_res[bx];

                        all_X[bx] = axX;
                        count_X[bx] = count;
                        all_starts[bx] = starts;
						all_res[bx] = axT;
					}else{break;}
				}
				exists = true;
				break;
			}
		}



		if(!exists){
			all_X.push_back(Xsmall);
			count_X.push_back(1);
            all_starts.push_back(std::vector< Eigen::VectorXd >());
            all_starts.back().push_back(startparam);
			all_res.push_back(current_guess);
		}
//}
        r++;
		//if(sumtime > 3){break;}

	}
}
	printf("sumtime: %f\n",sumtime);
    refinement->maxtime = 9999999999;


/*
	for(unsigned int ax = 0; ax < all_X.size() && ax < 5; ax++){
		printf("%i -> %i\n",ax,count_X[ax]);
		if(visualizationLvl >= 2){show(all_X[ax],Y);}
	}
*/
/*
	for(unsigned int ax = 0; ax < all_X.size(); ax++){
		printf("%i -> %i\n",ax,count_X[ax]);

		show(all_X[ax],Y);

		std::vector< Eigen::VectorXd > axstarts = all_starts[ax];

		myhull * mh = new myhull();
		//for(double alpha = 0.1; alpha <= 300; alpha += 0.1){
			double alpha = 0.5;
			mh->compute(axstarts,alpha);


			viewer->removeAllPointClouds();
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			for(unsigned int bx = 0; bx < all_X.size(); bx++){
				std::vector< Eigen::VectorXd > bxstarts = all_starts[bx];
				for(unsigned int i = 0; i < bxstarts.size(); i++){
					Eigen::VectorXd p2 = bxstarts[i];
					pcl::PointXYZRGBNormal p;
					p.x = p2(0);
					p.y = p2(1);
					p.z = p2(2);
					if(ax == bx){
						p.b = 0;
						p.g = 0;
						p.r = 255;
					}else{
						p.b = 0;
						p.g = 255;
						p.r = 0;
					}
					scloud->points.push_back(p);
				}
			}
			viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scloud");
			viewer->spin();
			viewer->removeAllPointClouds();

			viewer->addPolygonMesh<pcl::PointXYZ> (mh->cloud_hull, mh->polygons);
			viewer->spin();
			viewer->removeAllShapes();

			int inliers = 0;
			int outliers = 0;
			for(unsigned int bx = 0; bx < all_X.size(); bx++){
				if(bx == ax){continue;}

				std::vector< Eigen::VectorXd > bxstarts = all_starts[bx];

				for(unsigned int cx = 0; cx < bxstarts.size(); cx++){
					Eigen::VectorXd p = bxstarts[cx];
					bool inlier = mh->isInlier( p );
					inliers += inlier;
					outliers += !inlier;
				}
				//mh->inside(all_starts[ax]);
			}
			printf("hull: %i alpha: %f -> inliers: %i outliers: %i\n",ax,alpha,inliers,outliers);

		//}
        if(visualizationLvl >= 2){show(all_X[ax],Y);}
	}
*/

	FusionResults fr = FusionResults();
    refinement->allow_regularization = false;

	for(unsigned int ax = 0; ax < all_X.size(); ax++){
        Eigen::Matrix4d np = all_res[ax].matrix();
/*
        int tp = 250;
        while(tp < s_nr_data){
            tp *= 2;
            refinement->target_points = tp;
            //refinement->visualizationLvl = 2;
            double start = getTime();
            FusionResults fr = refinement->getTransform(np);
            double stop = getTime();
            printf("refinement cost: %fs\n",stop-start);
            np = fr.guess;
            //break;
        }
*/

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
