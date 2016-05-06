#include "registration/RegistrationRandom.h"
#include <iostream>
#include <fstream>

namespace reglib
{

RegistrationRandom::RegistrationRandom(){
	only_initial_guess		= false;
	visualizationLvl = 1;
	refinement = new RegistrationRefinement();
}
RegistrationRandom::~RegistrationRandom(){
	delete refinement;
}

void RegistrationRandom::setSrc(CloudData * src_){
	src = src_;
	refinement->setSrc(src_);
}
void RegistrationRandom::setDst(CloudData * dst_){
	dst = dst_;
	refinement->setDst(dst_);
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
	//printf("s_nr_data: %i d_nr_data: %i\n",s_nr_data,d_nr_data);

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
	//nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>                  kdtree(Y);

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

	std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > all_X;
	std::vector< Eigen::Affine3d > all_res;
	std::vector< int > count_X;
	std::vector< float > score_X;
    std::vector< std::vector< Eigen::VectorXd > > all_starts;
	int stepxsmall = std::max(1,int(s_nr_data)/250);
	Eigen::VectorXd Wsmall (s_nr_data/stepxsmall);
	for(unsigned int i = 0; i < s_nr_data/stepxsmall; i++){Wsmall(i) = src->information(0,i*stepxsmall);}


	double sumtime = 0;
	double sumtimeSum = 0;
	double sumtimeOK = 0;
	int r = 0;

    refinement->viewer = viewer;
	refinement->visualizationLvl = 0;
	//for(int r = 0; r < 1000; r++){
//	while(true){
//		double rx = 2.0*M_PI*0.0001*double(rand()%10000);
//		double ry = 2.0*M_PI*0.0001*double(rand()%10000);
//		double rz = 2.0*M_PI*0.0001*double(rand()%10000);
    //double stop = 0;
	double step = 0.1+2.0*M_PI/5;
	for(double rx = 0; rx < 2.0*M_PI; rx += step){
	for(double ry = 0; ry < 2.0*M_PI; ry += step)
	for(double rz = 0; rz < 2.0*M_PI; rz += step){
		printf("rx: %f ry: %f rz: %f\n",rx,ry,rz);

		double start = getTime();

		double meantime = 999999999999;
		if(r != 0){meantime = sumtimeSum/double(sumtimeOK+1.0);}
		refinement->maxtime = std::min(0.5,3*meantime);

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
//fr.timeout = timestopped;
        double stoptime = getTime();
        sumtime += stoptime-start;

		if(!fr.timeout){
			sumtimeSum += stoptime-start;
			sumtimeOK++;
		}

		//printf("sumtime: %f\n",sumtime);
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
				float score = score_X[ax];
                std::vector< Eigen::VectorXd > starts = all_starts[ax];
				for(int bx = ax-1; bx >= 0; bx--){
					if(count_X[bx] < count_X[bx+1]){
						count_X[bx+1]		= count_X[bx];
						score_X[bx+1]		= score_X[bx];
						all_X[bx+1]			= all_X[bx];
						all_starts[bx+1]	= all_starts[bx];
						all_res[bx+1]		= all_res[bx];


						all_X[bx] = axX;
						count_X[bx] = count;
						score_X[bx] = score;
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
			score_X.push_back(stop);
            all_starts.push_back(std::vector< Eigen::VectorXd >());
            all_starts.back().push_back(startparam);
			all_res.push_back(current_guess);
		}
        r++;
	}
}


	FusionResults fr = FusionResults();
    refinement->allow_regularization = false;

	int tpbef = refinement->target_points;
	refinement->target_points = 2000;
	for(unsigned int ax = 0; ax < all_X.size(); ax++){
        Eigen::Matrix4d np = all_res[ax].matrix();
		refinement->visualizationLvl = visualizationLvl;
		if(ax < 25){
			double start = getTime();
			FusionResults fr1 = refinement->getTransform(np);
			double stop = getTime();
			np = fr1.guess;
		}
		refinement->visualizationLvl = 0;
        fr.candidates.push_back(np);
		fr.counts.push_back(count_X[ax]);
		fr.scores.push_back(1.0/score_X[ax]);
	}
	refinement->target_points = tpbef;
	return fr;
}

}
