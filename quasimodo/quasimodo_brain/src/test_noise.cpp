#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <sys/time.h>

#include "../../quasimodo_models/include/modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include "core/Util.h"
#include <random>

#include <omp.h>

using namespace std;
using namespace Eigen;
using namespace reglib;

default_random_engine generator;
Matrix3Xd conc(Matrix3Xd A, Matrix3Xd B){
	Eigen::Matrix3Xd C	= Eigen::Matrix3Xd::Zero(3,	A.cols()+B.cols());
	C << A, B;
	return C;
}



//	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	X.cols());


Matrix3Xd getPoints(int cols){
	Eigen::Matrix3Xd points	= Eigen::Matrix3Xd::Zero(3,cols);
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < cols; j++){
			points(i,j) = (rand()%10000)*0.0001;
		}
	}
	return points;
}

Matrix3Xd getMeasurements(Matrix3Xd gt, double noise){
	int cols = gt.cols();

	normal_distribution<double> distribution(0,noise);
	Eigen::Matrix3Xd points	= Eigen::Matrix3Xd::Zero(3,cols);
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < cols; j++){
			points(i,j) = gt(i,j) + distribution(generator);
		}
	}
	return points;
}


Matrix3Xd getMeasurementsOffseted(Matrix3Xd gt, double noise){
	int cols = gt.cols();
	//printf("getMeasurementsOffseted %i and %f\n",cols,noise);
	Eigen::Matrix3Xd points	= Eigen::Matrix3Xd::Zero(3,cols);
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < cols; j++){
			points(i,j) = gt(i,j) + noise*(2.0*(rand()%10000)*0.0001 - 0.5);
		}
	}
	return points;
}

Matrix3Xd getMeasurementsOffseted(Matrix3Xd gt, double offset, double noise){
	int cols = gt.cols();
	normal_distribution<double> distribution(0,offset);

	Eigen::Matrix3Xd points	= Eigen::Matrix3Xd::Zero(3,cols);
	for(int i = 0; i < 3; i++){
		double o = distribution(generator);
		for(int j = 0; j < cols; j++){
			points(i,j) = gt(i,j) + o + noise*(2.0*(rand()%10000)*0.0001 - 1.0);
		}
	}
	return points;
}

Matrix3Xd transform_points(Matrix3Xd points, Matrix4d transform){
	int cols = points.cols();

	float m00 = transform(0,0); float m01 = transform(0,1); float m02 = transform(0,2); float m03 = transform(0,3);
	float m10 = transform(1,0); float m11 = transform(1,1); float m12 = transform(1,2); float m13 = transform(1,3);
	float m20 = transform(2,0); float m21 = transform(2,1); float m22 = transform(2,2); float m23 = transform(2,3);

	Eigen::Matrix3Xd points2	= Eigen::Matrix3Xd::Zero(3,cols);
	for(int i = 0; i < cols; i++){
		float x	= points(0,i);
		float y	= points(1,i);
		float z	= points(2,i);
		points2(0,i)	= m00*x + m01*y + m02*z + m03;
		points2(1,i)	= m10*x + m11*y + m12*z + m13;
		points2(2,i)	= m20*x + m21*y + m22*z + m23;
	}
	return points2;
}

double get_reconst_rms(Matrix3Xd gt, Matrix3Xd points, int nr_points){
	double rms = 0;
	for(int i = 0; i < nr_points; i++){
		float dx	= gt(0,i) - points(0,i);
		float dy	= gt(1,i) - points(1,i);
		float dz	= gt(2,i) - points(2,i);
		rms += dx*dx+dy*dy+dz*dz;
	}
	rms/= double(nr_points);
	rms = sqrt(rms);
	return rms;
}

void align(DistanceWeightFunction2 * func, Eigen::Matrix3Xd & X, Eigen::Matrix3Xd & Qp, double stop = 0.00001,bool debugg = false){
	//printf("\n");
	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;

	Eigen::MatrixXd residuals;
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	X.cols());
	func->reset();

	bool breaknext = false;
	for(int i=0; i< 50; ++i) {
		for(int j=0; j< 10; ++j) {
			residuals = X-Qp;
			func->computeModel(residuals);

			for(int k=0; k< 30; ++k) {
				if(k != 0){residuals = X-Qp;}
				W = func->getProbs(residuals);

				int count = 0;for(int k=0; k < W.rows(); ++k) {count += W(k) > 0.0000001;}if(count < 3){break;}

				RigidMotionEstimator::point_to_point(X, Qp, W);
				double stop1 = (X-Xo1).colwise().norm().maxCoeff();
				Xo1 = X;
				stop = func->getConvergenceThreshold();

				if(debugg){
					printf("i:%i j:%i k:%i ",i,j,k);
					printf("stop: %10.10f stop1: %10.10f\n",stop,stop1);
				}
				if(stop1 < stop) break;
			}
			double stop2 = (X-Xo2).colwise().norm().maxCoeff();
			Xo2 = X;
			if(stop2 < stop) break;
		}

		bool breaknext_tmp = func->update();
		if(breaknext && breaknext_tmp){break;}
		//if(breaknext_tmp){break;}
		breaknext = breaknext_tmp;
		//double stop3 = (X-Xo3).colwise().norm().maxCoeff();
		//Xo3 = X;
		//if(stop3 < stop) break;
	}
	//printf("stop align\n");
}
/*
double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}
*/
double phi(double x)
{
	// constants
	double a1 =  0.254829592;
	double a2 = -0.284496736;
	double a3 =  1.421413741;
	double a4 = -1.453152027;
	double a5 =  1.061405429;
	double p  =  0.3275911;

	// Save the sign of x
	int sign = 1;
	if (x < 0)
		sign = -1;
	x = fabs(x)/sqrt(2.0);

	// A&S formula 7.1.26
	double t = 1.0/(1.0 + p*x);
	double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

	return 0.5*(1.0 + sign*y);
}

void blurHistogram(std::vector<float> & blur_hist,  std::vector<float> & hist, float stdval,bool debugg_print = false){
	int nr_data = blur_hist.size();
	int nr_data2 = 2*nr_data;

	std::vector<float> tmphist;
	tmphist.resize(nr_data2);

	std::vector<float> tmphist_blur;
	tmphist_blur.resize(nr_data2);

	for(int i = 0; i < nr_data; i++){
		tmphist[i]			= hist[nr_data-1-i];
		tmphist[nr_data+i]	= hist[i];

		tmphist_blur[i]			= 0;
		tmphist_blur[nr_data+i]	= 0;
	}

	double info = -0.5/(stdval*stdval);

	double weights[nr_data2];
	for(int i = 0; i < nr_data2; i++){
		double current = exp(i*i*info);
		weights[i] = current;
	}

	int offset = 4.0*stdval;
	offset = std::max(4,offset);

	for(int i = 0; i < nr_data2; i++){
		double v = tmphist[i];
		if(v == 0){continue;}

		int start	= std::max(0,i-offset);
		int stop	= std::min(nr_data2,i+offset+1);

		//double sum = 1;
		//for(int j = start; j < stop; j++){sum += weights[abs(i-j)];}
		for(int j = start; j < stop; j++){
			double w = weights[abs(i-j)];
			tmphist_blur[j] += v*w;///sum;
		}
	}
	for(int i = 0; i < nr_data; i++){
		blur_hist[i] = tmphist_blur[i+nr_data];
	}

	float bef = 0;
	float aft = 0;
	for(int i = 0; i < nr_data; i++){
		bef += hist[i];
		aft += blur_hist[i];
	}

	for(int i = 0; i < nr_data; i++){
		blur_hist[i] *= bef/aft;
	}
}


int main(int argc, char **argv){
	int size = 100;
	int cols = 1000;

	double overall_scale = 1.0;
	double noise = overall_scale*0.01;
	//double ratio_of_outliers = 10.0;
	double ratio_of_outliers = 10.0;
	double sample_area = 100;
	int nr_outliers = cols*ratio_of_outliers;

	DistanceWeightFunction2 * L2 = new DistanceWeightFunction2();
	L2->f = NONE;

	//vector<double> sum;
	vector<DistanceWeightFunction2 *> funcs;
//	funcs.push_back(new DistanceWeightFunction2());
//	funcs.back()->f = PNORM;	funcs.back()->p = 2.0;			funcs.back()->convergence_threshold = 0.00001*noise;
//	funcs.push_back(new DistanceWeightFunction2());
//	funcs.back()->f = PNORM;	funcs.back()->p = 0.1;			funcs.back()->convergence_threshold = 0.00001*noise;
//	funcs.push_back(new DistanceWeightFunction2());
//	funcs.back()->f = THRESHOLD; funcs.back()->p = noise*4; funcs.back()->convergence_threshold = 0.0001*noise;
//	funcs.push_back(new DistanceWeightFunction2());
//	funcs.back()->f = THRESHOLD; funcs.back()->p = noise*3; funcs.back()->convergence_threshold = 0.0001*noise;


	/*
	DistanceWeightFunction2PPR * ppr = new DistanceWeightFunction2PPR(2,1000);
	ppr->startreg			= 0;
	//ppr2->blurval			= i;
	ppr->stdval				= 100;//ppr->blurval;
	ppr->stdgrow			= 1.0;
	ppr->blur				= 0.055;
	ppr->noiseval			= 100.0;
	ppr->debugg_print		= false;
	ppr->threshold			= false;
	ppr->uniform_bias		= false;
	ppr->convergence_threshold = 0.05;
	ppr->scale_convergence	= true;
	ppr->update_size = true;
	ppr->noiseval = ppr->maxd;
	ppr->meanoffset = std::max(0.0,(ppr->maxd - ppr->regularization - ppr->noiseval)/ppr->target_length);
	funcs.push_back(ppr);
*/
/*
	std::vector<double> sr;
	//sr.push_back(0);
	//sr.push_back(1.0);
	//sr.push_back(0.1);
	//sr.push_back(1.0);
	for(unsigned int i = 0; i < sr.size(); i++){
		for(double  j = 0.06; j <= 0.06; j += 0.01){
			for(double  k = 1.00; k <= 1.00; k += 0.5){
				for(int dpb = 10; dpb <= 10; dpb += 30){
					for(int tl = 8; tl <= 8; tl += 2){
						for(int state = 0; state < 1; state++){
							DistanceWeightFunction2PPR * ppr2 = new DistanceWeightFunction2PPR(2,1000);

							ppr2->target_length = tl;
							ppr2->data_per_bin = dpb;

							ppr2->startreg			= sr[i];
							ppr2->stdval			= 100;//ppr->blurval;
							ppr2->stdgrow			= 1.0;
							ppr2->blur              = j;
							ppr2->noiseval			= 100.0;
							ppr2->debugg_print		= false;
							ppr2->threshold			= false;
							ppr2->uniform_bias		= false;
							ppr2->convergence_threshold = k;
							ppr2->scale_convergence	= true;

							ppr2->max_under_mean	= (state&1 != 0);
							ppr2->interp			= (state&2 != 0);

							ppr2->update_size = true;
							ppr2->noiseval = ppr2->maxd;
							ppr2->meanoffset = std::max(0.0,(ppr2->maxd - ppr2->regularization - ppr2->noiseval)/ppr2->target_length);
							//funcs.push_back(ppr2);
						}
					}
				}
			}
		}
	}
*/
	bool debugg = false;

	std::vector<double> nsr;
	nsr.push_back(0.0);
	nsr.push_back(0.01);
	nsr.push_back(0.1);
	nsr.push_back(1.0);

	for(unsigned int s = 0; s < nsr.size(); s++){
		for(double cp = 2; cp < 3; cp += 1)
		for(unsigned int dpb = 30; dpb < 40; dpb += 10){
			//for(double blur = 0.03; blur < 0.04; blur += 0.01){
			for(double blur = 0.03; blur < 0.04; blur += 0.01){
				for(int setting = 0; setting < 1; setting++){
					//DistanceWeightFunction2PPR2 * ppr = new DistanceWeightFunction2PPR2(1.0,1000);
					DistanceWeightFunction2PPR2 * ppr = new DistanceWeightFunction2PPR2(1.0,1000);
					ppr->startreg			= nsr[s];
					ppr->blurval			= 4;
					ppr->stdval				= 100;
					ppr->stdgrow			= 1.0;
					ppr->noiseval			= 100.0;
					ppr->debugg_print		= false;//true;
					ppr->threshold			= false;
					ppr->scale_convergence	= true;
					ppr->blur				= blur;
					ppr->data_per_bin		= dpb;
					ppr->costpen			= cp;

					ppr->uniform_bias		= false;//(setting&4 != 0);
					ppr->refine_mean		= false;//(setting&2 != 0);
					ppr->refine_mul			= false;//(setting&1 != 0);
					ppr->update_size		= true;//(setting&2 != 1);
					ppr->max_under_mean		= true;//(setting&1 != 0);
					ppr->rescaling			= false;//(setting&1 != 0);
					ppr->interp				= true;//(setting&1 != 0);
					ppr->refine_std			= true;
					ppr->nr_refineiters		= 1;
					ppr->convergence_threshold = 0.05;
					funcs.push_back(ppr);
				}
			}
		}
	}


//exit(0);
	vector< Matrix3Xd > gt;
	vector< Matrix3Xd > measurements;
	for(int i = 0; i < size; i++){gt.push_back(getPoints(cols));}
	for(int i = 0; i < size; i++){measurements.push_back(getMeasurements(gt[i],noise));}



	vector<Matrix4d> transformations;

	for(int i = 0; i <= 100; i+=100){
		double angle1 = double(0);double angle2 = 0;double angle3 = 0;
		double t1 = double(i*0.01);double t2 = 0;	double t3 = 0;

		//printf("\%\% transformation %i -> angle: %f %f %f translation: %f %f %f\n",i,angle1,angle2,angle3,t1,t2,t3);
		Matrix3d rotmat;
		rotmat = AngleAxisd(angle1, Vector3d::UnitZ()) * AngleAxisd(angle2, Vector3d::UnitY()) * AngleAxisd(angle3, Vector3d::UnitZ());
		Matrix4d transformation = Matrix4d::Identity();
		transformation.block(0,0,3,3) = rotmat;
		transformation(0,3) = t1; transformation(1,3) = t2; transformation(2,3) = t3;

		transformations.push_back(transformation);
	}

	for(int i = 0; i <= 100; i+=100){
		double angle1 = 0.01*double(i)*M_PI;double angle2 = 0;double angle3 = 0;
		double t1 = double(0);double t2 = 0;	double t3 = 0;

		printf("\%\% transformation %i -> angle: %f %f %f translation: %f %f %f\n",i,angle1,angle2,angle3,t1,t2,t3);
		Matrix3d rotmat;
		rotmat = AngleAxisd(angle1, Vector3d::UnitZ()) * AngleAxisd(angle2, Vector3d::UnitY()) * AngleAxisd(angle3, Vector3d::UnitZ());
		Matrix4d transformation = Matrix4d::Identity();
		transformation.block(0,0,3,3) = rotmat;
		transformation(0,3) = t1; transformation(1,3) = t2; transformation(2,3) = t3;

		transformations.push_back(transformation);
	}

	double optimal = 0;
	vector< vector< double > > results;
	vector< vector< double > > times;
	results.resize(funcs.size());
	times.resize(funcs.size());
	for(unsigned int j = 0; j < funcs.size(); j++){
		results[j].resize(transformations.size());
		times[j].resize(transformations.size());
		for(unsigned int k = 0; k < transformations.size(); k++){
			results[j][k] = 0;
			times[j][k] = 0;
		}
	}

	vector< vector< vector< double > > > all_results;
	vector< vector< vector< double > > > all_times;
	all_results.resize(funcs.size());
	all_times.resize(funcs.size());
	for(unsigned int j = 0; j < funcs.size(); j++){
		all_results[j].resize(transformations.size());
		all_times[j].resize(transformations.size());
		for(unsigned int k = 0; k < transformations.size(); k++){
			all_results[j][k].resize(size);
			all_times[j][k].resize(size);
			for(int i = 0; i < size; i++){
				all_results[j][k][i] = 0;
				all_times[j][k][i] = 0;
			}
		}
	}

	int nr_funcs = funcs.size();
	for(int i = 0; i < size; i++){
		//printf("\%\% %i / %i\n",i,size);
		Matrix3Xd measurements_tmp	= overall_scale * measurements[i];
		Matrix3Xd gt_tmp			= overall_scale * gt[i];
		//printf("start L2:\n");
		align(L2, measurements_tmp, gt_tmp);
		//printf("stop  L2:\n");
		double opt_rms = get_reconst_rms(gt_tmp, measurements_tmp, cols)/sqrt(3.0) / overall_scale;
		//printf("opt rms: %6.6f\n",opt_rms);
		optimal += opt_rms;

		Matrix3Xd gt_outliers			= getPoints(nr_outliers);
		Matrix3Xd measurements_outliers	= getMeasurementsOffseted(gt_outliers, noise*sample_area);

		//Matrix3Xd gt_full			= gt_outliers;//conc(gt[i],gt_outliers);
		//Matrix3Xd measurements_full	= measurements_outliers;//conc(measurements[i],measurements_outliers);
		Matrix3Xd gt_full			= overall_scale*conc(gt[i],gt_outliers);
		Matrix3Xd measurements_full	= conc(measurements[i],measurements_outliers);


		for(unsigned int k = 0; k < transformations.size(); k++){
			Matrix3Xd measurements_full_trans = transform_points(measurements_full, transformations[k]);

			//#pragma omp parallel for num_threads(7)
			for(int j = 0; j < nr_funcs; j++){
				//if(debugg){if(k < 1 || i != 8){continue;}}
				//if(i < 2){continue;}
				//if(i < 91){continue;}
				//if(i > 2){continue;}
				Matrix3Xd measurements_full_tmp = overall_scale*measurements_full_trans;

				printf("%i %i %i / %i %i %i --> ",i,k,j,size,transformations.size(),funcs.size());
				//cout << measurements_full_tmp << endl << endl;
				double start = getTime();
				//if(i == 93){
				//	funcs[j]->debugg_print = true;
				//	align(funcs[j], measurements_full_tmp, gt_full,0.00001,true);}
				//else{align(funcs[j], measurements_full_tmp, gt_full,0.00001,debugg);}
				align(funcs[j], measurements_full_tmp, gt_full,0.00001,debugg);
				double stop = getTime()-start;
				//cout << measurements_full_tmp << endl << endl;

				double rms = get_reconst_rms(gt_full, measurements_full_tmp, cols)/sqrt(3.0) / overall_scale;
				results[j][k] += rms-opt_rms;
				times[j][k] += stop;//-start;
				all_results[j][k][i] = rms-opt_rms;
				all_times[j][k][i] = stop;//-start;

				printf("rms: %12.12f\n",10000.0*(rms-opt_rms));
/*
				if(rms-opt_rms > 0.000001){
					measurements_full_tmp = overall_scale*measurements_full_trans;
					align(funcs[j], measurements_full_tmp, gt_full,0.00001,true);
					exit(0);
				}
*/
				//if(stop > 1){exit(0);}

				//printf("getNoise: %f\n",funcs[j]->getNoise());
				//printf("%f += %f - %f (%f)\n",times[j],stop,start,stop-start);
				//printf("%s diff from opt rms: %6.6f\n",funcs[j]->getString().c_str(),rms-opt_rms);
			}
		}
	}

	for(unsigned int j = 0; j < funcs.size(); j++){
		printf("%s_rms =[",funcs[j]->getString().c_str());
		for(unsigned int k = 0; k < transformations.size(); k++){
			printf("%4.4f ",10000.0*(results[j][k]));
		}printf("];\n");
	}

	for(unsigned int j = 0; j < funcs.size(); j++){
		printf("\%\% %i -> %s\n",j,funcs[j]->getString().c_str());
	}
/*
	printf("\%\% all_rms(funcs,transformations,testcase)\n");
	printf("all_rms = zeros(%i,%i,%i);\n",funcs.size(),transformations.size(),size);
	for(unsigned int j = 0; j < funcs.size(); j++){
		for(unsigned int k = 0; k < transformations.size(); k++){
			printf("all_rms(%i,%i,:) = [",j,k);//buf,funcs.size(),transformations.size(),size);
			for(int i = 0; i < size; i++){
				printf("%3.4f ",10000.0*(all_results[j][k][i]));
			}printf("];\n");
		}
	}
*/

	for(unsigned int j = 0; j < funcs.size(); j++){
		printf("%s_stab =[",funcs[j]->getString().c_str());
		for(unsigned int k = 0; k < transformations.size(); k++){
			double inl = 0;
			for(int i = 0; i < size; i++){
				inl += all_results[j][k][i] < 0.01;
			}

			printf("%5.5f ",inl/double(size));
		}printf("];\n");
	}


	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	for(unsigned int j = 0; j < funcs.size(); j++){
		printf("%s_time =[",funcs[j]->getString().c_str());
		for(unsigned int k = 0; k < transformations.size(); k++){
			printf("%3.3f ",times[j][k]);
		}printf("];\n");
	}


	return 0;
}
