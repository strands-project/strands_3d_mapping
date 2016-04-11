#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <sys/time.h>

#include "../../quasimodo_models/include/modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include "core/Util.h"
#include <random>

#include <omp.h>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"

using namespace std;
using namespace Eigen;
using namespace reglib;

using ceres::NumericDiffCostFunction;
using ceres::SizedCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::Solve;

default_random_engine generator;
Matrix3Xd conc(Matrix3Xd A, Matrix3Xd B){
	Eigen::Matrix3Xd C	= Eigen::Matrix3Xd::Zero(3,	A.cols()+B.cols());
	C << A, B;
	return C;
}
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

template <typename T> Eigen::Matrix<T,4,4> getMat(const T* const camera, int mode = 0){
	Eigen::Matrix<T,4,4> ret = Eigen::Matrix<T,4,4>::Identity();
	if(mode == 0){//yaw pitch roll tx ty tz
		Eigen::AngleAxis<T> yawAngle(camera[0], Eigen::Matrix<T,3,1>::UnitY());
		Eigen::AngleAxis<T> pitchAngle(camera[1], Eigen::Matrix<T,3,1>::UnitX());
		Eigen::AngleAxis<T> rollAngle(camera[2], Eigen::Matrix<T,3,1>::UnitZ());
		Eigen::Quaternion<T> q = rollAngle * yawAngle * pitchAngle;
		Eigen::Matrix<T,3,3> rotationMatrix = q.matrix();
		ret.block(0,0,3,3) = rotationMatrix;
		ret(0,3) = camera[3];
		ret(1,3) = camera[4];
		ret(2,3) = camera[5];
	}
	return ret;
}

struct PointError {
	double m1 [3];
	double m2 [3];
	double w;


	PointError(double m1x, double m1y, double m1z, double m2x, float m2y, double m2z, double w_){
		m1[0] = m1x;
		m1[1] = m1y;
		m1[2] = m1z;

		m2[0] = m2x;
		m2[1] = m2y;
		m2[2] = m2z;
		w = w_;
	}

	template <typename T> bool operator()(const T* const camera1, const T* camera2, T* residuals) const {
		T m1T[3];
		m1T[0] = T(m1[0]);
		m1T[1] = T(m1[1]);
		m1T[2] = T(m1[2]);
		T p1[3];
		ceres::AngleAxisRotatePoint(camera1, m1T, p1);
		p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

		T p2[3];
		T m2T[3];
		m2T[0] = T(m2[0]);
		m2T[1] = T(m2[1]);
		m2T[2] = T(m2[2]);
		ceres::AngleAxisRotatePoint(camera2, m2T, p2);
		p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

		residuals[0] = T(w)*(p1[0]-p2[0]);
		residuals[1] = T(w)*(p1[1]-p2[1]);
		residuals[2] = T(w)*(p1[2]-p2[2]);
		return true;
	}

	static ceres::CostFunction* Create(double m1x, double m1y, double m1z, double m2x, double m2y, double m2z, double w_) {
		return (new ceres::AutoDiffCostFunction<PointError, 3, 6, 6>( new PointError(m1x,m1y,m1z,m2x,m2y,m2z,w_)));
	}
};

struct PointErrorCostFunctor {
	double m1 [3];
	double m2 [3];
	double w;

	PointErrorCostFunctor(double m1x, double m1y, double m1z, double m2x, float m2y, double m2z, double w_){
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


struct PointError2CostFunctor {
	double m1 [3];
	double m2 [3];
	double w;

	PointError2CostFunctor(double m1x, double m1y, double m1z, double m2x, float m2y, double m2z, double w_){
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

		residuals[0] = sqrt(w*((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]))) ;
		//residuals[0] = w*(p1[0]-p2[0]);
		//residuals[1] = w*(p1[1]-p2[1]);
		//residuals[2] = w*(p1[2]-p2[2]);
		return true;
	}
};

struct PointError3CostFunctor {
	vector<double> m1 [3];
	vector<double> m2 [3];
	vector<double> w;

	PointError3CostFunctor(){}

	bool operator()(const double* const camera1, const double* camera2, double* residuals) const {
		residuals[0] = 0;
		int nr_matches = w.size();
		double m1T[3];
		double m2T[3];
		double p1[3];
		double p2[3];
		for(int i = 0; i < nr_matches; i++){
			m1T[0] = m1[0][i];
			m1T[1] = m1[1][i];
			m1T[2] = m1[2][i];

			ceres::AngleAxisRotatePoint(camera1, m1T, p1);
			p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

			m2T[0] = m2[0][i];
			m2T[1] = m2[1][i];
			m2T[2] = m2[2][i];
			ceres::AngleAxisRotatePoint(camera2, m2T, p2);
			p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

			residuals[0] += sqrt(w[i]*((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]))) ;
		}
		/*
		double p1[3];
		ceres::AngleAxisRotatePoint(camera1, m1, p1);
		p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

		double p2[3];
		ceres::AngleAxisRotatePoint(camera2, m2, p2);
		p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

		residuals[0] = sqrt(w*((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]))) ;
		*/
		return true;
	}
};

struct PointError2 {
	double m1 [3];
	double m2 [3];
	double w;


	PointError2(double m1x, double m1y, double m1z, double m2x, float m2y, double m2z, double w_){
		m1[0] = m1x;
		m1[1] = m1y;
		m1[2] = m1z;

		m2[0] = m2x;
		m2[1] = m2y;
		m2[2] = m2z;
		w = w_;
	}

	template <typename T> bool operator()(const T* const camera1, const T* camera2, T* residuals) const {
		T m1T[3];
		m1T[0] = T(m1[0]);
		m1T[1] = T(m1[1]);
		m1T[2] = T(m1[2]);
		T p1[3];
		ceres::AngleAxisRotatePoint(camera1, m1T, p1);
		p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

		T p2[3];
		T m2T[3];
		m2T[0] = T(m2[0]);
		m2T[1] = T(m2[1]);
		m2T[2] = T(m2[2]);
		ceres::AngleAxisRotatePoint(camera2, m2T, p2);
		p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

		T dx = p1[0]-p2[0];
		T dy = p1[1]-p2[1];
		T dz = p1[2]-p2[2];
		//residuals[1] = T(w)*(p1[1]-p2[1]);
		//residuals[2] = T(w)*(p1[2]-p2[2]);
		residuals[0] = sqrt(T(w)*(T(1)+dx*dx+dy*dy+dz*dz));

		return true;
	}

	static ceres::CostFunction* Create(double m1x, double m1y, double m1z, double m2x, double m2y, double m2z, double w_) {
		return (new ceres::AutoDiffCostFunction<PointError2, 1, 6, 6>( new PointError2(m1x,m1y,m1z,m2x,m2y,m2z,w_)));
	}
};

struct MassPointError {
//	double m1 [3];
//	double m2 [3];
//	double w;

	vector<double> m1x;
	vector<double> m1y;
	vector<double> m1z;

	vector<double> m2x;
	vector<double> m2y;
	vector<double> m2z;


	vector<double> w;

	MassPointError(){
	}

//	template <typename T> bool operator()(const T* const camera1, const T* camera2, T* residuals) const {
//		T m1T[3];
//		m1T[0] = T(m1[0]);
//		m1T[1] = T(m1[1]);
//		m1T[2] = T(m1[2]);
//		T p1[3];
//		ceres::AngleAxisRotatePoint(camera1, m1T, p1);
//		p1[0] += camera1[3]; p1[1] += camera1[4]; p1[2] += camera1[5];

//		T p2[3];
//		T m2T[3];
//		m2T[0] = T(m2[0]);
//		m2T[1] = T(m2[1]);
//		m2T[2] = T(m2[2]);
//		ceres::AngleAxisRotatePoint(camera2, m2T, p2);
//		p2[0] += camera2[3]; p2[1] += camera2[4]; p2[2] += camera2[5];

//		residuals[0] = T(w)*(p1[0]-p2[0]);
//		residuals[1] = T(w)*(p1[1]-p2[1]);
//		residuals[2] = T(w)*(p1[2]-p2[2]);
//		return true;
//	}

//	static ceres::CostFunction* Create(double m1x, double m1y, double m1z, double m2x, double m2y, double m2z, double w_) {
//		return (new ceres::AutoDiffCostFunction<MassPointError, 3, 6, 6>( new MassPointError(m1x,m1y,m1z,m2x,m2y,m2z,w_)));
//	}
};

std::vector<Eigen::Matrix4d>  align_clouds_ceres(int type, vector< Matrix3Xd > measurements, vector< vector< vector<int> > > src_matches, vector< vector< vector<int> > > dst_matches, vector< vector< vector<float> > > w_matches){
    printf("align_clouds_ceres\n");

    ceres::Problem problem;
    Solver::Options options;
    options.max_num_iterations = 1500;
    options.minimizer_progress_to_stdout = true;
	//options.num_linear_solver_threads = 11;
	//options.num_threads = 11;
    Solver::Summary summary;

    double ** poses = new double*[src_matches.size()];
    for(int f = 0; f < src_matches.size(); f++){
        poses[f] = new double[6];
        for(int i = 0; i < 6; i++){poses[f][i] = 0;}
    }

    for(int f1 = 0; f1 < src_matches.size(); f1++){
        for(int f2 = 0; f2 < src_matches.size(); f2++){
/*
			if(src_matches[f1][f2].size() > 0){
				PointError3CostFunctor * cf = new PointError3CostFunctor();
				for(int i = 0; i < src_matches[f1][f2].size(); i++){
					int src = src_matches[f1][f2][i];
					int dst = dst_matches[f1][f2][i];
					double w = w_matches[f1][f2][i];

					cf->m1[0].push_back(measurements[f1](0,src));
					cf->m1[1].push_back(measurements[f1](1,src));
					cf->m1[2].push_back(measurements[f1](2,src));
					cf->m2[0].push_back(measurements[f2](0,dst));
					cf->m2[1].push_back(measurements[f2](1,dst));
					cf->m2[2].push_back(measurements[f2](2,dst));
					cf->w.push_back(w);
				}
				CostFunction* cost_function = new NumericDiffCostFunction<PointError3CostFunctor, CENTRAL, 1, 6, 6> (cf);
				problem.AddResidualBlock(cost_function, NULL ,poses[f1],poses[f2]);
			}
*/


			for(int i = 0; i < src_matches[f1][f2].size(); i++){
                int src = src_matches[f1][f2][i];
                int dst = dst_matches[f1][f2][i];
                double w = w_matches[f1][f2][i];
				CostFunction* cost_function = 0;
				if(type == 0){cost_function = new NumericDiffCostFunction<PointErrorCostFunctor, CENTRAL, 3, 6, 6> (new PointErrorCostFunctor(measurements[f1](0,src),measurements[f1](1,src),measurements[f1](2,src),measurements[f2](0,dst),measurements[f2](1,dst),measurements[f2](2,dst),w));}
				if(type == 1){cost_function = new NumericDiffCostFunction<PointError2CostFunctor, CENTRAL, 1, 6, 6> (new PointError2CostFunctor(measurements[f1](0,src),measurements[f1](1,src),measurements[f1](2,src),measurements[f2](0,dst),measurements[f2](1,dst),measurements[f2](2,dst),w));}
				if(type == 2){cost_function = PointError::Create(measurements[f1](0,src),measurements[f1](1,src),measurements[f1](2,src),measurements[f2](0,dst),measurements[f2](1,dst),measurements[f2](2,dst),w);}
				if(cost_function != 0){problem.AddResidualBlock(cost_function, NULL ,poses[f1],poses[f2]);}
            }

        }
    }

    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

	std::vector<Eigen::Matrix4d> transforms;
    for(int f = 0; f < src_matches.size(); f++){
        Eigen::Matrix4d mat = (getMat(poses[0]).inverse()*getMat(poses[f]));
		transforms.push_back(mat);
		//cout << mat << endl;
		//printf("---------------------\n");
    }
	return transforms;
}

int main(int argc, char **argv){
    int nr_tests = 1;
	int nr_frames = 100;
	int points_per_match = 100;
	double overlap_prob = 0.1;
	int nr_points = 5000;
    double noise = 0.01;

    for(int nt = 0; nt < nr_tests; nt++){
        Matrix3Xd gt = getPoints(nr_points);
        vector< Matrix3Xd > measurements;
        for(int f = 0; f < nr_frames; f++){
            measurements.push_back(getMeasurements(gt,noise));
        }

        vector< vector< vector<int> > > src_matches;
        vector< vector< vector<int> > > dst_matches;
        vector< vector< vector<float> > > w_matches;
        src_matches.resize(nr_frames);
        dst_matches.resize(nr_frames);
        w_matches.resize(nr_frames);

        for(int f1 = 0; f1 < nr_frames; f1++){
            src_matches[f1].resize(nr_frames);
            dst_matches[f1].resize(nr_frames);
            w_matches[f1].resize(nr_frames);
        }

        for(int f1 = 0; f1 < nr_frames; f1++){
            for(int f2 = f1+1; f2 < nr_frames; f2++){
                if((rand()%1000) > 1000.0*(1.0-overlap_prob)){
                    printf("%i %i\n",f1,f2);
                    for(int p = 0; p < points_per_match; p++){
                        int ind = rand()%nr_points;
                        src_matches[f1][f2].push_back(ind);
                        dst_matches[f1][f2].push_back(ind);;
                        w_matches[f1][f2].push_back((rand()%1000)/1000.0);
                    }

                    for(int p = 0; p < points_per_match; p++){
                        int ind = rand()%nr_points;
                        src_matches[f2][f1].push_back(ind);
                        dst_matches[f2][f1].push_back(ind);;
                        w_matches[f2][f1].push_back((rand()%1000)/1000.0);					
                    }
                }
            }
        }

		std::vector<Eigen::Matrix4d> ceres0_transforms = align_clouds_ceres(0,measurements,src_matches,dst_matches,w_matches);
		std::vector<Eigen::Matrix4d> ceres1_transforms = align_clouds_ceres(1,measurements,src_matches,dst_matches,w_matches);
		std::vector<Eigen::Matrix4d> ceres2_transforms = align_clouds_ceres(2,measurements,src_matches,dst_matches,w_matches);
    }

////exit(0);




//	vector<Matrix4d> transformations;

//	for(int i = 0; i <= 100; i+=100){
//		double angle1 = double(0);double angle2 = 0;double angle3 = 0;
//		double t1 = double(i*0.01);double t2 = 0;	double t3 = 0;

//		//printf("\%\% transformation %i -> angle: %f %f %f translation: %f %f %f\n",i,angle1,angle2,angle3,t1,t2,t3);
//		Matrix3d rotmat;
//		rotmat = AngleAxisd(angle1, Vector3d::UnitZ()) * AngleAxisd(angle2, Vector3d::UnitY()) * AngleAxisd(angle3, Vector3d::UnitZ());
//		Matrix4d transformation = Matrix4d::Identity();
//		transformation.block(0,0,3,3) = rotmat;
//		transformation(0,3) = t1; transformation(1,3) = t2; transformation(2,3) = t3;

//		transformations.push_back(transformation);
//	}

//	for(int i = 0; i <= 100; i+=100){
//		double angle1 = 0.01*double(i)*M_PI;double angle2 = 0;double angle3 = 0;
//		double t1 = double(0);double t2 = 0;	double t3 = 0;

//		printf("\%\% transformation %i -> angle: %f %f %f translation: %f %f %f\n",i,angle1,angle2,angle3,t1,t2,t3);
//		Matrix3d rotmat;
//		rotmat = AngleAxisd(angle1, Vector3d::UnitZ()) * AngleAxisd(angle2, Vector3d::UnitY()) * AngleAxisd(angle3, Vector3d::UnitZ());
//		Matrix4d transformation = Matrix4d::Identity();
//		transformation.block(0,0,3,3) = rotmat;
//		transformation(0,3) = t1; transformation(1,3) = t2; transformation(2,3) = t3;

//		transformations.push_back(transformation);
//	}

//	double optimal = 0;
//	vector< vector< double > > results;
//	vector< vector< double > > times;
//	results.resize(funcs.size());
//	times.resize(funcs.size());
//	for(unsigned int j = 0; j < funcs.size(); j++){
//		results[j].resize(transformations.size());
//		times[j].resize(transformations.size());
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			results[j][k] = 0;
//			times[j][k] = 0;
//		}
//	}

//	vector< vector< vector< double > > > all_results;
//	vector< vector< vector< double > > > all_times;
//	all_results.resize(funcs.size());
//	all_times.resize(funcs.size());
//	for(unsigned int j = 0; j < funcs.size(); j++){
//		all_results[j].resize(transformations.size());
//		all_times[j].resize(transformations.size());
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			all_results[j][k].resize(size);
//			all_times[j][k].resize(size);
//			for(int i = 0; i < size; i++){
//				all_results[j][k][i] = 0;
//				all_times[j][k][i] = 0;
//			}
//		}
//	}

//	int nr_funcs = funcs.size();
//	for(int i = 0; i < size; i++){
//		//printf("\%\% %i / %i\n",i,size);
//		Matrix3Xd measurements_tmp	= overall_scale * measurements[i];
//		Matrix3Xd gt_tmp			= overall_scale * gt[i];
//		//printf("start L2:\n");
//		align(L2, measurements_tmp, gt_tmp);
//		//printf("stop  L2:\n");
//		double opt_rms = get_reconst_rms(gt_tmp, measurements_tmp, cols)/sqrt(3.0) / overall_scale;
//		//printf("opt rms: %6.6f\n",opt_rms);
//		optimal += opt_rms;

//		Matrix3Xd gt_outliers			= getPoints(nr_outliers);
//		Matrix3Xd measurements_outliers	= getMeasurementsOffseted(gt_outliers, noise*sample_area);

//		//Matrix3Xd gt_full			= gt_outliers;//conc(gt[i],gt_outliers);
//		//Matrix3Xd measurements_full	= measurements_outliers;//conc(measurements[i],measurements_outliers);
//		Matrix3Xd gt_full			= overall_scale*conc(gt[i],gt_outliers);
//		Matrix3Xd measurements_full	= conc(measurements[i],measurements_outliers);


//		for(unsigned int k = 0; k < transformations.size(); k++){
//			Matrix3Xd measurements_full_trans = transform_points(measurements_full, transformations[k]);

//			//#pragma omp parallel for num_threads(7)
//			for(int j = 0; j < nr_funcs; j++){
//				//if(debugg){if(k < 1 || i != 8){continue;}}
//				//if(i < 2){continue;}
//				//if(i < 91){continue;}
//				//if(i > 2){continue;}
//				Matrix3Xd measurements_full_tmp = overall_scale*measurements_full_trans;

//				printf("%i %i %i / %i %i %i --> ",i,k,j,size,transformations.size(),funcs.size());
//				//cout << measurements_full_tmp << endl << endl;
//				double start = getTime();
//				//if(i == 93){
//				//	funcs[j]->debugg_print = true;
//				//	align(funcs[j], measurements_full_tmp, gt_full,0.00001,true);}
//				//else{align(funcs[j], measurements_full_tmp, gt_full,0.00001,debugg);}
//				align(funcs[j], measurements_full_tmp, gt_full,0.00001,debugg);
//				double stop = getTime()-start;
//				//cout << measurements_full_tmp << endl << endl;

//				double rms = get_reconst_rms(gt_full, measurements_full_tmp, cols)/sqrt(3.0) / overall_scale;
//				results[j][k] += rms-opt_rms;
//				times[j][k] += stop;//-start;
//				all_results[j][k][i] = rms-opt_rms;
//				all_times[j][k][i] = stop;//-start;

//				printf("rms: %12.12f\n",10000.0*(rms-opt_rms));
///*
//				if(rms-opt_rms > 0.000001){
//					measurements_full_tmp = overall_scale*measurements_full_trans;
//					align(funcs[j], measurements_full_tmp, gt_full,0.00001,true);
//					exit(0);
//				}
//*/
//				//if(stop > 1){exit(0);}

//				//printf("getNoise: %f\n",funcs[j]->getNoise());
//				//printf("%f += %f - %f (%f)\n",times[j],stop,start,stop-start);
//				//printf("%s diff from opt rms: %6.6f\n",funcs[j]->getString().c_str(),rms-opt_rms);
//			}
//		}
//	}

//	for(unsigned int j = 0; j < funcs.size(); j++){
//		printf("%s_rms =[",funcs[j]->getString().c_str());
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			printf("%4.4f ",10000.0*(results[j][k]));
//		}printf("];\n");
//	}

//	for(unsigned int j = 0; j < funcs.size(); j++){
//		printf("\%\% %i -> %s\n",j,funcs[j]->getString().c_str());
//	}
///*
//	printf("\%\% all_rms(funcs,transformations,testcase)\n");
//	printf("all_rms = zeros(%i,%i,%i);\n",funcs.size(),transformations.size(),size);
//	for(unsigned int j = 0; j < funcs.size(); j++){
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			printf("all_rms(%i,%i,:) = [",j,k);//buf,funcs.size(),transformations.size(),size);
//			for(int i = 0; i < size; i++){
//				printf("%3.4f ",10000.0*(all_results[j][k][i]));
//			}printf("];\n");
//		}
//	}
//*/

//	for(unsigned int j = 0; j < funcs.size(); j++){
//		printf("%s_stab =[",funcs[j]->getString().c_str());
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			double inl = 0;
//			for(int i = 0; i < size; i++){
//				inl += all_results[j][k][i] < 0.01;
//			}

//			printf("%5.5f ",inl/double(size));
//		}printf("];\n");
//	}


//	printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
//	for(unsigned int j = 0; j < funcs.size(); j++){
//		printf("%s_time =[",funcs[j]->getString().c_str());
//		for(unsigned int k = 0; k < transformations.size(); k++){
//			printf("%3.3f ",times[j][k]);
//		}printf("];\n");
//	}


	return 0;
}
