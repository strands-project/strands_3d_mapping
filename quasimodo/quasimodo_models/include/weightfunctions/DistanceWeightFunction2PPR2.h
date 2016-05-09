#ifndef DistanceWeightFunction2PPR2test_H
#define DistanceWeightFunction2PPR2test_H

#include <cmath>
#include <sys/time.h>
#include "DistanceWeightFunction2.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

using namespace Eigen;
namespace reglib{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class DistanceWeightFunction2PPR2 : public DistanceWeightFunction2
{
public:
    bool fixed_histogram_size;

	double stdval;
	double stdval2;
	double mulval;
	double meanval;
	double meanval2;

    double maxnoise;

    bool zeromean;

	double costpen;

	double maxp;

	bool first;

	bool update_size;
	double target_length;
	double data_per_bin;
	double meanoffset;
	double blur;

	double startmaxd;
	double maxd;
	int histogram_size;
	int starthistogram_size;
	double blurval;
	double stdgrow;

	double noiseval;
	double startreg;
	
	std::vector<float> prob;
	std::vector<float> histogram;
	std::vector<float> blur_histogram;
	std::vector<float> noise;

	int nr_refineiters;
	bool refine_mean;
	bool refine_mul;
	bool refine_std;

	bool threshold;
	bool uniform_bias;
	bool scale_convergence;
	double nr_inliers;

	bool rescaling;

	bool interp;

	bool max_under_mean;

	bool bidir;
	int iter;

	DistanceWeightFunction2PPR2(	double maxd_	= 0.25, int histogram_size_ = 25000);
	~DistanceWeightFunction2PPR2();
	virtual void computeModel(MatrixXd mat);
	virtual VectorXd getProbs(MatrixXd mat);
	virtual double getProb(double d);
	virtual double getNoise();
	virtual double getConvergenceThreshold();
	virtual bool update();
	virtual void reset();
	virtual std::string getString();
};

}

#endif // DistanceWeightFunction2PPR2test_H
