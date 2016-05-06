#ifndef DistanceWeightFunction2PPRtest_H
#define DistanceWeightFunction2PPRtest_H

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

class DistanceWeightFunction2PPR : public DistanceWeightFunction2
{
public:
	double stdval;
	double mulval;
	double meanval;

	double maxd;
	int histogram_size;
	double blurval;
	double stdgrow;

	double noiseval;
	double startreg;

	bool update_size;
	double target_length;
	double data_per_bin;
	double meanoffset;
    double blur;

	double start_maxd;
	
	std::vector<float> prob;
	std::vector<float> histogram;
    std::vector<float> smallblur_histogram;
	std::vector<float> blur_histogram;
	std::vector<float> noise;

	bool max_under_mean;
	bool interp;
	bool threshold;
	bool uniform_bias;
	bool scale_convergence;
	double nr_inliers;

	double maxp;

	bool bidir;
	int iter;

	DistanceWeightFunction2PPR(	double maxd_	= 0.25, int histogram_size_ = 25000);
	~DistanceWeightFunction2PPR();
	virtual void computeModel(MatrixXd mat);
	virtual VectorXd getProbs(MatrixXd mat);
	virtual double getNoise();
	virtual double getConvergenceThreshold();
	virtual bool update();
	virtual void reset();
	virtual std::string getString();
};

}

#endif // DistanceWeightFunction2PPRtest_H
