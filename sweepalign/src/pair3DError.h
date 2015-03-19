#ifndef pair3DError_H_
#define pair3DError_H_
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"

#include "util.h"



using ceres::NumericDiffCostFunction;
using ceres::SizedCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::Solve;

class pair3DError : public SizedCostFunction<3, 6, 6, 4> {
	public:
	pair3DError(double sw, double sh, double sz,double dw, double dh, double dz, double weight);
	~pair3DError();
	bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const ;

	int id;
	double sw;
	double sh;
	double sz;
	double dw;
	double dh;
	double dz;
	double weight;
    bool optimizeCameraParams;
    double information;
};
#endif
