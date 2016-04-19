#include "ceres/ceres.h"
#include "ceres/rotation.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct ProjectionResidual {
  ProjectionResidual(double* p1, double *p2, double weight) : weight_(weight)
  {
      memcpy(&p1_, p1, 3*sizeof(double));
      memcpy(&p2_, p2, 3*sizeof(double));
  }

  template <typename T> bool operator()(const T* const camera_rotation1,
                                        const T* const camera_translation1,
                                        const T* const camera_rotation2,
                                        const T* const camera_translation2,
                                        T* residuals) const {
      T p_c1[3];
      T point_c1[3] = {T(p1_[0]), T(p1_[1]), T(p1_[2])};
      ceres::QuaternionRotatePoint(camera_rotation1, point_c1, p_c1);
      p_c1[0] += camera_translation1[0];
      p_c1[1] += camera_translation1[1];
      p_c1[2] += camera_translation1[2];

      T p_c2[3];
      T point_c2[3] = {T(p2_[0]), T(p2_[1]), T(p2_[2])};
      ceres::QuaternionRotatePoint(camera_rotation2, point_c2, p_c2);
      p_c2[0] += camera_translation2[0];
      p_c2[1] += camera_translation2[1];
      p_c2[2] += camera_translation2[2];

      residuals[0] = (p_c1[0] - p_c2[0]) / T(weight_);
      residuals[1] = (p_c1[1] - p_c2[1]) / T(weight_);
      residuals[2] = (p_c1[2] - p_c2[2]) / T(weight_);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double* p1, double* p2, double weight) {
    return (new ceres::AutoDiffCostFunction<
            ProjectionResidual, 3, 4, 3, 4, 3>(
                new ProjectionResidual(p1,p2, weight)));
  }

 private:
    double p1_[3];
    double p2_[3];
    double weight_; // distance squared
};
