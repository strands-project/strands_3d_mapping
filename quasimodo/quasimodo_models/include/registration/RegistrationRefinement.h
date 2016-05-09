#ifndef RegistrationRefinement_H
#define RegistrationRefinement_H

#include "Registration.h"
#include <time.h>
#include "nanoflann.hpp"

namespace reglib
{
	class RegistrationRefinement : public Registration
	{
		public:

		MatchType type;
		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		Eigen::Matrix<double, 3, Eigen::Dynamic> Y;
		Eigen::Matrix<double, 3, Eigen::Dynamic> N;
		std::vector<double> total_dweight;
		unsigned int ycols;

		Eigen::VectorXd DST_INORMATION;

		DistanceWeightFunction2PPR2 * func;

		int nr_arraypoints;
		double * arraypoints;
		Tree3d * trees3d;
		ArrayData3D<double> * a3d;

		RegistrationRefinement();
		~RegistrationRefinement();

		void setDst(CloudData * dst_);
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};
}

#endif
