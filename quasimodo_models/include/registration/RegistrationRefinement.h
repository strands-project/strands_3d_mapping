#ifndef RegistrationRefinement_H
#define RegistrationRefinement_H

#include "Registration.h"
#include <time.h>

#include "RegistrationPPR.h"
#include "ICP.h"
//#include "GoICP_V1.3/src/jly_goicp.h"

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
		//nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> kdtree;
		nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * tree;

		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
		std::vector< DistanceWeightFunction2 * > feature_func;

		DistanceWeightFunction2 * func;

		RegistrationRefinement();
		~RegistrationRefinement();

		void setDst(CloudData * dst_);
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};
}

#endif
