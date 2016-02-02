#ifndef RegistrationGOICP_H
#define RegistrationGOICP_H

#include "Registration.h"
#include <time.h>

#include "RegistrationPPR.h"

//#include "DistanceWeightFunction2.h"
//#include "ICP.h"
namespace reglib
{
	class RegistrationGOICP : public Registration
	{
		public:

		MatchType type;
		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
		std::vector< DistanceWeightFunction2 * > feature_func;

		DistanceWeightFunction2 * func;

		RegistrationGOICP();
		~RegistrationGOICP();
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};
}

#endif
