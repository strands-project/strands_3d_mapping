#ifndef RegistrationRandom_H
#define RegistrationRandom_H

#include "Registration.h"
#include <time.h>

#include "RegistrationPPR.h"

#include "RegistrationRefinement.h"

//#include "DistanceWeightFunction2.h"
//#include "ICP.h"
namespace reglib
{
	class RegistrationRandom : public Registration
	{
		public:

		MatchType type;
		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
		std::vector< DistanceWeightFunction2 * > feature_func;

		virtual void setSrc(CloudData * src_);
		virtual void setDst(CloudData * dst_);

		Registration * refinement;

		DistanceWeightFunction2 * func;

		RegistrationRandom();
		~RegistrationRandom();
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};
}

#endif
