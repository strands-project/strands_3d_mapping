#ifndef RegistrationRandom_H
#define RegistrationRandom_H

#include "Registration.h"
#include <time.h>

#include "RegistrationRefinement.h"

namespace reglib
{
	class RegistrationRandom : public Registration
	{
		public:

		Registration * refinement;

		virtual void setSrc(CloudData * src_);
		virtual void setDst(CloudData * dst_);


		RegistrationRandom();
		~RegistrationRandom();
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};
}

#endif
