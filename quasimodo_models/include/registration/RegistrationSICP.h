#ifndef RegistrationSICP_H
#define RegistrationSICP_H

#include "Registration.h"

//#include "ICP.h"
namespace reglib
{
	class RegistrationSICP : public Registration
	{
		public:

		RegistrationSICP();
		~RegistrationSICP();
		
		FusionResults getTransform(Eigen::MatrixXd guess);
	};

}

#endif
