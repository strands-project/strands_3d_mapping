#include "Scoring.h"

namespace quasimodo
{
	Scoring::Scoring(){
		printf("WARNING:new in dummy scoring class not implemented\n");
	}
	
	Scoring::~Scoring(){
		printf("WARNING:delete in dummy scoring class not implemented\n");
	}
	
	double Scoring::getScore(std::vector<int> & found, std::vector<int> & groundtruth){
		printf("WARNING:getScore() in dummy scoring class not implemented\n");
		return -1;
	}
}
