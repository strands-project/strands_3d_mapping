#ifndef quasimodoScoring_H
#define quasimodoScoring_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 

namespace quasimodo
{
	class Scoring{
		public:
		Scoring();
		~Scoring();
		
		virtual double getScore(std::vector<int> & found, std::vector<int> & groundtruth);
	};
}

#endif
