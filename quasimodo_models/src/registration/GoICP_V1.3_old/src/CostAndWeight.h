#ifndef CostAndWeight_def
#define CostAndWeight_def

class CostAndWeight {
	public:	
	CostAndWeight();
	~CostAndWeight();
	
	double threshold;
	
	double getCost(double dist);
	double getCostSquaredDistance(double squared_dist);
	double getWeightSquaredDistance(double squared_dist);
};

#endif
