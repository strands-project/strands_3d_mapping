#ifndef ProblemFrameConnection_H_
#define ProblemFrameConnection_H_

#include "pair3DError.h"
#include "Frame.h"

class ProblemFrameConnection {
	public:
	Frame * src;
	Frame * dst;

	double * params;
	double * src_variable;
	double * dst_variable;

	std::vector< Eigen::Vector3f > full_src_points;
	std::vector< Eigen::Vector3f > full_dst_points;


	std::vector< Eigen::Vector3f > src_points;
	std::vector< Eigen::Vector3f > dst_points;

	std::vector< float > possible_matches_fdistance;
	std::vector< float > possible_matches_edistance;
	std::vector<int> src_possible_matches_id;
	std::vector<int> dst_possible_matches_id;

	std::vector<int> src_matches;
	std::vector<int> dst_matches;

	ProblemFrameConnection(ceres::Problem & problem, Frame * src_, Frame * dst_, double * shared_params, double * src_variable_, double * dst_variable_, float weight = 1, bool show = false);
	void addMatchesToProblem(ceres::Problem & problem, std::vector< CostFunction * > & costfunctions);
	void addMatchesToProblem(ceres::Problem & problem, float weight = 1);
	void findPossibleMatches(float di = 1, float pi = 0, Eigen::Matrix4f pose = Eigen::Matrix4f::Identity());
	void recalculatePoints();
	~ProblemFrameConnection();
};

#endif
