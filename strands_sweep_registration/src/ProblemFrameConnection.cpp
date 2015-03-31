#include "strands_sweep_registration/ProblemFrameConnection.h"

ProblemFrameConnection::ProblemFrameConnection(ceres::Problem & problem, Frame * src_, Frame * dst_, double * shared_params, double * src_variable_, double * dst_variable_, float weight, bool show){
	src = src_;
	dst = dst_;

	params = shared_params;

	src_variable = src_variable_;
	dst_variable = dst_variable_;

	findPossibleMatches();
	recalculatePoints();
}

void ProblemFrameConnection::addMatchesToProblem(ceres::Problem & problem, std::vector< CostFunction * > & costfunctions){
	for(unsigned int i = 0; i < costfunctions.size() && i < 1000; i++ ){
		problem.AddResidualBlock(costfunctions.at(i), 0 , src_variable, dst_variable, params);
	}
}

void ProblemFrameConnection::addMatchesToProblem(ceres::Problem & problem, float weight){
	for(unsigned int i = 0; i < src_matches.size(); i++ ){
		int src_kp_id = src_matches.at(i);
		int dst_kp_id = dst_matches.at(i);

		cv::KeyPoint src_kp = src->keypoints.at(src_kp_id);
		cv::KeyPoint dst_kp = dst->keypoints.at(dst_kp_id);

		double sz	= src->keypoint_depth.at(src_kp_id);
		double dz	= dst->keypoint_depth.at(dst_kp_id);			

		CostFunction* err = new pair3DError(src_kp.pt.x,src_kp.pt.y,sz,dst_kp.pt.x,dst_kp.pt.y,dz,weight);
		problem.AddResidualBlock(err, NULL, src_variable, dst_variable, params);
	}
}

void ProblemFrameConnection::findPossibleMatches(float di, float pi, Eigen::Matrix4f pose){
	int nr_src = src->keypoints.size();
	int nr_dst = dst->keypoints.size();

	if(nr_src == 0 || nr_dst == 0){return;}

	std::vector<Eigen::Vector3f> & src_keypoint_location = src->keypoint_location;
	std::vector<Eigen::Vector3f> & dst_keypoint_location = dst->keypoint_location;

	float pmat00 = pose(0,0);
	float pmat01 = pose(0,1);
	float pmat02 = pose(0,2);
	float pmat03 = pose(0,3);

	float pmat10 = pose(1,0);
	float pmat11 = pose(1,1);
	float pmat12 = pose(1,2);
	float pmat13 = pose(1,3);

	float pmat20 = pose(2,0);
	float pmat21 = pose(2,1);
	float pmat22 = pose(2,2);
	float pmat23 = pose(2,3);

	float * best_src = new float[nr_src];
	float * best_src_e = new float[nr_src];
	float * best_src_f = new float[nr_src];
	int * best_src_id = new int[nr_src];
	for(int i = 0; i < nr_src; i++){
		best_src[i]		= 999999999999;
		best_src_id[i]	= -1;
	}

	float * best_dst = new float[nr_dst];
	int * best_dst_id = new int[nr_dst];
	for(int i = 0; i < nr_dst; i++){
		best_dst[i]		= 999999999999;
		best_dst_id[i]	= -1;
	}

	uint64_t xordata [4];
	const uint64_t * src_data = (uint64_t *)(src->descriptors.data);
	const uint64_t * dst_data = (uint64_t *)(dst->descriptors.data);

	for(int i = 0; i < nr_src; i++){

		Eigen::Vector3f & sp = src_keypoint_location.at(i);
		float sx	= sp(0);
		float sy	= sp(1);
		float sz	= sp(2);
		float src_x = sx*pmat00+sy*pmat01+sz*pmat02+pmat03;
		float src_y = sx*pmat10+sy*pmat11+sz*pmat12+pmat13;
		float src_z = sx*pmat20+sy*pmat21+sz*pmat22+pmat23;

		unsigned int i4 = i*4;
		const uint64_t s1 = src_data[i4+0];
		const uint64_t s2 = src_data[i4+1];
		const uint64_t s3 = src_data[i4+2];
		const uint64_t s4 = src_data[i4+3];

		for(int j = 0; j < nr_dst; j++){
			//printf("%i %i\n",i,j);
			float f_dist;
			if(src->featuretype == 0){//ORB
				unsigned int j4 = j*4;
				const uint64_t d1 = dst_data[j4+0];
				const uint64_t d2 = dst_data[j4+1];
				const uint64_t d3 = dst_data[j4+2];
				const uint64_t d4 = dst_data[j4+3];

				xordata[0] = s1 ^ d1;
				xordata[1] = s2 ^ d2;
				xordata[2] = s3 ^ d3;
				xordata[3] = s4 ^ d4;

				int cnt = popcount_lauradoux(xordata, 4);
				f_dist = float(cnt)/256.0f;
			}

			Eigen::Vector3f & dp = dst_keypoint_location.at(j);
			float dx = dp(0)-src_x;
			float dy = dp(1)-src_y;			
			float dz = dp(2)-src_z;

			float p_dist = (dx*dx+dy*dy+dz*dz);

			float d = f_dist*di + p_dist*pi;

			if(d < best_src[i]){
				best_src_id[i] = j;
				best_src[i] = d;
				best_src_e[i] = p_dist;
				best_src_f[i] = f_dist;
			}
				
			if(d < best_dst[j]){
				best_dst_id[j] = i;
				best_dst[j] = d;
			}
		}
	}

	possible_matches_fdistance.clear();
	possible_matches_edistance.clear();	
	src_possible_matches_id.clear();
	dst_possible_matches_id.clear();

	for(int i = 0; i < nr_src; i++){
		int j = best_src_id[i];
		if(best_dst_id[j] != i){continue;}//One to one

		possible_matches_fdistance.push_back(best_src_f[i]);
		possible_matches_edistance.push_back(best_src_e[i]);
		src_possible_matches_id.push_back(i);
		dst_possible_matches_id.push_back(j);
	}

	delete[] best_src;
	delete[] best_src_e;
	delete[] best_src_f;
	delete[] best_src_id;
	delete[] best_dst;
	delete[] best_dst_id;
}

void ProblemFrameConnection::recalculatePoints(){
	src_points.clear();
	dst_points.clear();
	
	for(unsigned int i = 0; i < src_possible_matches_id.size(); i++){
		int src_kp_id = src_possible_matches_id.at(i);
		int dst_kp_id = dst_possible_matches_id.at(i);

		cv::KeyPoint src_kp = src->keypoints.at(src_kp_id);
		cv::KeyPoint dst_kp = dst->keypoints.at(dst_kp_id);

		double sz	= src->keypoint_depth.at(src_kp_id);
		double dz	= dst->keypoint_depth.at(dst_kp_id);

		double sx	= (src_kp.pt.x - params[2]) * sz * params[0];
		double sy	= (src_kp.pt.y - params[3]) * sz * params[1];
	
		double dx	= (dst_kp.pt.x - params[2]) * dz * params[0];
		double dy	= (dst_kp.pt.y - params[3]) * dz * params[1];

		src_points.push_back(Eigen::Vector3f(sx,sy,sz));
		dst_points.push_back(Eigen::Vector3f(dx,dy,dz));
	}
}

ProblemFrameConnection::~ProblemFrameConnection(){};

