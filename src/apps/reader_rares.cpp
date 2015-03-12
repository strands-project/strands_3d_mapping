#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string.h>

#include "Frame.h"
#include "Camera.h"
#include "Sweep.h"

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ceres/rotation.h"
#include "ceres/iteration_callback.h"

#include "simple_xml_parser.h"//../../scitos_3d_mapping/metaroom_xml_parser/include/
#include "simple_summary_parser.h"
//#include "load_utilities.h"

#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transformation_from_correspondences.h>

#include "tf_conversions/tf_eigen.h"

#include <semantic_map/room_xml_parser.h>
#include <pcl_ros/transforms.h>

#include "RobotContainer.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;

typedef typename SimpleSummaryParser::EntityStruct Entities;

using ceres::NumericDiffCostFunction;
using ceres::SizedCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::Solve;

static inline int popcount_lauradoux(uint64_t *buf, uint32_t size) {
 const uint64_t* data = (uint64_t*) buf;

  const uint64_t m1  = UINT64_C(0x5555555555555555);
  const uint64_t m2  = UINT64_C(0x3333333333333333);
  const uint64_t m4  = UINT64_C(0x0F0F0F0F0F0F0F0F);
  const uint64_t m8  = UINT64_C(0x00FF00FF00FF00FF);
  const uint64_t m16 = UINT64_C(0x0000FFFF0000FFFF);
  const uint64_t h01 = UINT64_C(0x0101010101010101);

  uint32_t bitCount = 0;
  uint32_t i, j;
  uint64_t count1, count2, half1, half2, acc;
  uint64_t x;
  uint32_t limit30 = size - size % 30;

  // 64-bit tree merging (merging3)
  for (i = 0; i < limit30; i += 30, data += 30) {
    acc = 0;
    for (j = 0; j < 30; j += 3) {
      count1  =  data[j];
      count2  =  data[j+1];
      half1   =  data[j+2];
      half2   =  data[j+2];
      half1  &=  m1;
      half2   = (half2  >> 1) & m1;
      count1 -= (count1 >> 1) & m1;
      count2 -= (count2 >> 1) & m1;
      count1 +=  half1;
      count2 +=  half2;
      count1  = (count1 & m2) + ((count1 >> 2) & m2);
      count1 += (count2 & m2) + ((count2 >> 2) & m2);
      acc    += (count1 & m4) + ((count1 >> 4) & m4);
    }
    acc = (acc & m8) + ((acc >>  8)  & m8);
    acc = (acc       +  (acc >> 16)) & m16;
    acc =  acc       +  (acc >> 32);
    bitCount += (uint32_t)acc;
  }

  for (i = 0; i < size - limit30; i++) {
    x = data[i];
    x =  x       - ((x >> 1)  & m1);
    x = (x & m2) + ((x >> 2)  & m2);
    x = (x       +  (x >> 4)) & m4;
    bitCount += (uint32_t)((x * h01) >> 56);
  }
  return bitCount;
}

template <typename T> Eigen::Matrix<T,4,4> getMat(const T* const camera, int mode = 0){
	Eigen::Matrix<T,4,4> ret = Eigen::Matrix<T,4,4>::Identity();
	if(mode == 0){//yaw pitch roll tx ty tz
		Eigen::AngleAxis<T> yawAngle(camera[0], Eigen::Matrix<T,3,1>::UnitY());
		Eigen::AngleAxis<T> pitchAngle(camera[1], Eigen::Matrix<T,3,1>::UnitX());
		Eigen::AngleAxis<T> rollAngle(camera[2], Eigen::Matrix<T,3,1>::UnitZ());
		Eigen::Quaternion<T> q = rollAngle * yawAngle * pitchAngle;
		Eigen::Matrix<T,3,3> rotationMatrix = q.matrix();
		ret.block(0,0,3,3) = rotationMatrix;
		ret(0,3) = camera[3];
		ret(1,3) = camera[4];
		ret(2,3) = camera[5];
	}
	return ret;
}

template <typename T> void transformPoint(const T* const camera, T * point, int mode = 0){
	Eigen::Matrix<T,4,4> mat = getMat(camera, mode);
	T tx = mat(0,0)*point[0] + mat(0,1)*point[1] + mat(0,2)*point[2] + mat(0,3);
	T ty = mat(1,0)*point[0] + mat(1,1)*point[1] + mat(1,2)*point[2] + mat(1,3);
	T tz = mat(2,0)*point[0] + mat(2,1)*point[1] + mat(2,2)*point[2] + mat(2,3);
	point[0] = tx;
	point[1] = ty;
	point[2] = tz;
}



int sumid = 0;
void getMat(const double* const camera, double * mat){
	
	Eigen::AngleAxis<double> yawAngle(camera[0], Eigen::Matrix<double,3,1>::UnitY());
	Eigen::AngleAxis<double> pitchAngle(camera[1], Eigen::Matrix<double,3,1>::UnitX());
	Eigen::AngleAxis<double> rollAngle(camera[2], Eigen::Matrix<double,3,1>::UnitZ());
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	Eigen::Matrix<double,3,3> rotationMatrix = q.matrix();
	
	mat[0 ] = rotationMatrix(0,0);
	mat[1 ] = rotationMatrix(0,1);
	mat[2 ] = rotationMatrix(0,2);
	mat[3 ] = camera[3];

	mat[4 ] = rotationMatrix(1,0);
	mat[5 ] = rotationMatrix(1,1);
	mat[6 ] = rotationMatrix(1,2);
	mat[7 ] = camera[4];

	mat[8 ] = rotationMatrix(2,0);
	mat[9 ] = rotationMatrix(2,1);
	mat[10] = rotationMatrix(2,2);
	mat[11] = camera[5];
}

bool optimizeCameraParams = false;
double information = 1.0/1.5;//0.01;

class pair3DError : public SizedCostFunction<3, 6, 6, 4> {
	public:
	pair3DError(double sw, double sh, double sz,double dw, double dh, double dz, double weight) : sw(sw), sh(sh), sz(sz), dw(dw), dh(dh), dz(dz), weight(weight) {id = sumid++;}
	virtual ~pair3DError() {}
	virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
		const double h = 3e-7;
		const double* const scamera = parameters[0];
		const double* const dcamera = parameters[1];
		const double* const params  = parameters[2];

		double sp[3];
		double dp[3];

		double sp2[3];
		double dp2[3];

		double sc[6];
		double dc[6];
		double p [5];

		double smat[12];
		double dmat[12];


		double invfx = 1.0/540.0;
		double invfy = 1.0/540.0;
		double cx = 319.5;
		double cy = 239.5;


		if(optimizeCameraParams){
			invfx = params[0];
			invfy = params[1];
			cx = params[2];
			cy = params[3];
		}

		sp[2]	= sz;
		sp[0]	= (sw-cx) * sp[2] * invfx;
		sp[1]	= (sh-cy) * sp[2] * invfy;

		dp[2]	= dz;
		dp[0]	= (dw-cx) * dp[2] * invfx;
		dp[1]	= (dh-cy) * dp[2] * invfy;

		for(int i = 0; i < 6; i++){
			sc[i] = scamera[i];
			dc[i] = dcamera[i];
		}
		for(int i = 0; i < 4; i++){p[i] = params[i];}

		getMat(sc, smat);
		sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
		sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
		sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

		double sp20 = sp2[0];
		double sp21 = sp2[1];
		double sp22 = sp2[2];

		getMat(dc, dmat);
		dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
		dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
		dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

		residuals[0] = (sp2[0] - dp2[0]);
		residuals[1] = (sp2[1] - dp2[1]);
		residuals[2] = (sp2[2] - dp2[2]);

		double d2 = residuals[0]*residuals[0]+residuals[1]*residuals[1]+residuals[2]*residuals[2];
		double w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);

		residuals[0] *= w;
		residuals[1] *= w;
		residuals[2] *= w;

		if (jacobians != NULL && jacobians[0] != NULL) {
			//Src camera
			for(int i = 0; i < 6; i++){
				sc[i]+=h;
				getMat(sc, smat);
				sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
				sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
				sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

				double r0 = (sp2[0] - dp2[0]);
				double r1 = (sp2[1] - dp2[1]);
				double r2 = (sp2[2] - dp2[2]);
				d2 = r0*r0+r1*r1+r2*r2;
				w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
				r0 *= w;
				r1 *= w;
				r2 *= w;

				sc[i]-=2*h;
				getMat(sc, smat);
				sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
				sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
				sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

				double r3 = (sp2[0] - dp2[0]);
				double r4 = (sp2[1] - dp2[1]);
				double r5 = (sp2[2] - dp2[2]);
				d2 = r3*r3+r4*r4+r5*r5;
				w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
				r3 *= w;
				r4 *= w;
				r5 *= w;

				jacobians[0][i+0] = (r0-r3)/(2*h);
				jacobians[0][i+6] = (r1-r4)/(2*h);
				jacobians[0][i+12] = (r2-r5)/(2*h);

				sc[i]+=h;
			}

			sp2[0] = sp20;
			sp2[1] = sp21;
			sp2[2] = sp22;
			
			//dst camera
			for(int i = 0; i < 6; i++){
				dc[i]+=h;
				getMat(dc, dmat);
				dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
				dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
				dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

				double r0 = (sp2[0] - dp2[0]);
				double r1 = (sp2[1] - dp2[1]);
				double r2 = (sp2[2] - dp2[2]);
				d2 = r0*r0+r1*r1+r2*r2;
				w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
				r0 *= w;
				r1 *= w;
				r2 *= w;

				dc[i]-=2*h;
				getMat(dc, dmat);
				dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
				dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
				dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

				double r3 = (sp2[0] - dp2[0]);
				double r4 = (sp2[1] - dp2[1]);
				double r5 = (sp2[2] - dp2[2]);
				d2 = r3*r3+r4*r4+r5*r5;
				w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
				r3 *= w;
				r4 *= w;
				r5 *= w;

				jacobians[1][i+0] = (r0-r3)/(2*h);
				jacobians[1][i+6] = (r1-r4)/(2*h);
				jacobians[1][i+12] = (r2-r5)/(2*h);

				dc[i]+=h;	
			}

			if(!optimizeCameraParams){
				for(int i = 0; i < 4; i++){
					jacobians[2][i+0] = 0;
					jacobians[2][i+4] = 0;
					jacobians[2][i+8] = 0;
				}
			}else{
				getMat(sc, smat);
				getMat(dc, dmat);

				for(int i = 0; i < 4; i++){
					p[i]+=h;

					invfx = p[0];
					invfy = p[1];
					cx = p[2];
					cy = p[3];

					sp[0]	= (sw-cx) * sp[2] * invfx;
					sp[1]	= (sh-cy) * sp[2] * invfy;
					dp[0]	= (dw-cx) * dp[2] * invfx;
					dp[1]	= (dh-cy) * dp[2] * invfy;
					sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
					sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
					sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];
					dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
					dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
					dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

					double r0 = (sp2[0] - dp2[0]);
					double r1 = (sp2[1] - dp2[1]);
					double r2 = (sp2[2] - dp2[2]);
					d2 = r0*r0+r1*r1+r2*r2;
					w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
					r0 *= w;
					r1 *= w;
					r2 *= w;

					p[i]-=2*h;

					invfx = p[0];
					invfy = p[1];
					cx = p[2];
					cy = p[3];

					sp[0]	= (sw-cx) * sp[2] * invfx;
					sp[1]	= (sh-cy) * sp[2] * invfy;
					dp[0]	= (dw-cx) * dp[2] * invfx;
					dp[1]	= (dh-cy) * dp[2] * invfy;
					sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
					sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
					sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];
					dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
					dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
					dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

					double r3 = (sp2[0] - dp2[0]);
					double r4 = (sp2[1] - dp2[1]);
					double r5 = (sp2[2] - dp2[2]);
					d2 = r3*r3+r4*r4+r5*r5;
					w = weight;// * exp(0.25*d2*information*information);//*pow(information,7);
					r3 *= w;
					r4 *= w;
					r5 *= w;

					jacobians[2][i+0] = (r0-r3)/(2*h);
					jacobians[2][i+4] = (r1-r4)/(2*h);
					jacobians[2][i+8] = (r2-r5)/(2*h);

					p[i]+=h;
				}
			}
		}
		return true;
	}

	int id;
	double sw;
	double sh;
	double sz;
	double dw;
	double dh;
	double dz;
	double weight;
};

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

	ProblemFrameConnection(ceres::Problem & problem, Frame * src_, Frame * dst_, double * shared_params, double * src_variable_, double * dst_variable_, float weight = 1, bool show = false){
		src = src_;
		dst = dst_;

		params = shared_params;

		src_variable = src_variable_;
		dst_variable = dst_variable_;

		findPossibleMatches();
		recalculatePoints();
	}

	void addMatchesToProblem(ceres::Problem & problem, vector< CostFunction * > & costfunctions){
		for(unsigned int i = 0; i < costfunctions.size() && i < 1000; i++ ){
			problem.AddResidualBlock(costfunctions.at(i), 0 , src_variable, dst_variable, params);
		}
	}

	void addMatchesToProblem(ceres::Problem & problem, float weight = 1){
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

	void findPossibleMatches(float di = 1, float pi = 0, Eigen::Matrix4f pose = Eigen::Matrix4f::Identity()){
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

	void recalculatePoints(){
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

	~ProblemFrameConnection(){};
};

vector< CostFunction * > getMatchesICP(vector< ProblemFrameConnection * > & pc_vec, Eigen::Matrix4f & retpose, int iterations = 20){

	float std_descriptor = 100000000;
	float std_distance = 0.005;

	float info_descriptor	= 1.0f/(std_descriptor*std_descriptor);
	float info_distance		= 1.0f/(std_distance*std_distance);

	for(int iter = 0; iter < iterations; iter++){
		
		for(unsigned int i = 0; i < pc_vec.size(); i++){
			pc_vec.at(i)->findPossibleMatches(std_descriptor,info_distance,retpose);
		}

		float score = 0;
		pcl::TransformationFromCorrespondences tfc;
		for(unsigned int i = 0; i < pc_vec.size(); i++){

			pc_vec.at(i)->recalculatePoints();
			std::vector< float > & possible_matches_fdistance = pc_vec.at(i)->possible_matches_fdistance;
			std::vector< float > & possible_matches_edistance = pc_vec.at(i)->possible_matches_edistance;
			std::vector< Eigen::Vector3f > & src_points = pc_vec.at(i)->src_points;
			std::vector< Eigen::Vector3f > & dst_points = pc_vec.at(i)->dst_points;

			unsigned int nr_kp = src_points.size();
			for(unsigned int j = 0; j < nr_kp; j++){
				float fdistance		= possible_matches_fdistance.at(j);
				float edistance		= possible_matches_edistance.at(j);
				float totdistance	= fdistance*info_descriptor+edistance*info_distance;
				if(totdistance < 40){
					Eigen::Vector3f src_data = src_points.at(j);
					Eigen::Vector3f dst_data = dst_points.at(j);
					float weight = exp(-0.5*totdistance);
					score+=weight;
					tfc.add(src_data,dst_data,weight);
				}
			}
		}
		retpose = tfc.getTransformation().matrix();
		if(iter % 1 == 0){
			printf("ICP iteration: %3i score: %10.10f ",iter,score);
			printf("info_descriptor: %10.10f info_distance: %10.10f\n",info_descriptor,info_distance);
		}
	}

	vector< CostFunction * > errors;
	float score = 0;
	for(unsigned int i = 0; i < pc_vec.size(); i++){
		ProblemFrameConnection * pc = pc_vec.at(i);
		pc->recalculatePoints();
		std::vector< float > & possible_matches_fdistance	= pc->possible_matches_fdistance;
		std::vector< float > & possible_matches_edistance	= pc->possible_matches_edistance;

		unsigned int nr_kp = possible_matches_fdistance.size();
		for(unsigned int j = 0; j < nr_kp; j++){
			float fdistance		= possible_matches_fdistance.at(j);
			float edistance		= possible_matches_edistance.at(j);
			float totdistance	= fdistance*info_descriptor+edistance*info_distance;
			if(totdistance < 40){
				float weight = exp(-0.5*totdistance);
				score+=weight;

				int src_kp_id = pc->src_possible_matches_id.at(j);
				int dst_kp_id = pc->dst_possible_matches_id.at(j);
				pc->src_matches.push_back(src_kp_id);
				pc->dst_matches.push_back(dst_kp_id);
				cv::KeyPoint src_kp = pc->src->keypoints.at(src_kp_id);
				cv::KeyPoint dst_kp = pc->dst->keypoints.at(dst_kp_id);
				double sz	= pc->src->keypoint_depth.at(src_kp_id);
				double dz	= pc->dst->keypoint_depth.at(dst_kp_id);

				CostFunction* err = new pair3DError(src_kp.pt.x,src_kp.pt.y,sz,dst_kp.pt.x,dst_kp.pt.y,dz,weight);
				errors.push_back(err);
			}
		}
	}

	printf("ICP final score: %10.10f\n",score);

	return errors;
}

vector< CostFunction * > getMatchesRansac(vector< ProblemFrameConnection * > & pc_vec, float weight = 1, float threshold = 0.005, int ransac_iter = 200000, int nr_points = 3){
	vector<int> owner;
	vector<int> match_id;
	vector< Eigen::Vector3f > src_points;
	vector< Eigen::Vector3f > dst_points;

	for(unsigned int i = 0; i < pc_vec.size(); i++){
		for(unsigned int j = 0; j < pc_vec.at(i)->src_points.size(); j++){
			owner.push_back(i);
			match_id.push_back(j);
			src_points.push_back(pc_vec.at(i)->src_points.at(j));
			dst_points.push_back(pc_vec.at(i)->dst_points.at(j));
		}
	}

	int nr_kp = src_points.size();
	vector< CostFunction * > errors;
	if(nr_kp == 0){return errors;}
	int nr_consistent = 0;


	Eigen::Matrix4f retpose;
	vector<int> bestmatching;
	for(int it = 0; it < ransac_iter; it++){
		//Sample points
		vector< int > indexes;
		vector< Eigen::Vector3f > src_samples;
		vector< Eigen::Vector3f > dst_samples;

		for(int j = 0; j < nr_points; j++){
			int ind;
			bool failed = true;
			while(failed){
				ind = rand()%nr_kp;
				failed = false;
				for(int k = 0; k < j; k++){
					if(ind == indexes.at(k)){failed = true;}
				}
			}
			indexes.push_back(ind);
			src_samples.push_back(src_points.at(ind));
			dst_samples.push_back(dst_points.at(ind));
		}
			
		//Check consistency
		bool consistent = true;

		for(int j = 0; j < nr_points; j++){
			for(int k = j+1; k < nr_points; k++){
				float src_distance = (src_samples.at(j)-src_samples.at(k)).norm();
				float dst_distance = (dst_samples.at(j)-dst_samples.at(k)).norm();
				if(fabs(src_distance-dst_distance) > 0.02){consistent = false; break;break;}  
			}
		}

		//Check inliers
		nr_consistent += consistent;
		if(consistent){
			pcl::TransformationFromCorrespondences tfc;
			for(int i = 0; i < nr_points; i++){tfc.add(dst_samples.at(i),src_samples.at(i),1);}
			Eigen::Affine3f ret = tfc.getTransformation();
			
			Eigen::Matrix4f pose = ret.matrix().inverse();
			
			std::vector<int> matching;
			for(int j = 0; j < nr_kp; j++){
				Eigen::Vector3f src_data = src_points.at(j);
				Eigen::Vector3f dst_data = dst_points.at(j);
				
				Eigen::Vector4f sd = pose*Eigen::Vector4f(src_data(0),src_data(1),src_data(2),1);
				Eigen::Vector4f dd = Eigen::Vector4f(dst_data(0),dst_data(1),dst_data(2),1);
				float t = threshold;//0.1*0.005*(dst_data(2)*dst_data(2)+src_data(2)*src_data(2));
				if((sd-dd).norm() < t){matching.push_back(j);}
			}

			//save if best
			if(matching.size() > bestmatching.size()){
				bestmatching = matching;
				retpose = pose;
				//printf("%i -> %i\n",it,matching.size());
			}
			if(nr_consistent == 30000){printf("break at %i\n",it);break;}
		}
	}
	
	printf("nr matches best: %i / %i consistent: %i / %i\n",int(bestmatching.size()),nr_kp,nr_consistent,ransac_iter);

	//getMatchesICP(pc_vec,retpose);

	for(int iter = 0; iter < 200; iter++){
		bestmatching.clear();
		pcl::TransformationFromCorrespondences tfc;	
		int matching = 0;
		for(int j = 0; j < nr_kp; j++){
			Eigen::Vector3f src_data = src_points.at(j);
			Eigen::Vector3f dst_data = dst_points.at(j);
				
			Eigen::Vector4f sd = retpose*Eigen::Vector4f(src_data(0),src_data(1),src_data(2),1);
			Eigen::Vector4f dd = Eigen::Vector4f(dst_data(0),dst_data(1),dst_data(2),1);

			if((sd-dd).norm() < threshold){
				bestmatching.push_back(j);
				tfc.add(dst_data.head<3>(),src_data.head<3>(),1);
				matching++;				
			}
		}
		if(iter % 50 == 0){printf("iteration: %i matches: %i\n",iter,matching);}

		Eigen::Affine3f ret = tfc.getTransformation();
		retpose = ret.matrix().inverse();
	}
	printf("nr matches best: %i / %i \n",int(bestmatching.size()),nr_kp);
//exit(0);
	if(bestmatching.size() < 15){return errors;}
	for(unsigned int i = 0; i < pc_vec.size(); i++){
		pc_vec.at(i)->src_matches.clear();
		pc_vec.at(i)->dst_matches.clear();
	}

	for(unsigned int i = 0; i < bestmatching.size() && i < 1000; i++){
		ProblemFrameConnection * pc = pc_vec.at(owner.at(bestmatching.at(i)));
		int id = match_id.at(bestmatching.at(i));
		//printf("id: %i %i %i\n ",id,pc->src_matches.size(),pc->dst_matches.size());
		int src_kp_id = pc->src_possible_matches_id.at(id);
		int dst_kp_id = pc->dst_possible_matches_id.at(id);
		pc->src_matches.push_back(src_kp_id);
		pc->dst_matches.push_back(dst_kp_id);
		cv::KeyPoint src_kp = pc->src->keypoints.at(src_kp_id);
		cv::KeyPoint dst_kp = pc->dst->keypoints.at(dst_kp_id);
		double sz	= pc->src->keypoint_depth.at(src_kp_id);
		double dz	= pc->dst->keypoint_depth.at(dst_kp_id);
		CostFunction* err = new pair3DError(src_kp.pt.x,src_kp.pt.y,sz,dst_kp.pt.x,dst_kp.pt.y,dz,weight);
		errors.push_back(err);
	}

	return errors;
}

void saveSweep(Sweep * sweep){//std::vector<tf::StampedTransform> transforms)
	std::string sweep_xml = sweep->xmlpath;
	std::vector<Eigen::Matrix4f> poses = sweep->getPoseVector();

	std::cout<<"Sweep xml "<<sweep_xml<<std::endl;
    std::cout<<"No poses "<<poses.size()<<std::endl;

 	auto room = SemanticRoomXMLParser<PointType>::loadRoomFromXML(sweep_xml, true);
	room.clearIntermediateCloudRegisteredTransforms();
	room.clearIntermediateCloudCameraParametersCorrected();
    auto original_transforms = room.getIntermediateCloudTransforms();
	auto intClouds = room.getIntermediateClouds();
	auto original_params = room.getIntermediateCloudCameraParameters();

	static tf::StampedTransform firstTransform = original_transforms[0];

	Camera * camera = sweep->frames[0][0]->camera;
	for (unsigned int i=0; i<poses.size(); i++){
		Eigen::Matrix4f pose = poses[i];
		// add reg transform and corrected camera parameters to sweep
		auto transform = original_transforms[i];
		tf::Transform tfTr;
		const Eigen::Affine3d eigenTr(pose.cast<double>());
		tf::transformEigenToTF(eigenTr, tfTr);
		tf::Transform combinedTransform = firstTransform * tfTr;

		sensor_msgs::CameraInfo camInfo;
		camInfo.P = {camera->fx, 0.0, camera->cx, 0.0, 0.0, camera->fy, camera->cy, 0.0,0.0, 0.0, 1.0,0.0}; 	
		camInfo.D = {0,0,0,0,0}; 
		image_geometry::PinholeCameraModel aCameraModel;
		aCameraModel.fromCameraInfo(camInfo);
		room.addIntermediateCloudCameraParametersCorrected(aCameraModel);	

		transform.setOrigin(tfTr.getOrigin());
		transform.setBasis(tfTr.getBasis());
		room.addIntermediateRoomCloudRegisteredTransform(transform);
	}

    SemanticRoomXMLParser<PointType> parser("/media/johane/SSDstorage/output/");
	parser.saveRoomAsXML(room);
}

std::vector<std::string> loadAllSweeps(std::string folder_path){
	std::vector<std::string> toRet;
    SimpleSummaryParser summaryParser(folder_path+"/index.xml");
	summaryParser.createSummaryXML(folder_path+"/");
    std::vector<SimpleSummaryParser::EntityStruct> allRooms = summaryParser.getRooms();
	for (size_t i=0; i<allRooms.size(); i++){
		toRet.push_back(allRooms[i].roomXmlFile);
		//std::cout<<"Found sweep "<<allRooms[i].roomXmlFile<<std::endl;
	}
	return toRet;
}	

pcl::PointCloud<PointType>::Ptr rebuildRegisteredCloud(std::string room_xml){
    cout<<"Rebuilding registered cloud for "<<room_xml<<endl;
    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(room_xml,true);
    parser.setRootFolderFromRoomXml(room_xml);

    std::vector<tf::StampedTransform> cloudTransformsReg = aRoom.getIntermediateCloudTransformsRegistered();
    std::vector<pcl::PointCloud<PointType>::Ptr> clouds= aRoom.getIntermediateClouds();

    pcl::PointCloud<PointType>::Ptr mergedCloudRegistered(new pcl::PointCloud<PointType>);
    if (cloudTransformsReg.size() == clouds.size()){
        for (size_t j=0; j<clouds.size(); j++){
            pcl::PointCloud<PointType> transformed_cloud;
            pcl_ros::transformPointCloud(*clouds[j], transformed_cloud,cloudTransformsReg[j]);
            *mergedCloudRegistered+=transformed_cloud;
        }
        //aRoom.setCompleteRoomCloud(mergedCloudRegistered);
        //parser.saveRoomAsXML(aRoom);
    } else {
        cout<<"Cannot build registered cloud, the registered intermediate cloud transforms have not been set "<<endl;
    }
	return mergedCloudRegistered;
}

template <class PointType>
std::vector<std::string> getSweepXmlsForTopologicalWaypoint(std::string folderPath, std::string waypoint, bool verbose= false){
    SimpleSummaryParser summary_parser;
    summary_parser.createSummaryXML(folderPath);
    auto sweep_xmls = summary_parser.getRooms();

    std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
    for (size_t i=0; i<sweep_xmls.size(); i++){
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
        waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
    }
    return waypointToSweepsMap[waypoint];
}

int main(int argc, char **argv){

	unsigned int gx = 17;
	unsigned int todox = 17;
	unsigned int gy = 3;
	unsigned int todoy = 3;

    unsigned int start_sweep = 0;
    unsigned int stop_sweep = 1;

    unsigned int sweeps_for_training = 1000;
	float loop_weight = 1000;

	RobotContainer * rc = new RobotContainer(gx,todox,gy,todoy);

	double * shared_params = new double[5];
	shared_params[0] = 1.0/540.0;		//invfx
	shared_params[1] = 1.0/540.0;		//invfy
	shared_params[2] = (640.0-1.0)/2;	//cx
	shared_params[3] = (480.0-1.0)/2;	//cy
	shared_params[4] = 0.1;

	unsigned int inds[17][3];
	for(unsigned int y = 0; y < gy; y++){//Camera moving forward
		for (unsigned int x = 0; x < gx ; x++){inds[x][y] = 0;}
		if(y % 2 == 0){
			for (unsigned int x = 0; x < gx ; x++){inds[x][y] = y*gx+x;}
		}else{
			for (unsigned int x = 0; x < gx ; x++){inds[x][y] = y*gx+gx-x-1;}
		}
	}
		
	string folderPath;
	if (argc < 2){
		folderPath = "/media/johane/SSDstorage/rareshdata/";
	}else{
		folderPath = argv[1];
	}

    string summaryXMLPath = folderPath + "/index.xml";
    SimpleSummaryParser summary_parser(summaryXMLPath);
    summary_parser.createSummaryXML(folderPath);

    SimpleXMLParser<PointType> simple_parser;

	//WayPoint20
	//WayPoint4
	//WayPoint5
	//WayPoint7
	//WayPoint1
	//WayPoint22
	//WayPoint12
	//WayPoint16
	//WayPoint19
	//WayPoint20
	//WayPoint4
	//WayPoint5
	//WayPoint7
	//WayPoint1

    string waypId = "WayPoint16";


    vector<string> allSweeps;
	allSweeps = getSweepXmlsForTopologicalWaypoint<PointType>(folderPath, waypId);

	if(false){
		auto sweep_xmls = summary_parser.getRooms();
		for (size_t i=0; i<sweep_xmls.size(); i++){
		    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), false);
			printf("%s\n",sweep.roomWaypointId.c_str());
			allSweeps.push_back(sweep_xmls[i].roomXmlFile);
		}
	}

    unsigned int nr_sweeps = allSweeps.size();

	for (unsigned int i = 0; i < nr_sweeps; i++){
		if(i < start_sweep || i > stop_sweep){continue;}
		rc->addToTraining(allSweeps[i]);
	}
exit(0);





	Camera * camera = 0;//new Camera(fx, fy, cx, cy, width,	height);
	vector<Sweep *> sweeps;
	vector<Frame *> all_frames;
	vector<string> idtags;
	float * rgb = 0;
	float * depth = 0;

	for (unsigned int i = 0; i < nr_sweeps; i++){
		if(i < start_sweep || i > stop_sweep){continue;}
		rc->addToTraining(allSweeps[i]);
		continue;
        SimpleXMLParser<PointType>::RoomData roomData = simple_parser.loadRoomFromXML(allSweeps[i]);

		if(roomData.vIntermediateRoomClouds.size() < gx*gy){continue;}
		float fx = 540.0;//roomData.vIntermediateRoomCloudCamParams[i].fx();
		float fy = 540.0;//roomData.vIntermediateRoomCloudCamParams[i].fy();
		float cx = 319.5;//roomData.vIntermediateRoomCloudCamParams[i].cx();
		float cy = 239.5;//roomData.vIntermediateRoomCloudCamParams[i].cy();
		
		cv::Size res = roomData.vIntermediateRoomCloudCamParams[0].fullResolution();
		unsigned int height	= res.height; 
		unsigned int width	= res.width;

		if(camera == 0){
			camera = new Camera(fx, fy, cx, cy, width,	height);
			camera->version = 1;
			rgb		=	new float[3*width*height];
			depth	=	new float[	width*height];
		}
		vector<Frame *> sweep_frames;

		int counter = 0;
		sweep_frames.resize(todox*todoy);
		for(unsigned int y = 0; y < todoy; y++){//Camera moving forward
			for (unsigned int x = 0; x < todox ; x++){

				pcl::PointCloud<PointType>::Ptr clouddata = roomData.vIntermediateRoomClouds[inds[x][y]];//Crashes on 145, 
				for(unsigned int w = 0; w < width; w++){
					for(unsigned int h = 0; h < height; h++){
						unsigned int ind = h*width+w;
						unsigned int ind3 = 3*ind;
						rgb[ind3+0] = clouddata->points.at(ind).r;	
						rgb[ind3+1] = clouddata->points.at(ind).g;
						rgb[ind3+2] = clouddata->points.at(ind).b;
						depth[ind] = clouddata->points.at(ind).z;
					}
				}
				sweep_frames.at(y*todox+x) = new Frame(camera,rgb,depth);
				sweep_frames.at(y*todox+x)->framepos = inds[x][y];
				all_frames.push_back(sweep_frames.at(y*todox+x));
			}
		}

		sweeps.push_back(new Sweep(todox, todoy, sweep_frames));
		sweeps.back()->idtag = roomData.roomWaypointId;
        sweeps.back()->xmlpath = allSweeps[i];
	}
	printf("ceres optimizer stuff\n");

	double *** poses = new double**[todox];
	for(unsigned int x = 0; x < todox; x++){
		poses[x] = new double*[todoy]; 
		for(unsigned int y = 0; y < todoy; y++){
			poses[x][y] = new double[6];
			for(unsigned int k = 0; k < 6; k++){poses[x][y][k] = 0;}
		}
	}

	using namespace Eigen;

	ceres::Problem problem;
	Solver::Options options;
	options.max_num_iterations = 1500;
	options.minimizer_progress_to_stdout = true;
	options.num_linear_solver_threads = 7;
	Solver::Summary summary;

//1st forward X loop
	vector< vector< ProblemFrameConnection * > > x1_vec;
	x1_vec.resize(todoy);
	for(unsigned int s = 0; s < sweeps.size() && s < sweeps_for_training; s++){
		printf("1st forward X loop: %i\n",s);
		Sweep * sweep = sweeps.at(s);
		for(unsigned int x = 0; x < todox-1; x++){
			for(unsigned int y = 0; y < todoy; y++){
				ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x+1][y], shared_params, poses[x][y], poses[x+1][y]);
				x1_vec.at(y).push_back(pc);
			}
		}
	}

	for(unsigned int y = 0; y < todoy; y++){
		vector< CostFunction * > matches = getMatchesRansac(x1_vec.at(y));
		for(unsigned int i = 0; i < x1_vec.at(y).size(); i++){
			x1_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
		}
	}

	Solve(options, &problem, &summary);
	//std::cout << summary.FullReport() << "\n";

//1st forward Y loop
	vector< vector< ProblemFrameConnection * > > y1_vec;
	y1_vec.resize(todox);
	for(unsigned int s = 0; s < sweeps.size() && s < sweeps_for_training; s++){
		printf("1st forward Y loop: %i\n",s);
		Sweep * sweep = sweeps.at(s);
		for(unsigned int x = 0; x < todox; x++){
			for(unsigned int y = 0; y < todoy-1; y++){
				ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x][y+1], shared_params, poses[x][y], poses[x][y+1]);
				y1_vec.at(x).push_back(pc);
			}
		}
	}

	for(unsigned int x = 0; x < todox; x++){
		vector< CostFunction * > matches = getMatchesRansac(y1_vec.at(x));
		for(unsigned int i = 0; i < y1_vec.at(x).size(); i++){
			y1_vec.at(x).at(i)->addMatchesToProblem(problem, matches);
		}
	}

	Solve(options, &problem, &summary);
	//std::cout << summary.FullReport() << "\n";

//2nd forward X loop
	vector< vector< ProblemFrameConnection * > > x2_vec;
	x2_vec.resize(todoy);
	for(unsigned int s = 0; s < sweeps.size() && s < sweeps_for_training; s++){
		printf("2t forward X loop: %i\n",s);
		Sweep * sweep = sweeps.at(s);
		for(unsigned int x = 0; x < todox-2; x++){
			for(unsigned int y = 0; y < todoy; y++){
				ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x+2][y], shared_params, poses[x][y], poses[x+2][y]);
				x2_vec.at(y).push_back(pc);
			}
		}
	}

	for(unsigned int y = 0; y < todoy; y++){
		vector< CostFunction * > matches = getMatchesRansac(x2_vec.at(y));
		for(unsigned int i = 0; i < x2_vec.at(y).size(); i++){
			x2_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
		}
	}
	
	Solve(options, &problem, &summary);
	//std::cout << summary.FullReport() << "\n";
/*
	camera->fx = 1.0/shared_params[0];	camera->fy = 1.0/shared_params[1];	camera->cx = shared_params[2];		camera->cy = shared_params[3];
	camera->print();
	for(unsigned int s = 0; s < sweeps.size(); s++){
		for(unsigned int x = 0; x < todox; x++){
			for(unsigned int y = 0; y < todoy; y++){
				sweeps.at(s)->poses[x][y] = (getMat(poses[0][0]).inverse()*getMat(poses[x][y])).cast<float>();
			}
		}
	}
*/

//Loop closure
	vector< vector< ProblemFrameConnection * > > loop_vec;
	loop_vec.resize(todoy);
	for(unsigned int s = 0; s < sweeps.size() && s < sweeps_for_training; s++){
		printf("Loop closure: %i\n",s);
		Sweep * sweep = sweeps.at(s);
		for(unsigned int y = 0; y < todoy; y++){
			ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[0][y],sweep->frames[todox-1][y], shared_params, poses[0][y], poses[todox-1][y],1,false);
			loop_vec.at(y).push_back(pc);
		}
	}

	for(unsigned int y = 0; y < todoy; y++){
		vector< CostFunction * > matches = getMatchesRansac(loop_vec.at(y),x1_vec.at(y).size());
		for(unsigned int i = 0; i < loop_vec.at(y).size(); i++){
			loop_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
		}
	}
	Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";

	//Optimize camera parameter
	optimizeCameraParams = true;
	//Solve(options, &problem, &summary);
	//std::cout << summary.FullReport() << "\n";

	camera->fx = 1.0/shared_params[0];	camera->fy = 1.0/shared_params[1];	camera->cx = shared_params[2];		camera->cy = shared_params[3];
	camera->print();
	for(unsigned int s = 0; s < sweeps.size(); s++){
		for(unsigned int x = 0; x < todox; x++){
			for(unsigned int y = 0; y < todoy; y++){
				sweeps.at(s)->poses[x][y] = (getMat(poses[0][0]).inverse()*getMat(poses[x][y])).cast<float>();
			}
		}
	}


	vector< Sweep * > donesweeps;
	while(sweeps.size() > 0){
		Sweep * sweep = sweeps.back();
		donesweeps.push_back(sweep);
		sweeps.pop_back();
		printf("%s\n",sweep->idtag.c_str());
		for(unsigned int i = 0; i < sweeps.size(); i++){
			Sweep * s = sweeps.at(i);
			if((sweep->idtag.compare(s->idtag))  == 0){
				sweeps.at(i) = sweeps.back();
				sweeps.pop_back();
				i--;
				printf("%s\n",s->idtag.c_str());
				sweep->align(s);

				donesweeps.push_back(s);
			}
		}
	}

	for(unsigned int i = 0; i < donesweeps.size(); i++){saveSweep(donesweeps.at(i));}

	return 0;
}
