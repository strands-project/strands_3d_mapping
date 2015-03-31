#include "strands_sweep_registration/Sweep.h"

#include "cv.h" 
#include "highgui.h"

#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;

Sweep::Sweep(int width_, int height_, std::vector<Frame *> framesvec){
	width = width_;
	height = height_;

	frames = new Frame**[width];
	poses = new Eigen::Matrix4f*[width];
	for(int w = 0; w < width; w++){
		frames[w] = new Frame*[height];
		poses[w] = new Eigen::Matrix4f[height];
	}
	


	int ind = 0;
	for(int h = 0; h < height; h++){
		for(int w = 0; w < width; w++){
			frames[w][h] = framesvec.at(ind);
			poses[w][h] = Eigen::Matrix4f::Identity();
			ind++;
		}
	}
}

Sweep::Sweep(){
	width			= 0;
	height			= 0;
	frames			= 0;
	poses			= 0;
};

Sweep::~Sweep(){};

static inline int popcount_lauradoux2(uint64_t *buf, uint32_t size) {
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

Eigen::Matrix4f Sweep::align(Sweep * sweep,float threshold, int ransac_iter, int nr_points){

	float centerX		= frames[0][0]->camera->cx;
	float centerY		= frames[0][0]->camera->cy;
	float invFocalX		= 1.0f/frames[0][0]->camera->fx;
    float invFocalY		= 1.0f/frames[0][0]->camera->fy;

	vector<Eigen::Vector4f> src_points;
	vector<Eigen::Vector4f> dst_points;

	for(int h = 0; h < height; h++){
		for(int w = 0; w < width; w++){
			Matrix4f src_pose = poses[w][h];
			Matrix4f dst_pose = sweep->poses[w][h];
			Frame * src = frames[w][h];
			Frame * dst = sweep->frames[w][h];

			int nr_src = src->keypoints.size();
			int nr_dst = dst->keypoints.size();
			if(nr_src == 0 || nr_dst == 0){continue;}

			float * best_src = new float[nr_src];
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

				unsigned int i4 = i*4;
				const uint64_t s1 = src_data[i4];
				const uint64_t s2 = src_data[i4+1];
				const uint64_t s3 = src_data[i4+2];
				const uint64_t s4 = src_data[i4+3];

				for(int j = 0; j < nr_dst; j++){
					float d;

					if(src->featuretype == 0){//ORB
						unsigned int j4 = j*4;
						const uint64_t d1 = dst_data[j4];
						const uint64_t d2 = dst_data[j4+1];
						const uint64_t d3 = dst_data[j4+2];
						const uint64_t d4 = dst_data[j4+3];

						xordata[0] = s1 ^ d1;
						xordata[1] = s2 ^ d2;
						xordata[2] = s3 ^ d3;
						xordata[3] = s4 ^ d4;

						int cnt = popcount_lauradoux2(xordata, 4);
						d = float(cnt)/256.0f;
					}

					if(src->featuretype == 1){//Sift
						d = cv::norm(src->descriptors.row(i)-dst->descriptors.row(j));
					}

					if(d < best_src[i]){
						best_src_id[i] = j;
						best_src[i] = d;
					}
				
					if(d < best_dst[j]){
						best_dst_id[j] = i;
						best_dst[j] = d;
					}
				}
			}
		
			for(int i = 0; i < nr_src; i++){
				int j = best_src_id[i];
				if(best_dst_id[j] != i){continue;}//One to one

				cv::KeyPoint src_kp = src->keypoints.at(i);
				cv::KeyPoint dst_kp = dst->keypoints.at(j);

				double sz	= (src->keypoint_location.at(i))(2);
				double sx	= (src_kp.pt.x - centerX) * sz * invFocalX;
				double sy	= (src_kp.pt.y - centerY) * sz * invFocalY;
	
				double dz	= (dst->keypoint_location.at(j))(2);	
				double dx	= (dst_kp.pt.x - centerX) * dz * invFocalX;
				double dy	= (dst_kp.pt.y - centerY) * dz * invFocalY;


				Eigen::Vector4f src_tmp = src_pose*Eigen::Vector4f(sx,sy,sz,1);
				Eigen::Vector4f dst_tmp = src_pose*Eigen::Vector4f(dx,dy,dz,1);

				src_points.push_back(src_pose*Eigen::Vector4f(sx,sy,sz,1));
				dst_points.push_back(dst_pose*Eigen::Vector4f(dx,dy,dz,1));
			}

			delete[] best_src;
			delete[] best_src_id;
			delete[] best_dst;
			delete[] best_dst_id;
		}
	}

//    printf("src_points: %d\n",src_points.size());

	int nr_kp = src_points.size();
	int nr_consistent = 0;
	
	vector<int> bestmatching;
	Eigen::Matrix4f retpose;

	for(int it = 0; it < ransac_iter; it++){
		//Sample points
		vector< int > indexes;
		vector< Eigen::Vector4f > src_samples;
		vector< Eigen::Vector4f > dst_samples;

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
				if(fabs(src_distance-dst_distance) > threshold){consistent = false; break;break;}  
			}
		}

		//Check inliers
		nr_consistent += consistent;
		if(consistent){
			pcl::TransformationFromCorrespondences tfc;
			for(int i = 0; i < nr_points; i++){tfc.add(dst_samples.at(i).head<3>(),src_samples.at(i).head<3>(),1);}
			Eigen::Affine3f ret = tfc.getTransformation();
			
			Eigen::Matrix4f pose = ret.matrix().inverse();
			
			std::vector<int> matching;
			for(int j = 0; j < nr_kp; j++){
				Eigen::Vector4f src_data = src_points.at(j);
				Eigen::Vector4f dst_data = dst_points.at(j);
				
				Eigen::Vector4f sd = pose*src_data;
				Eigen::Vector4f dd = dst_data;
				if((sd-dd).norm() < 0.005){matching.push_back(j);}
			}

			//save if best
			if(matching.size() > bestmatching.size()){bestmatching = matching; retpose = pose;}
			if(nr_consistent == 10000){printf("break at %i\n",it);break;}
		}
	}
	
	printf("nr matches best: %i / %i consistent: %i / %i\n",int(bestmatching.size()),nr_kp,nr_consistent,ransac_iter);

	for(int iter = 0; iter < 100; iter++){	
		pcl::TransformationFromCorrespondences tfc;	
		int matching = 0;
		for(int j = 0; j < nr_kp; j++){
			Eigen::Vector4f src_data = src_points.at(j);
			Eigen::Vector4f dst_data = dst_points.at(j);
				
            Eigen::Vector4f sd = retpose*src_data;
			Eigen::Vector4f dd = dst_data;
			if((sd-dd).norm() < 0.005){
				tfc.add(dst_data.head<3>(),src_data.head<3>(),1);
				matching++;				
			}
		}
		if(iter <= 10 || iter % 20 == 0){printf("iter: %i imatches: %i\n",iter,matching);}
		Eigen::Affine3f ret = tfc.getTransformation();
		retpose = ret.matrix().inverse();
	}

	cout << retpose << endl;

	for(int h = 0; h < height; h++){
		for(int w = 0; w < width; w++){
            sweep->poses[w][h] = retpose.inverse()*sweep->poses[w][h];
		}	
	}

    return retpose.inverse();
}

std::vector<Eigen::Matrix4f> Sweep::getPoseVector(){
	std::vector<Eigen::Matrix4f> vec;
	vec.resize(width*height);
	for(int h = 0; h < height; h++){
		for(int w = 0; w < width; w++){
			vec.at(frames[w][h]->framepos) = poses[w][h];
		}	
	}
	return vec;
}
