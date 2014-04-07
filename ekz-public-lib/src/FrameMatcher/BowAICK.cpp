#include "BowAICK.h"
#include <vector>

using namespace std;
const bool debugg_BowAICK = false;

BowAICK::BowAICK()
{
	debugg = false;
	verbose = false;
	name = "BowAICK";
	nr_iter = 30;
	
	feature_scale = 1;
	
	bow_threshold = 0.2;
	
	distance_threshold = 0.025f;
	feature_threshold = 0.15f;
	shrinking = 0.8f;
	stabilety_threshold = 0.000001f;
	max_points = 100000;

	converge = 1;
	iteration_shrinking = true;
	fitness_shrinking = false;
	fitness_constant = 0.1;
}

BowAICK::BowAICK(int max_points_)
{
	debugg = false;
	verbose = false;
	name = "BowAICK";
	nr_iter = 30;
	
	feature_scale = 1;
	
	bow_threshold = 0.2;
	
	distance_threshold = 0.015f;
	feature_threshold = 0.15f;
	shrinking = 0.8f;
	stabilety_threshold = 0.000001f;
	max_points = max_points_;

	converge = 1;
	iteration_shrinking = true;
	fitness_shrinking = false;
	fitness_constant = 0.1;
}

BowAICK::BowAICK(int max_points_, int nr_iter_, float shrinking_,float bow_threshold_, float distance_threshold_,float feature_threshold_){
	debugg = false;
	verbose = false;
	name = "BowAICK";
	max_points = max_points_;
	nr_iter = nr_iter_;
	shrinking = shrinking_;
	bow_threshold = bow_threshold_;
	distance_threshold = distance_threshold_;
	feature_threshold = feature_threshold_;
	stabilety_threshold = 0.000001f;
	feature_scale = 1;
	converge = 1;
	iteration_shrinking = true;
	fitness_shrinking = false;
	fitness_constant = 0.1;
}

BowAICK::~BowAICK(){printf("delete BowAICK\n");}

bool converged(Matrix4f change, float trans_min, float rot_min){
	float rotchange = 0;
	float transchange = 0;
	for(int i = 0; i < 3 ; i++){
		for(int j = 0; j < 3 ; j++){
			if(i == j){	rotchange+=(1-change(i,j))*(1-change(i,j));}
			else{		rotchange+=change(i,j)*change(i,j);}
		}
		transchange+=change(i,3)*change(i,3);
	}
	//printf("change: %f %f\n",sqrt(transchange),rotchange);
	if(sqrt(transchange) < trans_min && rotchange < rot_min){		return true;}
	else{															return false;}
}

float BowAICK::getAlpha(int iteration){return 1-pow(shrinking,float(iteration));}

float BowAICK::getAlpha(float avg_d2, int iteration){
	float a = pow(shrinking,float(iteration));

	float part = fitness_constant*avg_d2/distance_threshold;
	if(part > 1){part = 1;}
/*
	float in_exp = -10*(avg_d2-0.5*distance_threshold);
	if(in_exp > 0){in_exp = 0;}
	float b = 2/(1+exp(in_exp)) -1;
*/

	float b = -2*part*part*part + 3*part*part;
	//printf("b: %f = %f + %f\n",b,-2*part*part*part,3*part*part);

	if(!iteration_shrinking)	{a = 1;}
	if(!fitness_shrinking)		{b = 1;}
	//printf("iter: %i alpha: %f\n",iteration,1-a*b);
	return 1-a*b;
}

Transformation * BowAICK::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = transformationMat;
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 100;

	if(debugg_BowAICK){printf("BowAICK::getTransformation(%i,%i)\n",src->id,dst->id);}
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_BowAICK){
		IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_src = (char *)rgb_img_src->imageData;
		IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_dst = (char *)rgb_img_dst->imageData;
		width = rgb_img_src->width;
		height = rgb_img_src->height;
		img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
		char * data = (char *)img_combine->imageData;
		for (int j = 0; j < height; j++){
			for (int i = 0; i < width; i++){
				int ind = 3*(640*j+i);
				data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
				data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
				data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];
				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +2];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
			}
		}
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );
	}
	
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	


	vector<KeyPoint * > src_keypoints;
	for(int i = 0; i < nr_loop_src; i++){src_keypoints.push_back(src->keypoints->valid_key_points.at(i));}

	vector<KeyPoint * > dst_keypoints;
	for(int i = 0; i < nr_loop_dst; i++){dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));}
	
	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();
	
	int nr_bow = src->input->calibration->words.size();
	vector<int> * bow = new vector<int>[nr_bow];
	for(int i = 0; i < nr_bow; i++){bow[i] = vector<int>();}
	for(unsigned int i = 0; i < dst_keypoints.size(); i++){
		KeyPoint * kp = dst_keypoints.at(i);
		if(kp->cluster_distance_pairs.size()>0){
			int id = kp->cluster_distance_pairs.at(0).first;
			bow[id].push_back(i);
		}
	}

	vector< vector< int > > possible_matches;
	float ** feature_distances = new float*[src_nr_points];
	for(int i = 0; i < src_nr_points; i++){
		possible_matches.push_back(vector< int >());
		KeyPoint * src_kp = src_keypoints.at(i);
		//printf("%i gives %i -> ",i,src_kp->cluster_distance_pairs.size());
		for(unsigned int j = 0; j < src_kp->cluster_distance_pairs.size(); j++){
			float d = src_kp->cluster_distance_pairs.at(j).second;
			//printf("%5.5f ",d);
			int id = src_kp->cluster_distance_pairs.at(j).first;
			if(d < bow_threshold){
				vector<int> vec = bow[id];
				for(unsigned int k = 0; k < vec.size(); k++){
					possible_matches.at(i).push_back(vec.at(k));
				}
			}else{break;}
		}
		//printf("\n");
		FeatureDescriptor * descriptorA = src_kp->descriptor;
		int dst_nr_matches = possible_matches.at(i).size();
		feature_distances[i] = new float[dst_nr_matches];		
		for(int j = 0; j < dst_nr_matches;j++){
			FeatureDescriptor * descriptorB = dst_keypoints.at(possible_matches.at(i).at(j))->descriptor;
			feature_distances[i][j] = feature_scale*(descriptorA->distance(descriptorB));
		}
	}
	
/*
	for(int i = 0; i < src_nr_points;i++){
		FeatureDescriptor * descriptorA = src_keypoints.at(i)->descriptor;
		int dst_nr_matches = possible_matches.at(i).size();
		feature_distances[i] = new float[dst_nr_matches];		
		for(int j = 0; j < dst_nr_matches;j++)
		{
			FeatureDescriptor * descriptorB = dst_keypoints.at(possible_matches.at(i).at(j))->descriptor;
			feature_distances[i][j] = feature_scale*(descriptorA->distance(descriptorB));
		}
	}
*/
	float * pos_src_x 	= new float[src_nr_points];
	float * pos_src_y 	= new float[src_nr_points];
	float * pos_src_z 	= new float[src_nr_points];
	for(int i = 0; i < src_nr_points; i++)
	{
		pos_src_x[i] = src_keypoints.at(i)->point->x;
		pos_src_y[i] = src_keypoints.at(i)->point->y;
		pos_src_z[i] = src_keypoints.at(i)->point->z;
	}	
	
	float * pos_dst_x 	= new float[dst_nr_points];
	float * pos_dst_y 	= new float[dst_nr_points];
	float * pos_dst_z 	= new float[dst_nr_points];
	for(int i = 0; i < dst_nr_points; i++)
	{
		pos_dst_x[i] = dst_keypoints.at(i)->point->x;
		pos_dst_y[i] = dst_keypoints.at(i)->point->y;
		pos_dst_z[i] = dst_keypoints.at(i)->point->z;
	}
	float alpha = getAlpha(999999999,0);
	int iter = 0;
	for(; iter < nr_iter; iter++){
		//printf("alpha= %f\n",alpha);
		pcl::TransformationFromCorrespondences tfc;
		float threshold = distance_threshold*alpha + (1 - alpha)*feature_threshold;
		transformation->weight = 0;
		int nr_matches = 0;
		
		float mat00 = transformationMat(0,0);
		float mat01 = transformationMat(0,1);
		float mat02 = transformationMat(0,2);
		float mat03 = transformationMat(0,3);
		float mat10 = transformationMat(1,0);
		float mat11 = transformationMat(1,1);
		float mat12 = transformationMat(1,2);
		float mat13 = transformationMat(1,3);
		float mat20 = transformationMat(2,0);
		float mat21 = transformationMat(2,1);
		float mat22 = transformationMat(2,2);
		float mat23 = transformationMat(2,3);
		
		IplImage * img_combine_clone;
		if(debugg_BowAICK){
			img_combine_clone = cvCreateImage(cvSize(img_combine->width, img_combine->height), IPL_DEPTH_8U, 3);
			cvCopy( img_combine, img_combine_clone, NULL );
		}
		float wei = 0;
		vector<KeyPoint * > inlier_src_kp;
		vector<KeyPoint * > inlier_dst_kp;
		for(int i = 0; i < src_nr_points;i++)
		{
			float x_tmp = pos_src_x[i];
			float y_tmp = pos_src_y[i];
			float z_tmp = pos_src_z[i];
				
			float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
			float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
			float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;

			float dx,dy,dz;
			int dst_nr_matches = possible_matches.at(i).size();
			float best_d = 100000000;
			int best_j = -1;
			for(int jj = 0; jj < dst_nr_matches;jj++)
			{
				int j = possible_matches.at(i).at(jj);
				dx = x-pos_dst_x[j];
				dy = y-pos_dst_y[j];
				dz = z-pos_dst_z[j];

				float d = (1-alpha)*feature_distances[i][jj] + alpha*sqrt(dx*dx + dy*dy + dz*dz);
				if(d < best_d){
					best_d = d;
					best_j = j;
				}
			}
			if(best_d < threshold && best_j != -1){
				wei++;
				KeyPoint * src_kp = src_keypoints.at(i);
				KeyPoint * dst_kp = dst_keypoints.at(best_j);
				if(debugg_BowAICK){
					cvCircle(img_combine_clone,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvCircle(img_combine_clone,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvLine(img_combine_clone,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
				}
				tfc.add(src_kp->point->pos, dst_kp->point->pos);
				inlier_src_kp.push_back(src_kp);
				inlier_dst_kp.push_back(dst_kp);
				if(iter == nr_iter-1){
					transformation->weight++;
					transformation->matches.push_back(make_pair (src_kp, dst_kp));
				}
			}
		}
		
		Matrix4f last = transformationMat;
		transformationMat = tfc.getTransformation().matrix();
		Matrix4f change = last.inverse()*transformationMat;
		transformation->transformationMatrix = transformationMat;
		//sleep(1);	
		if(alpha >= converge && converged(change,1e-4,1e-5)){
			transformation->weight = inlier_src_kp.size();
			//printf("-------------------------BREAK----------------------\n");
			break;
		}
		
		
		mat00 = transformationMat(0,0);
		mat01 = transformationMat(0,1);
		mat02 = transformationMat(0,2);
		mat03 = transformationMat(0,3);
		mat10 = transformationMat(1,0);
		mat11 = transformationMat(1,1);
		mat12 = transformationMat(1,2);
		mat13 = transformationMat(1,3);
		mat20 = transformationMat(2,0);
		mat21 = transformationMat(2,1);
		mat22 = transformationMat(2,2);
		mat23 = transformationMat(2,3);

		float fit = 0;
		for(unsigned int i = 0; i < inlier_src_kp.size(); i++){
				Point * src_p = inlier_src_kp.at(i)->point;
				float x_tmp = src_p->x;
				float y_tmp = src_p->y;
				float z_tmp = src_p->z;
				
				float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
				float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
				float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;


				Point * dst_p = inlier_dst_kp.at(i)->point;
				float dx = x-dst_p->x;
				float dy = y-dst_p->y;
				float dz = z-dst_p->z;

				fit += sqrt(dx*dx + dy*dy + dz*dz);
		}
		fit /= float(inlier_src_kp.size());
		//printf("%i -> fit: %f inlier_src_kp.size()\n",iter,fit);
		alpha = getAlpha(fit,iter+1);
		//usleep(100000);

		if(debugg_BowAICK ){
			printf("%i -> fit: %f\n",iter,fit);
			transformation->printError();
			printf("weight: %f\n",float(wei));
			//if(iter == nr_iter-1 && transformation->weight < 50)
			{
				cvShowImage("combined image", img_combine_clone);
				cvWaitKey(0);
			}
			cvReleaseImage( &img_combine_clone);
		}
	}
	//cout<< "transformation->transformationMatrix\n" <<transformation->transformationMatrix<<endl;
	if(debugg_BowAICK){printf("done\n");cvReleaseImage( &img_combine );}
	
	for(int i = 0; i < src_nr_points;i++){delete[] feature_distances[i];}
	delete[] feature_distances;
	
	delete[] pos_src_x;
	delete[] pos_src_y;
	delete[] pos_src_z;
	
	delete[] pos_dst_x;
	delete[] pos_dst_y;
	delete[] pos_dst_z;
	
	delete[] bow;
	

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	if(verbose){printf("BowAICK cost: %f, weight: %f\n",time,transformation->weight);}
	//transformation->print();
	//if(transformation->weight > 5){printf("BowAICK cost: %f, weight: %f\n",time,transformation->weight);}
	transformation->time = time;
	return transformation;
}
