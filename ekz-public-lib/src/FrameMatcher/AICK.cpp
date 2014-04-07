#include "AICK.h"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
const bool debugg_AICK = false;

AICK::AICK()
{
	name = "AICK";
	nr_iter = 25;
	
	feature_scale = 1;
	
	distance_threshold = 0.01f * feature_scale;
	feature_threshold = 0.25f;
	shrinking = 0.7f;
	stabilety_threshold = 0.000001f;
	max_points = 100000;
}

AICK::AICK(int max_points_)
{
	name = "AICK";
	nr_iter = 25;
	
	feature_scale = 1;
	
	distance_threshold = 0.005f * feature_scale;
	feature_threshold = 0.25f;
	shrinking = 0.7f;
	stabilety_threshold = 0.000001f;
	max_points = max_points_;
}

AICK::AICK(int max_points_, int nr_iter_, float shrinking_, float distance_threshold_, float feature_threshold_){
	name = "AICK";
	nr_iter = nr_iter_;
	
	feature_scale = 1;
	
	distance_threshold = distance_threshold_ * feature_scale;
	feature_threshold = feature_threshold_;
	shrinking = shrinking_;
	stabilety_threshold = 0.000001f;
	max_points = max_points_;
}

AICK::~AICK(){printf("delete AICK\n");}

float AICK::getAlpha(int iteration){return 1-pow(shrinking,float(iteration));}

Transformation * AICK::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	//printf("start AICK...\n");
	struct timeval start, end;
	gettimeofday(&start, NULL);
	//printf("AICK::getTransformation(%i,%i)\n",src->id,dst->id);
	if(debugg_AICK){printf("AICK::getTransformation(%i,%i)\n",src->id,dst->id);}
	IplImage* img_combine;
	int width;
	int height;
	if(debugg_AICK)
	{	
		IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_src = (char *)rgb_img_src->imageData;
		IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		char * data_dst = (char *)rgb_img_dst->imageData;
		
		width = rgb_img_src->width;
		height = rgb_img_src->height;
		
		img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
		char * data = (char *)img_combine->imageData;
		


		int index = 0;
		
		for (int j = 0; j < height; j++)
		{
			for (int i = 0; i < width; i++)
			{
				
				int ind = 3*(640*j+i);
				data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
				data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
				data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];

				data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +0];
				data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +1];
				data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
				
			}
		}
		
		//cvNamedWindow("src image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("src image", rgb_img_src);
		
		//cvNamedWindow("dst image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("dst image", rgb_img_dst);
		
		//cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
		//cvShowImage("combined image", img_combine);
		
		//cvWaitKey(0);
		cvReleaseImage( &rgb_img_src );
		cvReleaseImage( &rgb_img_dst );

	}
	//printf("img-0...\n");
	vector<KeyPoint * > src_keypoints;//	= src->keypoints->valid_key_points;
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	
	for(int i = 0; i < nr_loop_src; i++)
	{
		if(src->keypoints->valid_key_points.at(i)->stabilety > stabilety_threshold){
			src_keypoints.push_back(src->keypoints->valid_key_points.at(i));
		}
	}

	vector<KeyPoint * > dst_keypoints;//	= dst->keypoints->valid_key_points;
	for(int i = 0; i < nr_loop_dst; i++)
	{
		if(dst->keypoints->valid_key_points.at(i)->stabilety > stabilety_threshold){
			dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));
		}
	}
	//printf("src_keypoints.size() = %i, dst_keypoints.size() = %i\n",int(src_keypoints.size()),int(dst_keypoints.size()));
	//if(debugg_AICK){printf("src_keypoints.size() = %i, dst_keypoints.size() = %i\n",int(src_keypoints.size()),int(dst_keypoints.size()));}
	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();
	float ** surf_distances = new float*[src_nr_points];
	float ** match_distances = new float*[src_nr_points];
	for(int i = 0; i < src_nr_points;i++)
	{
		//printf("i:%i\n",i);
		surf_distances[i] = new float[dst_nr_points];
		match_distances[i]= new float[dst_nr_points];		
		for(int j = 0; j < dst_nr_points;j++)
		{
			FeatureDescriptor * descriptorA = src_keypoints.at(i)->descriptor;
			FeatureDescriptor * descriptorB = dst_keypoints.at(j)->descriptor;
			surf_distances[i][j] = feature_scale*(descriptorA->distance(descriptorB));
		}
	}
	float * pos_src_x 				= new float[src_nr_points];
	float * pos_src_y 				= new float[src_nr_points];
	float * pos_src_z 				= new float[src_nr_points];
	for(int i = 0; i < src_nr_points; i++)
	{
		pos_src_x[i] = src_keypoints.at(i)->point->x;
		pos_src_y[i] = src_keypoints.at(i)->point->y;
		pos_src_z[i] = src_keypoints.at(i)->point->z;
	}	
	float * pos_src_x_transform 	= new float[src_nr_points];
	float * pos_src_y_transform 	= new float[src_nr_points];
	float * pos_src_z_transform 	= new float[src_nr_points];
	
	float * pos_dst_x_transform 	= new float[dst_nr_points];
	float * pos_dst_y_transform 	= new float[dst_nr_points];
	float * pos_dst_z_transform 	= new float[dst_nr_points];
	for(int i = 0; i < dst_nr_points; i++)
	{
		pos_dst_x_transform[i] = dst_keypoints.at(i)->point->x;
		pos_dst_y_transform[i] = dst_keypoints.at(i)->point->y;
		pos_dst_z_transform[i] = dst_keypoints.at(i)->point->z;
	}
//printf("line:%i\n",__LINE__);
	
	int * src_matches 				= new int[src_nr_points];
	int * dst_matches 				= new int[dst_nr_points];
	
	
	Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = transformationMat;
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 100;
	for(int iter = 0; iter < nr_iter; iter++)
	{
		float alpha = getAlpha(iter);
		//printf("alpha= %f\n",alpha);
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
		
		for(int i = 0; i < src_nr_points;i++)
		{
			float x = pos_src_x[i];
			float y = pos_src_y[i];
			float z = pos_src_z[i];
				
			pos_src_x_transform[i] = x*mat00+y*mat01+z*mat02+mat03;
			pos_src_y_transform[i] = x*mat10+y*mat11+z*mat12+mat13;
			pos_src_z_transform[i] = x*mat20+y*mat21+z*mat22+mat23;
		}
		
		for(int i = 0; i < src_nr_points;i++)
		{
			float x = pos_src_x_transform[i];
			float y = pos_src_y_transform[i];
			float z = pos_src_z_transform[i];
			float dx,dy,dz;
			for(int j = 0; j < dst_nr_points;j++)
			{
				dx = x-pos_dst_x_transform[j];
				dy = y-pos_dst_y_transform[j];
				dz = z-pos_dst_z_transform[j];

				match_distances[i][j] = (1-alpha)*surf_distances[i][j] + alpha*sqrt(dx*dx + dy*dy + dz*dz);
			}
		}
		

		for(int i = 0; i < src_nr_points;i++)
		{
			src_matches[i] = -1;
			float best_value = 9999999;
			for(int j = 0; j < dst_nr_points;j++)
			{
				float current = match_distances[i][j];
				if(current<best_value)
				{
					best_value = current;
					src_matches[i] = j;
				}
			}
		}
		pcl::TransformationFromCorrespondences tfc;
		//int nr_matches = 0;
		float threshold = distance_threshold*alpha + (1 - alpha)*feature_threshold;

		IplImage * img_combine_clone;
		if(debugg_AICK){
			img_combine_clone = cvCreateImage(cvSize(img_combine->width, img_combine->height), IPL_DEPTH_8U, 3);
			cvCopy( img_combine, img_combine_clone, NULL );
		}
		transformation->weight = 0;
		int nr_matches = 0;
		for(int i = 0; i < src_nr_points;i++)
		{
			int j = src_matches[i];
			
			if(match_distances[i][j] < threshold && j != -1){
				//if(src->id == 793){printf("j = %i\n",j);}
				KeyPoint * src_kp = src_keypoints.at(i);
				KeyPoint * dst_kp = dst_keypoints.at(j);
				if(debugg_AICK){
					cvCircle(img_combine_clone,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvCircle(img_combine_clone,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
					cvLine(img_combine_clone,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
				}
				nr_matches++;
				tfc.add(src_kp->point->pos, dst_kp->point->pos);
				
				if(iter == nr_iter-1){
					transformation->weight++;
					transformation->matches.push_back(make_pair (src_kp, dst_kp));
				}
			}
		}		
		//printf("nr_matches: %i\n",nr_matches);
		if(debugg_AICK ){
			//if(iter == nr_iter-1)
			{
				cvShowImage("combined image", img_combine_clone);
				char buf[1024];
				sprintf(buf,"combine_%i_%i_%i.png",src->id,dst->id,iter);
				if(!cvSaveImage(buf,img_combine_clone)){printf("Could not save: %s\n",buf);}
				cvWaitKey(0);
			}
			cvReleaseImage( &img_combine_clone);
		}
		
		transformationMat = tfc.getTransformation().matrix();
		transformation->transformationMatrix = transformationMat;
		if(nr_matches < 3){transformation->transformationMatrix = Eigen::Matrix4f::Identity(); break;}
	}
	if(debugg_AICK){printf("done\n");cvReleaseImage( &img_combine );}
	for(int i = 0; i < src_nr_points;i++)
	{
		delete[] surf_distances[i];
		delete[] match_distances[i];		
	}
	delete[] surf_distances;
	delete[] match_distances;
	
	delete[] pos_src_x;
	delete[] pos_src_y;
	delete[] pos_src_z;
	
	delete[] pos_src_x_transform;
	delete[] pos_src_y_transform;
	delete[] pos_src_z_transform;
	
	
	delete[] pos_dst_x_transform;
	delete[] pos_dst_y_transform;
	delete[] pos_dst_z_transform;
	
	delete[] src_matches;
	delete[] dst_matches;

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("AICK cost: %f\n",time);
	transformation->time = time;
	return transformation;
}
