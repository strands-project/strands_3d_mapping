#include "FrameInput.h"

FrameInput::FrameInput(){}

FrameInput::FrameInput(Calibration * cal, pcl::PointCloud<pcl::PointXYZRGB> & input_cloud, bool save, int i, string path){
	rgb_img							= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_8U, 3);
	depth_img						= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_16U,1);

	char * rgb_data					= (char *)(rgb_img->imageData);
	unsigned short * depth_data		= (unsigned short *)(depth_img->imageData);

	calibration = cal;

	width = input_cloud.width;
	height = input_cloud.height;

	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = h*input_cloud.width + w;
			rgb_data[3*ind+0] = int(input_cloud.points[ind].b);
			rgb_data[3*ind+1] = int(input_cloud.points[ind].g);
			rgb_data[3*ind+2] = int(input_cloud.points[ind].r);
			depth_data[ind]   = (unsigned short)(5000*input_cloud.points[ind].z);
		}
	}

	rgb_in_ram = true;
	depth_in_ram = true;

	//cvShowImage("input rgb", rgb_img);
	//cvShowImage("input depth", depth_img);
	//cvWaitKey(0);

	if(save){
		char buf[1024];

		sprintf(buf,"%s/RGB%.10i.png",path.c_str(),i);
		if(!cvSaveImage(buf,rgb_img)){printf("Could not save: %s\n",buf);}
		rgb_path = string(buf);
		rgb_last_use_timestamp = current_time();

		sprintf(buf,"%s/Depth%.10i.png",path.c_str(),i);
		if(!cvSaveImage(buf,depth_img)){printf("Could not save: %s\n",buf);}
		depth_path = string(buf);
		depth_last_use_timestamp = current_time();
	}
}

FrameInput::FrameInput(Calibration * cal, string rgbpath, string depthpath){
	calibration = cal;
	
	//printf("rgbpath: %s\n",rgbpath.c_str());
	//printf("depthpath: %s\n",depthpath.c_str());

	width = 640;
	height = 480;

	rgb_path		= rgbpath;
	depth_path		= depthpath;
	depth_in_ram	= false;
	rgb_in_ram		= false;
	load_rgb();
	load_depth();


	if(false && rgb_img!=0 && depth_img != 0){
		cvNamedWindow("input rgb", CV_WINDOW_AUTOSIZE );
		IplImage * img 					= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		char * img_data					= (char *)(img->imageData);
		char * rgb_data					= (char *)(rgb_img->imageData);
		unsigned short * depth_data		= (unsigned short *)(depth_img->imageData);

		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				int ind = h*width + w;
				if(depth_data[ind] == 0 && (w % 2 == 0) && (h % 2 == 0)){
					img_data[3*ind+0] = 255;
					img_data[3*ind+1] = 0;
					img_data[3*ind+2] = 255;
				}else{
					img_data[3*ind+0] = rgb_data[3*ind+0];
					img_data[3*ind+1] = rgb_data[3*ind+1];
					img_data[3*ind+2] = rgb_data[3*ind+2];
				}
			}
		}

		for(int w = 0; w < width; w+=10){
			for(int h = 0; h < height; h+=10){
				//cvCircle(img,cvPoint(w,h), 3,cvScalar(255, 0, 255, 0),1, 8, 0);
			}
		}

		cvShowImage("input rgb", img);
		cvWaitKey(0);
		cvReleaseImage( &img);
	}

}

FrameInput::~FrameInput(){}

double FrameInput::current_time(){
	struct timeval t;
	gettimeofday(&t, NULL);
	return double(t.tv_sec*1000000+t.tv_usec)/1000000.0;
}

void FrameInput::load_rgb(){
	if(!rgb_in_ram){
		//printf("load rgb\n");
		rgb_in_ram					= true;
		rgb_img 					= cvLoadImage(rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		rgb_last_use_timestamp		= current_time();
	}
}

void FrameInput::release_rgb(){		
	if(rgb_in_ram){
		printf("release_rgb\n");
		exit(0);
		rgb_in_ram = false;
		cvReleaseImage( &rgb_img );
	}
}

void FrameInput::load_depth(){		
	if(!depth_in_ram){
		//printf("load depth\n");
		depth_in_ram				= true;
		depth_img					= cvLoadImage(depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		depth_last_use_timestamp	= current_time();
	}
}

void FrameInput::release_depth(){
	if(depth_in_ram){
		depth_in_ram = false;
		cvReleaseImage( &depth_img );
	}
}

float *** FrameInput::full_getXYZ(int & w, int & h){
	load_depth();

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	w = width;
	h = height;
	float *** mat = new float**[3];
	mat[0] = new float*[width];
	mat[1] = new float*[width];
	mat[2] = new float*[width];
	
	for(int w = 0; w < width; w++){
		mat[0][w] = new float[height];
		mat[1][w] = new float[height];
		mat[2][w] = new float[height];

		for(int h = 0; h < height; h++){
			int ind = width*h+w;
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (w - centerX) * tmp_z * invFocalX;
		       	tmp_y = (h - centerY) * tmp_z * invFocalY;
			}else{tmp_x = tmp_y = tmp_z = NAN;}

			mat[0][w][h] = tmp_x;
			mat[1][w][h] = tmp_y;
			mat[2][w][h] = tmp_z;
		}
	}
	depth_last_use_timestamp		= current_time();
	return mat;
}

pcl::PointCloud<pcl::PointXYZRGB> FrameInput::getDiffCloud(Eigen::Matrix4f & tm, vector< float * > & points){
	load_depth();
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;

	float mat00 = tm(0,0);
	float mat01 = tm(0,1);
	float mat02 = tm(0,2);
	float mat03 = tm(0,3);
	float mat10 = tm(1,0);
	float mat11 = tm(1,1);
	float mat12 = tm(1,2);
	float mat13 = tm(1,3);
	float mat20 = tm(2,0);
	float mat21 = tm(2,1);
	float mat22 = tm(2,2);
	float mat23 = tm(2,3);

	unsigned int nr_points = points.size();
	int ind = 0;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width    = nr_points;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);
	
	for(unsigned int i = 0; i < nr_points; i++){
		float * vp = points.at(i);
	
		float x_tmp = vp[0];
		float y_tmp = vp[1];
		float z_tmp = vp[2];
				
		float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
		float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
		float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;

		int w	= int(0.5f+x/(z * invFocalX) + centerX);
		int h	= int(0.5f+y/(z * invFocalY) + centerY);

		if(w>=0 && w < width && h >= 0 && h < height){
			float z_img = float(depth_data[width*h+w]) * d_scaleing;
			if(z_img != 0 && z_tmp < 4 && z_img < 4){
				cloud.points[ind].x = x;
				cloud.points[ind].y = y;
				cloud.points[ind].z = z;
				cloud.points[ind].r = 255;
				cloud.points[ind].g = 255;
				cloud.points[ind].b = 255;
				ind++;
			}
		}
	}

	cloud.width    = ind;
	cloud.height   = 1;
	cloud.points.resize (cloud.width * cloud.height);
	return cloud;
}

void FrameInput::getDiff(Eigen::Matrix4f & tm, vector< float * > points, float * d, int & nr_valid ){
	load_depth();

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;

	float mat00 = tm(0,0);
	float mat01 = tm(0,1);
	float mat02 = tm(0,2);
	float mat03 = tm(0,3);
	float mat10 = tm(1,0);
	float mat11 = tm(1,1);
	float mat12 = tm(1,2);
	float mat13 = tm(1,3);
	float mat20 = tm(2,0);
	float mat21 = tm(2,1);
	float mat22 = tm(2,2);
	float mat23 = tm(2,3);

	unsigned int nr_points = points.size();
	nr_valid = 0;
	for(unsigned int i = 0; i < nr_points; i++){
		float * vp = points.at(i);
	
		float x_tmp = vp[0];
		float y_tmp = vp[1];
		float z_tmp = vp[2];
				
		float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
		float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
		float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;

		int w	= int(0.5f+x/(z * invFocalX) + centerX);
		int h	= int(0.5f+y/(z * invFocalY) + centerY);

		if(w>=0 && w < width && h >= 0 && h < height){
			float z_img = float(depth_data[width*h+w]) * d_scaleing;
			if(z_img != 0 && z_tmp < 4 && z_img < 4){
				d[nr_valid] = z-z_img;
				nr_valid++;
			}
		}
	}
}

void FrameInput::getWH(float ** x, float ** y, float ** z,float **  w, float **  h){
	load_depth();
	
	printf("getWH\n");
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;

	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			if(!isnan(x[i][j])){
				w[i][j]	= int(0.5f+x[i][j]/(z[i][j] * invFocalX) + centerX);
				h[i][j]	= int(0.5f+y[i][j]/(z[i][j] * invFocalY) + centerY);
			}else{
				w[i][j]=h[i][j]=NAN;
			}
		}
	}

	depth_last_use_timestamp		= current_time();
}

void FrameInput::getWH(float x, float y, float z,int & w, int & h){
	load_depth();
	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;

	w 					= int(0.5f+x/(z * invFocalX) + centerX);
	h 					= int(0.5f+y/(z * invFocalY) + centerY);

	depth_last_use_timestamp		= current_time();
}

void FrameInput::getXYZ(float & x, float & y, float & z,float w, float h){
	load_depth();

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;

	int ww = w + 0.5f;
	int hh = h + 0.5f;
	int ind = width*hh+ww;

/*
int w0 = w;
int h0 = h;
int w1 = w+1;
int h1 = h+1;
printf("%f %f-> %i %i, %i %i\n",w,h,w0,h0,w1,h1);
*/
	float tmp_z = float(depth_data[ind]) * d_scaleing;
	//printf("%i %i depth_data[ind] = %i\n",w,h,depth_data[ind]);
	float tmp_x = 0;
	float tmp_y = 0;


	if(tmp_z > 0){
		tmp_x = (w - centerX) * tmp_z * invFocalX;
       	tmp_y = (h - centerY) * tmp_z * invFocalY;
	}else{
		tmp_z = -1.0f;
		tmp_x = (w - centerX) * tmp_z * invFocalX;
       	tmp_y = (h - centerY) * tmp_z * invFocalY;
	}
	x = tmp_x;
	y = tmp_y;
	z = tmp_z;

	//printf("x,y,z: %f %f %f\n",x,y,z);
	//if(depth_data[ind] != 0){printf("w h d = %i %i %i x y z = %f %f %f\n",w,h,depth_data[ind],x,y,z);}
	depth_last_use_timestamp		= current_time();
}
void FrameInput::vector_getXYZ(vector<float> & x, vector<float> & y, vector<float> & z, vector<int> & w_vec, vector<int> & h_vec){
	load_depth();

	x.resize(w_vec.size());
	y.resize(w_vec.size());
	z.resize(w_vec.size());

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;

	for(unsigned int i = 0; i < w_vec.size(); i++){
		int w = w_vec.at(i);
		int h = h_vec.at(i);
		int ind = h*width + w;
		float tmp_x = 0;
		float tmp_y = 0;
		float tmp_z = float(depth_data[ind]) * d_scaleing;
		
		if(tmp_z > 0){
			tmp_x = (w - centerX) * tmp_z * invFocalX;
		   	tmp_y = (h - centerY) * tmp_z * invFocalY;
		}else{tmp_x = tmp_y = tmp_z = NAN;}
		x.push_back(tmp_x);
		y.push_back(tmp_y);
		y.push_back(tmp_z);
	}

	depth_last_use_timestamp		= current_time();
}

float *** FrameInput::full_getRGB(int & w, int & h){
	load_rgb();
	char * rgb_data					= (char *)(rgb_img->imageData);
	w = width;
	h = height;
	float *** mat = new float**[3];
	mat[0] = new float*[width];
	mat[1] = new float*[width];
	mat[2] = new float*[width];

	for(int w = 0; w < width; w++){
		mat[0][w] = new float[height];
		mat[1][w] = new float[height];
		mat[2][w] = new float[height];

		for(int h = 0; h < height; h++){
			int ind = h*width + w;

			int tmp_r = char(rgb_data[3*ind+2]);
			int tmp_g = char(rgb_data[3*ind+1]);
			int tmp_b = char(rgb_data[3*ind+0]);

			if(tmp_r < 0){tmp_r = 255+tmp_r;}
			if(tmp_g < 0){tmp_g = 255+tmp_g;}
			if(tmp_b < 0){tmp_b = 255+tmp_b;}
			
			mat[0][w][h] = tmp_r;
			mat[1][w][h] = tmp_g;
			mat[2][w][h] = tmp_b;
		}
	}
	rgb_last_use_timestamp		= current_time();
	return mat;
}

void FrameInput::getRGB(float & r, float & g, float & b,int w, int h){
	load_rgb();
	char * rgb_data					= (char *)(rgb_img->imageData);
	int ind = h*width + w;
	r = char(rgb_data[3*ind+2]);
	g = char(rgb_data[3*ind+1]);
	b = char(rgb_data[3*ind+0]);

	if(r < 0){r = 255+r;}
	if(g < 0){g = 255+g;}
	if(b < 0){b = 255+b;}
	
	rgb_last_use_timestamp		= current_time();
}
void FrameInput::vector_getRGB(vector<float> & r, vector<float> & g, vector<float> & b, vector<int> & w_vec, vector<int> & h_vec){
	load_rgb();

	r.resize(w_vec.size());
	g.resize(w_vec.size());
	b.resize(w_vec.size());

	char * rgb_data					= (char *)(rgb_img->imageData);
	for(unsigned int i = 0; i < w_vec.size(); i++){
		int w = w_vec.at(i);
		int h = h_vec.at(i);
		int ind = h*width + w;
		r.at(i) = char(rgb_data[3*ind+2]);
		g.at(i) = char(rgb_data[3*ind+1]);
		b.at(i) = char(rgb_data[3*ind+0]);
		if(r.at(i) < 0){r.at(i) = 255+r.at(i);}
		if(g.at(i) < 0){g.at(i) = 255+g.at(i);}
		if(b.at(i) < 0){b.at(i) = 255+b.at(i);}
	}
	rgb_last_use_timestamp		= current_time();
}

IplImage * FrameInput::get_rgb_img(){
	//load_rgb();
	//rgb_last_use_timestamp		= current_time();
	return rgb_img;
}

pcl::PointCloud<pcl::PointXYZRGB> FrameInput::getCloud(){
	load_rgb();
	load_depth();
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width    = width;
	cloud.height   = height;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX		= 1.0f/calibration->fx;
    float invFocalY		= 1.0f/calibration->fy;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	char * rgb_data				= (char *)(rgb_img->imageData);

	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = h*width + w;

			int tmp_r = char(rgb_data[3*ind+2]);
			int tmp_g = char(rgb_data[3*ind+1]);
			int tmp_b = char(rgb_data[3*ind+0]);

			if(tmp_r < 0){tmp_r = 255+tmp_r;}
			if(tmp_g < 0){tmp_g = 255+tmp_g;}
			if(tmp_b < 0){tmp_b = 255+tmp_b;}

			float tmp_x = 0;
			float tmp_y = 0;
			float tmp_z = float(depth_data[ind]) * d_scaleing;
		
			if(tmp_z > 0){
				tmp_x = (w - centerX) * tmp_z * invFocalX;
			   	tmp_y = (h - centerY) * tmp_z * invFocalY;
			}
			cloud.points[ind].x = tmp_x;
			cloud.points[ind].y = tmp_y;
			cloud.points[ind].z = tmp_z;
			cloud.points[ind].r = tmp_r;
			cloud.points[ind].g = tmp_g;
			cloud.points[ind].b = tmp_b;
		}
	}
	rgb_last_use_timestamp = rgb_last_use_timestamp = current_time();
	return cloud;
}

float *** FrameInput::full_getNormals(int & w, int & h){
	pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals();
	w = width;
	h = height;
	float *** mat = new float**[3];
	mat[0] = new float*[width];
	mat[1] = new float*[width];
	mat[2] = new float*[width];
	for(int w = 0; w < width; w++){
		mat[0][w] = new float[height];
		mat[1][w] = new float[height];
		mat[2][w] = new float[height];
		for(int h = 0; h < height; h++){
			int ind = h*width + w;
			mat[0][w][h] = normals->points[ind].normal_x;
			mat[1][w][h] = normals->points[ind].normal_y;
			mat[2][w][h] = normals->points[ind].normal_z;
		}
	}
	return mat;
}

pcl::PointCloud<pcl::Normal>::Ptr FrameInput::getNormals(){
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud = getCloud();
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.04f);
	ne.setNormalSmoothingSize(6.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
	return normals;
}
