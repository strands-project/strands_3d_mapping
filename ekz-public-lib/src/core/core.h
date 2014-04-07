#ifndef core_H_
#define core_H_
#include "RGBDFrame.h"
#include "Transformation.h"

float *** render(Eigen::Matrix4f & tm, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float cx,float cy,float fx,float fy){
	float centerX		= cx;
	float centerY		= cy;
	float invFocalX		= 1.0f/fx;
    float invFocalY		= 1.0f/fy;

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

	float *** ret = new float**[4];
	for(int k = 0; k < 4; k++){
		ret[k] = new float*[640];
		for(int i = 0; i < 640; i++){
			ret[k][i] = new float[480];
			for(int j = 0; j < 480; j++){
				ret[k][i][j] = -1;
			}
		}
	}


	for(unsigned int i = 0; i < cloud->points.size(); i++){
		pcl::PointXYZRGB & p =  cloud->points[i];
		float x_tmp = p.x;
		float y_tmp = p.y;
		float z_tmp = p.z;

		float r = p.r;
		float g = p.g;
		float b = p.b;
				
		float x = x_tmp*mat00+y_tmp*mat01+z_tmp*mat02+mat03;
		float y = x_tmp*mat10+y_tmp*mat11+z_tmp*mat12+mat13;
		float z = x_tmp*mat20+y_tmp*mat21+z_tmp*mat22+mat23;


		float w	= x/(z * invFocalX) + centerX;
		float h	= y/(z * invFocalY) + centerY;

		int ww = 0.5+w;
		int hh = 0.5+h;
		if(ww>=0 && ww < 640 && hh >= 0 && hh < 480){
			float found_z = ret[3][ww][hh];
			if(found_z == -1 || z < found_z){
				ret[0][ww][hh] = r;
				ret[1][ww][hh] = g;
				ret[2][ww][hh] = b;
				ret[3][ww][hh] = z;
			}
		}
	}
	return ret;
}

#endif

