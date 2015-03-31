#include "strands_sweep_registration/util.h"

int popcount_lauradoux(uint64_t *buf, uint32_t size) {
	const uint64_t* data = (uint64_t*) buf;

	const uint64_t m1	= UINT64_C(0x5555555555555555);
	const uint64_t m2	= UINT64_C(0x3333333333333333);
	const uint64_t m4	= UINT64_C(0x0F0F0F0F0F0F0F0F);
	const uint64_t m8	= UINT64_C(0x00FF00FF00FF00FF);
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
			count1	=	data[j];
			count2	=	data[j+1];
			half1	=	data[j+2];
			half2	=	data[j+2];
			half1	&=	m1;
			half2	= (half2	>> 1) & m1;
			count1 -= (count1 >> 1) & m1;
			count2 -= (count2 >> 1) & m1;
			count1 +=	half1;
			count2 +=	half2;
			count1	= (count1 & m2) + ((count1 >> 2) & m2);
			count1 += (count2 & m2) + ((count2 >> 2) & m2);
			acc		+= (count1 & m4) + ((count1 >> 4) & m4);
		}
		acc = (acc & m8) + ((acc >>	8)	& m8);
		acc = (acc			 +	(acc >> 16)) & m16;
		acc =	acc			 +	(acc >> 32);
		bitCount += (uint32_t)acc;
	}

	for (i = 0; i < size - limit30; i++) {
		x = data[i];
		x =	x			 - ((x >> 1)	& m1);
		x = (x & m2) + ((x >> 2)	& m2);
		x = (x			 +	(x >> 4)) & m4;
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
