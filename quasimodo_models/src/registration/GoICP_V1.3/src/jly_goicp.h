/********************************************************************
Header File for Go-ICP Class
Last modified: Apr 21, 2014

"Go-ICP: Solving 3D Registration Efficiently and Globally Optimally"
Jiaolong Yang, Hongdong Li, Yunde Jia
International Conference on Computer Vision (ICCV), 2013

Copyright (C) 2013 Jiaolong Yang (BIT and ANU)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#ifndef JLY_GOICP_H
#define JLY_GOICP_H

#include <queue>
using namespace std;

#include "jly_icp3d.hpp"
#include "jly_3ddt.h"

#include <omp.h>

namespace goicp {

#define PI 3.1415926536
#define SQRT3 1.732050808

typedef struct _POINT3D
{
	float x, y, z;
}POINT3D;

typedef struct _ROTNODE
{
	float a, b, c, w;
	float ub, lb;
	int l;
	friend bool operator < (const struct _ROTNODE & n1, const struct _ROTNODE & n2)
	{
		if(n1.lb != n2.lb)
			return n1.lb > n2.lb;
		else
			return n1.w < n2.w;
			//return n1.ub > n2.ub;
	}
	
}ROTNODE;

typedef struct _TRANSNODE
{
	float x, y, z, w;
	float ub, lb;
	friend bool operator < (const struct _TRANSNODE & n1, const struct _TRANSNODE & n2)
	{
		if(n1.lb != n2.lb)
			return n1.lb > n2.lb;
		else
			return n1.w < n2.w;
			//return n1.ub > n2.ub;
	}
}TRANSNODE;

/********************************************************/



/********************************************************/

#define MAXROTLEVEL 20
#define NUM_THREADS 1

class GoICP
{
public:
	double max_time;

	bool prove_optimal;

	bool doneInnerBnB;

	pthread_t threads[NUM_THREADS];

	pthread_mutex_t working_thread_count_mutex;
	int working_thread_count = 0;

	pthread_mutex_t queueTrans_mutex;
	priority_queue<TRANSNODE> queueTrans;

	pthread_mutex_t current_optErrorT_mutex;
	float 			current_optErrorT;

	float 		*	current_maxRotDisL;
	TRANSNODE 	*	current_nodeTransOut;

	int runInnerBnBIteration(float * local_mindis);


	int Nm, Nd;
	POINT3D * pModel, * pData;

	ROTNODE initNodeRot;
	TRANSNODE initNodeTrans;

	DT3D dt;

	ROTNODE optNodeRot;
	TRANSNODE optNodeTrans;

	GoICP();
	float Register();
	void BuildDT();

	float MSEThresh;
	float SSEThresh;
	float icpThresh;

	float optError;
	Matrix optR;
	Matrix optT;

	clock_t clockBegin;

	float trimFraction;
	int inlierNum;
	bool doTrim;

	void InitializeBasic();
	void InitializeData();	
	void InitializeModel();
	
	void clearBasic();
	void clearData();	
	void clearModel();
	
	float OuterBnB();

private:
	//temp variables
	float * normData;
	float * minDis;
	float** maxRotDis;
	float * maxRotDisL;
	POINT3D * pDataTemp;
	POINT3D * pDataTempICP;
	
	ICP3D<float> icp3d;
	float * M_icp;
	float * D_icp;

	float ICP(Matrix& R_icp, Matrix& t_icp);
	float InnerBnB(float* maxRotDisL, TRANSNODE* nodeTransOut);
	
	void Initialize();
	void Clear();

};

};

/********************************************************/



#endif
