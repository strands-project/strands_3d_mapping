/********************************************************************
Implementation of Go-ICP Algorithm
Last modified: Jun 18, 2014

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

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
//using namespace std;

#include "jly_goicp.h"
#include "jly_sorting.hpp"

#include <unistd.h>
#include <pthread.h>

bool doprint = false;

#define MAXDATASIZE 1000000 //GOICP WONT WORK WELL WITH MORE THAN 1 million datapoints ANYHOW(more like 10K in reality)...

pthread_mutex_t innercount_mutex = PTHREAD_MUTEX_INITIALIZER;
int innercount = 0;

namespace goicp {

void * workerthread(void * threadid){
	cout << "Hello World! A" << endl;
	GoICP * goicp = (GoICP*)threadid;
	//goicp->print();

	float * local_mindis = new float[MAXDATASIZE];
	while(true){
		if(goicp->runInnerBnBIteration(local_mindis) == -1){
			usleep(100);
		}
	}
}

int GoICP::runInnerBnBIteration(float * local_mindis){

	if(doneInnerBnB){return -1;}

	//int i, j;
	//float transX, transY, transZ;
	//float lb, ub;
	float dis, maxTransDis;
	//TRANSNODE nodeTrans, nodeTransParent;
	TRANSNODE nodeTransParent;


	if(queueTrans.empty()){
		return -1;
	}

	working_thread_count++;

	nodeTransParent = queueTrans.top();
	queueTrans.pop();

	if(current_optErrorT-nodeTransParent.lb < SSEThresh){
		//doneInnerBnB = true;
		working_thread_count--;
		return -1;
	}

	innercount++;
	//printf("working..%i\n",innercount);
	//nodeTrans.w = nodeTransParent.w/2;
	//maxTransDis = SQRT3/2.0*nodeTrans.w;

	std::vector<TRANSNODE> tmpqueue;

//#pragma omp parallel for num_threads(2)
	for(int j = 0; j < 8; j++){
		TRANSNODE nodeTrans;
		nodeTrans.w = nodeTransParent.w/2;
		maxTransDis = SQRT3/2.0*nodeTrans.w;

		nodeTrans.x = nodeTransParent.x + (j&1)*nodeTrans.w ;
		nodeTrans.y = nodeTransParent.y + (j>>1&1)*nodeTrans.w ;
		nodeTrans.z = nodeTransParent.z + (j>>2&1)*nodeTrans.w ;

		float transX = nodeTrans.x + nodeTrans.w/2;
		float transY = nodeTrans.y + nodeTrans.w/2;
		float transZ = nodeTrans.z + nodeTrans.w/2;

		float ub = 0;
		float lb = 0;
//#define slowMinDistLoopibnb
#ifdef slowMinDistLoopibnb
		for(int i = 0; i < Nd; i++){
			minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			if(current_maxRotDisL){minDis[i] -= current_maxRotDisL[i];}
			if(minDis[i] < 0){minDis[i] = 0;}
		}
		for(int i = 0; i < inlierNum; i++){
			float fminDis = minDis[i]
			ub += errorcost(fminDis);
		}

		for(int i = 0; i < inlierNum; i++){
			// Subtract the translation uncertainty radius
			dis = minDis[i] - maxTransDis;
			if(dis > 0){lb += errorcost(dis);}
		}
#else
		if(current_maxRotDisL){
			for(int i = 0; i < Nd; i++){
				float minDisi = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
				minDisi -= current_maxRotDisL[i];
				if(minDisi < 0){minDisi = 0;}
				ub += minDisi*minDisi;
				float disi = minDisi - maxTransDis;
				if(disi > 0){lb += errorcost(disi);}
			}
		}else{
			for(int i = 0; i < Nd; i++){
				float minDisi = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
				if(minDisi < 0){minDisi = 0;}
				ub += errorcost(minDisi);
				float disi = minDisi - maxTransDis;
				if(disi > 0){lb += errorcost(disi);}
			}
		}
#endif

		// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
		pthread_mutex_lock( & queueTrans_mutex );
		if(ub < current_optErrorT){
			current_optErrorT = ub;
			if(current_nodeTransOut){*current_nodeTransOut = nodeTrans;}
		}

		// Remove subcube from queue if lb is bigger than optErrorT
		if(lb >= current_optErrorT){
			pthread_mutex_unlock( & queueTrans_mutex );
			continue;
		}

		nodeTrans.ub = ub;
		nodeTrans.lb = lb;

		queueTrans.push(nodeTrans);
		pthread_mutex_unlock( & queueTrans_mutex );
	}

	working_thread_count--;
	return 1;

/*
	if(doneInnerBnB){return -1;}
	pthread_mutex_lock( & queueTrans_mutex );

	int i, j;
	float transX, transY, transZ;
	float lb, ub;
	float dis, maxTransDis;
	TRANSNODE nodeTrans, nodeTransParent;


	if(queueTrans.empty()){
		pthread_mutex_unlock( & queueTrans_mutex );
		return -1;
	}

	pthread_mutex_lock( & working_thread_count_mutex );
	working_thread_count++;
	pthread_mutex_unlock( &working_thread_count_mutex );

	nodeTransParent = queueTrans.top();
	queueTrans.pop();
	//pthread_mutex_unlock( & queueTrans_mutex );

	if(current_optErrorT-nodeTransParent.lb < SSEThresh){
		//doneInnerBnB = true;
		pthread_mutex_lock( & working_thread_count_mutex );
		working_thread_count--;
		pthread_mutex_unlock( &working_thread_count_mutex );
		pthread_mutex_unlock( & queueTrans_mutex );
		return -1;
	}

	pthread_mutex_lock( & innercount_mutex );
	innercount++;
	pthread_mutex_unlock( & innercount_mutex );

	nodeTrans.w = nodeTransParent.w/2;
	maxTransDis = SQRT3/2.0*nodeTrans.w;

	pthread_mutex_unlock( & queueTrans_mutex );

	for(j = 0; j < 8; j++){

		pthread_mutex_lock( & queueTrans_mutex );

		nodeTrans.x = nodeTransParent.x + (j&1)*nodeTrans.w ;
		nodeTrans.y = nodeTransParent.y + (j>>1&1)*nodeTrans.w ;
		nodeTrans.z = nodeTransParent.z + (j>>2&1)*nodeTrans.w ;

		transX = nodeTrans.x + nodeTrans.w/2;
		transY = nodeTrans.y + nodeTrans.w/2;
		transZ = nodeTrans.z + nodeTrans.w/2;
#define slowMinDistLoop
#ifdef slowMinDistLoop
		for(i = 0; i < Nd; i++){
			minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			if(current_maxRotDisL){minDis[i] -= current_maxRotDisL[i];}
			if(minDis[i] < 0){minDis[i] = 0;}
		}
#else
		if(maxRotDisL){
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ) - maxRotDisL[i];
				if(minDis[i] < 0){minDis[i] = 0;}
			}
		}else{
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			}
		}
#endif

		// For each data point, find the incremental upper and lower bounds
		ub = 0;
		for(i = 0; i < inlierNum; i++){ub += minDis[i]*minDis[i];}

		lb = 0;
		for(i = 0; i < inlierNum; i++){
			// Subtract the translation uncertainty radius
			dis = minDis[i] - maxTransDis;
			if(dis > 0){lb += dis*dis;}
		}


		// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
		pthread_mutex_lock( & current_optErrorT_mutex );
		if(ub < current_optErrorT){
			current_optErrorT = ub;
			if(current_nodeTransOut){*current_nodeTransOut = nodeTrans;}
		}
		//pthread_mutex_unlock( & current_optErrorT_mutex );

		// Remove subcube from queue if lb is bigger than optErrorT
		if(lb >= current_optErrorT){
			pthread_mutex_unlock( & current_optErrorT_mutex );
			continue;
		}
		pthread_mutex_unlock( & current_optErrorT_mutex );

		nodeTrans.ub = ub;
		nodeTrans.lb = lb;

		//pthread_mutex_lock(		& queueTrans_mutex );
		queueTrans.push(nodeTrans);
		//pthread_mutex_unlock(	& queueTrans_mutex );
		pthread_mutex_unlock( & queueTrans_mutex );
	}
	pthread_mutex_lock( & working_thread_count_mutex );
	working_thread_count--;
	pthread_mutex_unlock( &working_thread_count_mutex );
	//pthread_mutex_unlock( & queueTrans_mutex );
	return 1;
*/
/*

	int i, j;
	float transX, transY, transZ;
	float lb, ub;
	float dis, maxTransDis;
	TRANSNODE nodeTrans, nodeTransParent;

	pthread_mutex_lock( & queueTrans_mutex );
	if(queueTrans.empty()){
		pthread_mutex_unlock( & queueTrans_mutex );
		return -1;
	}

	pthread_mutex_lock( & working_thread_count_mutex );
	working_thread_count++;
	pthread_mutex_unlock( &working_thread_count_mutex );

	nodeTransParent = queueTrans.top();
	queueTrans.pop();
	pthread_mutex_unlock( & queueTrans_mutex );

	if(current_optErrorT-nodeTransParent.lb < SSEThresh){
		//doneInnerBnB = true;
		pthread_mutex_lock( & working_thread_count_mutex );
		working_thread_count--;
		pthread_mutex_unlock( &working_thread_count_mutex );
		return -1;
	}

	pthread_mutex_lock( & innercount_mutex );
	innercount++;
	pthread_mutex_unlock( & innercount_mutex );

	nodeTrans.w = nodeTransParent.w/2;
	maxTransDis = SQRT3/2.0*nodeTrans.w;

	for(j = 0; j < 8; j++){
		nodeTrans.x = nodeTransParent.x + (j&1)*nodeTrans.w ;
		nodeTrans.y = nodeTransParent.y + (j>>1&1)*nodeTrans.w ;
		nodeTrans.z = nodeTransParent.z + (j>>2&1)*nodeTrans.w ;

		transX = nodeTrans.x + nodeTrans.w/2;
		transY = nodeTrans.y + nodeTrans.w/2;
		transZ = nodeTrans.z + nodeTrans.w/2;
#define slowMinDistLoop
#ifdef slowMinDistLoop
		for(i = 0; i < Nd; i++){
			minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			if(current_maxRotDisL){minDis[i] -= current_maxRotDisL[i];}
			if(minDis[i] < 0){minDis[i] = 0;}
		}
#else
		if(maxRotDisL){
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ) - maxRotDisL[i];
				if(minDis[i] < 0){minDis[i] = 0;}
			}
		}else{
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			}
		}
#endif

		// For each data point, find the incremental upper and lower bounds
		ub = 0;
		for(i = 0; i < inlierNum; i++){ub += minDis[i]*minDis[i];}

		lb = 0;
		for(i = 0; i < inlierNum; i++){
			// Subtract the translation uncertainty radius
			dis = minDis[i] - maxTransDis;
			if(dis > 0){lb += dis*dis;}
		}


		// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
		pthread_mutex_lock( & current_optErrorT_mutex );
		if(ub < current_optErrorT){
			current_optErrorT = ub;
			if(current_nodeTransOut){*current_nodeTransOut = nodeTrans;}
		}
		pthread_mutex_unlock( & current_optErrorT_mutex );

		// Remove subcube from queue if lb is bigger than optErrorT
		if(lb >= current_optErrorT){continue;}

		nodeTrans.ub = ub;
		nodeTrans.lb = lb;

		pthread_mutex_lock(		& queueTrans_mutex );
		if(!doneInnerBnB){queueTrans.push(nodeTrans);}
		pthread_mutex_unlock(	& queueTrans_mutex );
	}
	pthread_mutex_lock( & working_thread_count_mutex );
	working_thread_count--;
	pthread_mutex_unlock( &working_thread_count_mutex );
	return 1;
*/
/*
	pthread_mutex_lock( & queueTrans_mutex );
	if(queueTrans.empty()){
		printf("empty queue Line: %i\n",__LINE__);
		pthread_mutex_unlock( &queueTrans_mutex );
		return -1;
	}

	pthread_mutex_lock( & working_thread_count_mutex );
	working_thread_count++;
	pthread_mutex_unlock( &working_thread_count_mutex );

	nodeTransParent = queueTrans.top();
	queueTrans.pop();

	pthread_mutex_unlock( &queueTrans_mutex );

	if(current_optErrorT-nodeTransParent.lb < SSEThresh){
		doneInnerBnB = true;
printf("Line: %i\n",__LINE__);
		pthread_mutex_lock( & working_thread_count_mutex );
		working_thread_count--;
		pthread_mutex_unlock( &working_thread_count_mutex );
		return 0;
	}


	nodeTrans.w = nodeTransParent.w/2;
	maxTransDis = SQRT3/2.0*nodeTrans.w;
	for(j = 0; j < 8; j++){
		nodeTrans.x = nodeTransParent.x + (j&1)*nodeTrans.w ;
		nodeTrans.y = nodeTransParent.y + (j>>1&1)*nodeTrans.w ;
		nodeTrans.z = nodeTransParent.z + (j>>2&1)*nodeTrans.w ;

		transX = nodeTrans.x + nodeTrans.w/2;
		transY = nodeTrans.y + nodeTrans.w/2;
		transZ = nodeTrans.z + nodeTrans.w/2;

#ifdef slowMinDistLoop
		for(i = 0; i < Nd; i++){
			minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			if(maxRotDisL){		minDis[i] -= maxRotDisL[i];}
			if(minDis[i] < 0){	minDis[i] = 0;}
		}
#else
		if(maxRotDisL){
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ) - maxRotDisL[i];
				if(minDis[i] < 0){minDis[i] = 0;}
			}
		}else{
			for(i = 0; i < Nd; i++){
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
			}
		}
#endif

		// For each data point, find the incremental upper and lower bounds
		ub = 0;
		for(i = 0; i < inlierNum; i++){ub += minDis[i]*minDis[i];}

		lb = 0;
		for(i = 0; i < inlierNum; i++){
			dis = minDis[i] - maxTransDis;
			if(dis > 0){lb += dis*dis;}
		}


		// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
		if(ub < current_optErrorT){
			pthread_mutex_lock(		& current_optErrorT_mutex );
			if(ub < current_optErrorT){
				current_optErrorT = ub;
				if(current_nodeTransOut){*current_nodeTransOut = nodeTrans;}
			}
			pthread_mutex_unlock(	& current_optErrorT_mutex );
		}

		// Remove subcube from queue if lb is bigger than optErrorT
		if(lb >= current_optErrorT){continue;}

		nodeTrans.ub = ub;
		nodeTrans.lb = lb;

		pthread_mutex_lock(		& queueTrans_mutex );
		queueTrans.push(nodeTrans);
		pthread_mutex_unlock(	& queueTrans_mutex );
	}

	pthread_mutex_lock(		& working_thread_count_mutex );
	working_thread_count--;
	pthread_mutex_unlock(	&working_thread_count_mutex );

	return 1;
*/
}

GoICP::GoICP()
{
	initNodeRot.a = -PI;
	initNodeRot.b = -PI;
	initNodeRot.c = -PI;
	initNodeRot.w = 2*PI;
	initNodeRot.l = 0;

	initNodeRot.lb = 0;
	initNodeTrans.lb = 0;

	doTrim = true;

	doneInnerBnB = true;

	working_thread_count		= 0;
	working_thread_count_mutex	= PTHREAD_MUTEX_INITIALIZER;
	queueTrans_mutex			= PTHREAD_MUTEX_INITIALIZER;
	current_optErrorT_mutex		= PTHREAD_MUTEX_INITIALIZER;
/*
	int rc;
	int i;
	for( i=0; i < NUM_THREADS; i++ ){
		cout << "main() : creating thread, " << i << endl;
		rc = pthread_create(&threads[i], NULL, workerthread, (void *)(this));
		if (rc){
			cout << "Error:unable to create thread," << rc << endl;
			exit(-1);
		}
   }
*/
}

// Build Distance Transform
void GoICP::BuildDT()
{
	double* x = (double*)malloc(sizeof(double)*Nm);
	double* y = (double*)malloc(sizeof(double)*Nm);
	double* z = (double*)malloc(sizeof(double)*Nm);
	for(int i = 0; i < Nm; i++)
	{
		x[i] = pModel[i].x;
		y[i] = pModel[i].y;
		z[i] = pModel[i].z;
	}
	dt.Build(x, y, z, Nm);
	delete(x);
	delete(y);
	delete(z);
}

// Run ICP and calculate sum squared L2 error
float GoICP::ICP(Matrix& R_icp, Matrix& t_icp)
{
	max_time = 3;
  int i;
	float error, dis;

	icp3d.Run(D_icp, Nd, R_icp, t_icp); // data cloud, # data points, rotation matrix, translation matrix

	// Transform point cloud and use DT to determine the L2 error
	error = 0;
	for(i = 0; i < Nd; i++)
	{
		POINT3D& p = pData[i];
		pDataTempICP[i].x = R_icp.val[0][0]*p.x+R_icp.val[0][1]*p.y+R_icp.val[0][2]*p.z  + t_icp.val[0][0];
		pDataTempICP[i].y = R_icp.val[1][0]*p.x+R_icp.val[1][1]*p.y+R_icp.val[1][2]*p.z + t_icp.val[1][0];
		pDataTempICP[i].z = R_icp.val[2][0]*p.x+R_icp.val[2][1]*p.y+R_icp.val[2][2]*p.z + t_icp.val[2][0];

		if(!doTrim)
		{
			dis = dt.Distance(pDataTempICP[i].x, pDataTempICP[i].y, pDataTempICP[i].z);
			error += errorcost(dis);
		}
		else
		{
			minDis[i] = dt.Distance(pDataTempICP[i].x, pDataTempICP[i].y, pDataTempICP[i].z);
		}
	}

	if(doTrim)
	{
		//qsort(minDis, Nd, sizeof(float), cmp);
		//myqsort(minDis, Nd, inlierNum);
		intro_select(minDis,0,Nd-1,inlierNum-1);
		for(i = 0; i < inlierNum; i++)
		{
			float fmindis = minDis[i];
			error += errorcost(fmindis);//minDis[i]*minDis[i];
		}
	}

	return error;
}

void GoICP::InitializeBasic(){
	//initNodeRot.a = -PI;
	//initNodeRot.b = -PI;
	//initNodeRot.c = -PI;
	//initNodeRot.w = 2*PI;
	//initNodeRot.l = 0;

	initNodeRot.lb = 0;
	initNodeTrans.lb = 0;
	//doTrim = true;

	icp3d.trim_fraction = trimFraction;
	icp3d.do_trim = doTrim;

	// Initialise so-far-best rotation and translation nodes
	optNodeRot = initNodeRot;
	optNodeTrans = initNodeTrans;
	// Initialise so-far-best rotation and translation matrices
	optR = Matrix::eye(3);
	optT = Matrix::ones(3,1)*0;
	
	maxRotDis = new float*[MAXROTLEVEL];

	prove_optimal = true;
}

void GoICP::InitializeData(){
	int i, j;
	float sigma, maxAngle;

	// Precompute the rotation uncertainty distance (maxRotDis) for each point in the data and each level of rotation subcube
	// Calculate L2 norm of each point in data cloud to origin
	normData = (float*)malloc(sizeof(float)*Nd);
	for(i = 0; i < Nd; i++){normData[i] = sqrt(pData[i].x*pData[i].x + pData[i].y*pData[i].y + pData[i].z*pData[i].z);}
	
	for(i = 0; i < MAXROTLEVEL; i++){
		maxRotDis[i] = (float*)malloc(sizeof(float*)*Nd);
		sigma = initNodeRot.w/pow(2.0,i)/2.0; // Half-side length of each level of rotation subcube
		maxAngle = SQRT3*sigma;

		if(maxAngle > PI){maxAngle = PI;}
		for(j = 0; j < Nd; j++){maxRotDis[i][j] = 2*sin(maxAngle/2)*normData[j];}
	}
	
	// Temporary Variables
	minDis			= 	 (float*)malloc(sizeof(float)*Nd);
	pDataTemp		= (POINT3D *)malloc(sizeof(POINT3D)*Nd);
	pDataTempICP	= (POINT3D *)malloc(sizeof(POINT3D)*Nd);

	// ICP Initialisation
	// Copy data point clouds to variables for ICP
	D_icp = (float*)calloc(3*Nd,sizeof(float));
	for(i = 0, j = 0; i < Nd; i++){
		D_icp[j++] = pData[i].x;
		D_icp[j++] = pData[i].y;
		D_icp[j++] = pData[i].z;
	}
	
	// For untrimmed ICP, use all points, otherwise only use inlierNum points
	if(doTrim){	inlierNum = (int)(Nd * (1 - trimFraction));}
	else{		inlierNum = Nd;}
}

void GoICP::InitializeModel(){
	int i, j;
	float sigma, maxAngle;

	// ICP Initialisation
	// Copy model point clouds to variables for ICP
	M_icp = (float*)calloc(3*Nm,sizeof(float));
	for(i = 0, j = 0; i < Nm; i++){
		M_icp[j++] = pModel[i].x;
		M_icp[j++] = pModel[i].y;
		M_icp[j++] = pModel[i].z;
	}

	// Build ICP kdtree with model dataset
	icp3d.Build(M_icp,Nm);
	icp3d.err_diff_def = MSEThresh/10000;

	SSEThresh = MSEThresh * inlierNum;
}

void GoICP::Initialize()
{
	int i, j;
	float sigma, maxAngle;

	// Precompute the rotation uncertainty distance (maxRotDis) for each point in the data and each level of rotation subcube

	// Calculate L2 norm of each point in data cloud to origin
	normData = (float*)malloc(sizeof(float)*Nd);
	for(i = 0; i < Nd; i++)
	{
		normData[i] = sqrt(pData[i].x*pData[i].x + pData[i].y*pData[i].y + pData[i].z*pData[i].z);
	}

	maxRotDis = new float*[MAXROTLEVEL];
	for(i = 0; i < MAXROTLEVEL; i++)
	{
		maxRotDis[i] = (float*)malloc(sizeof(float*)*Nd);

		sigma = initNodeRot.w/pow(2.0,i)/2.0; // Half-side length of each level of rotation subcube
		maxAngle = SQRT3*sigma;

		if(maxAngle > PI)
			maxAngle = PI;
		for(j = 0; j < Nd; j++)
			maxRotDis[i][j] = 2*sin(maxAngle/2)*normData[j];
	}

	// Temporary Variables
	minDis = (float*)malloc(sizeof(float)*Nd);
	pDataTemp = (POINT3D *)malloc(sizeof(POINT3D)*Nd);
	pDataTempICP = (POINT3D *)malloc(sizeof(POINT3D)*Nd);

	// ICP Initialisation
	// Copy model and data point clouds to variables for ICP
	M_icp = (float*)calloc(3*Nm,sizeof(float));
	D_icp = (float*)calloc(3*Nd,sizeof(float));
	for(i = 0, j = 0; i < Nm; i++)
	{
		M_icp[j++] = pModel[i].x;
		M_icp[j++] = pModel[i].y;
		M_icp[j++] = pModel[i].z;
	}
	for(i = 0, j = 0; i < Nd; i++)
	{
		D_icp[j++] = pData[i].x;
		D_icp[j++] = pData[i].y;
		D_icp[j++] = pData[i].z;
	}

	// Build ICP kdtree with model dataset
	icp3d.Build(M_icp,Nm);
	icp3d.err_diff_def = MSEThresh/10000;
	icp3d.trim_fraction = trimFraction;
	icp3d.do_trim = doTrim;

	// Initialise so-far-best rotation and translation nodes
	optNodeRot = initNodeRot;
	optNodeTrans = initNodeTrans;
	// Initialise so-far-best rotation and translation matrices
	optR = Matrix::eye(3);
	optT = Matrix::ones(3,1)*0;

	// For untrimmed ICP, use all points, otherwise only use inlierNum points
	if(doTrim){	inlierNum = (int)(Nd * (1 - trimFraction));}
	else{		inlierNum = Nd;}
	SSEThresh = MSEThresh * inlierNum;
}

void GoICP::Clear()
{
	delete(pDataTemp);
	delete(pDataTempICP);
	delete(normData);
	delete(minDis);
	for(int i = 0; i < MAXROTLEVEL; i++)
	{
		delete(maxRotDis[i]);
	}
	delete(maxRotDis);
	delete(M_icp);
	delete(D_icp);
}

void GoICP::clearBasic(){
	delete(maxRotDis);
}
void GoICP::clearData(){
	delete(pDataTemp);
	delete(pDataTempICP);
	delete(normData);
	for(int i = 0; i < MAXROTLEVEL; i++)
	{
		delete(maxRotDis[i]);
	}
	delete(D_icp);
}	
void GoICP::clearModel(){
	delete(M_icp);
}

// Inner Branch-and-Bound, iterating over the translation space
float GoICP::InnerBnB(float* maxRotDisL, TRANSNODE* nodeTransOut)
{
	int i, j;
	float transX, transY, transZ;
	float lb, ub, optErrorT;
	float dis, maxTransDis;
	TRANSNODE nodeTrans, nodeTransParent;
	//priority_queue<TRANSNODE> queueTrans;

	// Set optimal translation error to overall so-far optimal error
	// Investigating translation nodes that are sub-optimal overall is redundant
	optErrorT = optError;

	// Push top-level translation node into the priority queue
	queueTrans.push(initNodeTrans);

	current_optErrorT		= optError;
	current_maxRotDisL		= maxRotDisL;
	current_nodeTransOut	= nodeTransOut;
	innercount = 0;
//#define runinnerbnb
#ifdef runinnerbnb


	doneInnerBnB = false;
	while( true ){
		//int result = runInnerBnBIteration(minDis);
/*
		if((result == -1) != (working_thread_count == 0 && (doneInnerBnB || queueTrans.empty()) )){
			printf("result: %i working_thread_count: %i queueTrans.empty(): %i\n",result,working_thread_count,queueTrans.empty());
			printf("WTF\n");
			exit(0);
		}
*/

		//printf("MAIN LOOP: working_thread_count: %i queueTrans.empty(): %i\n",working_thread_count,queueTrans.empty());
		if(working_thread_count == 0 && (doneInnerBnB || queueTrans.empty()) ){break;}
		usleep(100);
		//if(result == -1){break;}
		//printf("result: %i optErrorT: %f\n",result,current_optErrorT);
		//if(working_thread_count == 0 && (doneInnerBnB || queueTrans.empty()) ){break;}
	}

	doneInnerBnB = true;

	pthread_mutex_lock( & queueTrans_mutex );
	queueTrans = priority_queue<TRANSNODE>();
	pthread_mutex_unlock( & queueTrans_mutex );

	optErrorT = current_optErrorT;
	nodeTransOut = current_nodeTransOut;

#else
	//
	while(1)
	{
		if(queueTrans.empty())
			break;

		nodeTransParent = queueTrans.top();
		queueTrans.pop();

		innercount++;

		if(optErrorT-nodeTransParent.lb < SSEThresh)
		{
			queueTrans = priority_queue<TRANSNODE>();//.clear();
			break;
		}

		nodeTrans.w = nodeTransParent.w/2;
		maxTransDis = SQRT3/2.0*nodeTrans.w;

		for(j = 0; j < 8; j++)
		{
			nodeTrans.x = nodeTransParent.x + (j&1)*nodeTrans.w ;
			nodeTrans.y = nodeTransParent.y + (j>>1&1)*nodeTrans.w ;
			nodeTrans.z = nodeTransParent.z + (j>>2&1)*nodeTrans.w ;

			transX = nodeTrans.x + nodeTrans.w/2;
			transY = nodeTrans.y + nodeTrans.w/2;
			transZ = nodeTrans.z + nodeTrans.w/2;
#define slowMinDistLoop
#ifdef slowMinDistLoop
			// For each data point, calculate the distance to it's closest point in the model cloud
			for(i = 0; i < Nd; i++)
			{
				// Find distance between transformed point and closest point in model set ||R_r0 * x + t0 - y||
				// pDataTemp is the data points rotated by R0
				minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);

				// Subtract the rotation uncertainty radius if calculating the rotation lower bound
				// maxRotDisL == NULL when calculating the rotation upper bound
				if(maxRotDisL)
					minDis[i] -= maxRotDisL[i];

				if(minDis[i] < 0){
					minDis[i] = 0;
				}
			}
#else
			if(maxRotDisL){
				for(i = 0; i < Nd; i++){
					minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ) - maxRotDisL[i];
					if(minDis[i] < 0){minDis[i] = 0;}
				}
			}else{
				for(i = 0; i < Nd; i++){
					minDis[i] = dt.Distance(pDataTemp[i].x + transX, pDataTemp[i].y + transY, pDataTemp[i].z + transZ);
				}
			}
#endif

			// For each data point, find the incremental upper and lower bounds
			ub = 0;
			for(i = 0; i < inlierNum; i++){
				float fmindis = minDis[i];
				ub += errorcost(fmindis);//minDis[i]*minDis[i];
			}

			lb = 0;
			for(i = 0; i < inlierNum; i++)
			{
				// Subtract the translation uncertainty radius
				dis = minDis[i] - maxTransDis;
				if(dis > 0)
					lb += errorcost(dis);//dis*dis;
			}
			

			// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
			if(ub < optErrorT)
			{
				optErrorT = ub;
				if(nodeTransOut)
					*nodeTransOut = nodeTrans;
			}

			// Remove subcube from queue if lb is bigger than optErrorT
			if(lb >= optErrorT)
			{
				//discard
				continue;
			}

			nodeTrans.ub = ub;
			nodeTrans.lb = lb;
			queueTrans.push(nodeTrans);
		}
	}
#endif

	//printf("ic %5.5i ",innercount); printf("%10.10f ",optErrorT);
	//printf("\n");
	//exit(0);
	return optErrorT;
}

float GoICP::OuterBnB()
{

	clock_t  clockBegin = clock();



	//int i, j;
	ROTNODE nodeRotParent;

//	float v1, v2, v3, t, ct, ct2,st, st2;
//	float tmp121, tmp122, tmp131, tmp132, tmp231, tmp232;
//	float R11, R12, R13, R21, R22, R23, R31, R32, R33;
	float error;
	clock_t clockBeginICP;
	priority_queue<ROTNODE> queueRot;

	// Calculate Initial Error
	optError = 0;

	for(int i = 0; i < Nd; i++)
	{
		minDis[i] = dt.Distance(pData[i].x, pData[i].y, pData[i].z);
	}
	if(doTrim)
	{
		// Sort by distance
		//qsort(minDis, Nd, sizeof(float), cmp);
		//myqsort(minDis, Nd, inlierNum);
		intro_select(minDis,0,Nd-1,inlierNum-1);
	}
	for(int i = 0; i < inlierNum; i++)
	{
		float fmindisi = minDis[i];
		optError += errorcost(fmindisi);//*minDis[i];
	}
	//cout << "Error*: " << optError << " (Init)" << endl;

	Matrix R_icp = optR;
	Matrix t_icp = optT;

	// Run ICP from initial state
	clockBeginICP = clock();
	error = ICP(R_icp, t_icp);
	//printf("ICP return error: %f\n",error);

	if(error < optError)
	{
		optError = error;
		optR = R_icp;
		optT = t_icp;
		//cout << "Error*: " << error << " (ICP " << (double)(clock()-clockBeginICP)/CLOCKS_PER_SEC << "s)" << endl;
		//cout << "ICP-ONLY Rotation Matrix:" << endl;
		//cout << R_icp << endl;
		//cout << "ICP-ONLY Translation Vector:" << endl;
		//cout << t_icp << endl;
	}
//exit(0);
	// Push top-level rotation node into priority queue
	queueRot.push(initNodeRot);

	omp_lock_t writelock;
	omp_init_lock(&writelock);

	// Keep exploring rotation space until convergence is achieved
	long long count = 0;
	long long outercount = 0;
	while(prove_optimal){
		if(queueRot.empty()){
		  //cout << "Rotation Queue Empty" << endl;
		  //cout << "Error*: " << optError<< endl;// << ", LB: " << lb << endl;
		  break;
		}

		// Access rotation cube with lowest lower bound...
		nodeRotParent = queueRot.top();
		// ...and remove it from the queue
		queueRot.pop();

		// Exit if the optError is less than or equal to the lower bound plus a small epsilon
		if((optError-nodeRotParent.lb) <= SSEThresh){
			//cout << "Error*: " << optError << ", LB: " << nodeRotParent.lb << ", epsilon: " << SSEThresh << endl;
			break;
		}

		if(count>0 && count%300 == 0)
			printf("LB=%f  L=%d\n",nodeRotParent.lb,nodeRotParent.l);
		count ++;
		
		// Subdivide rotation cube into octant subcubes and calculate upper and lower bounds for each

		// For each subcube,
		for(int j = 0; j < 8; j++)
		{
			ROTNODE nodeRot;
			nodeRot.w = nodeRotParent.w/2;
			nodeRot.l = nodeRotParent.l+1;

			TRANSNODE nodeTrans;
			float v1, v2, v3, t, ct, ct2,st, st2;
			float tmp121, tmp122, tmp131, tmp132, tmp231, tmp232;
			float R11, R12, R13, R21, R22, R23, R31, R32, R33;
			float lb, ub;


		  // Calculate the smallest rotation across each dimension
			nodeRot.a = nodeRotParent.a + (j&1)*nodeRot.w ;
			nodeRot.b = nodeRotParent.b + (j>>1&1)*nodeRot.w ;
			nodeRot.c = nodeRotParent.c + (j>>2&1)*nodeRot.w ;

			// Find the subcube centre
			v1 = nodeRot.a + nodeRot.w/2;
			v2 = nodeRot.b + nodeRot.w/2;
			v3 = nodeRot.c + nodeRot.w/2;

			// Skip subcube if it is completely outside the rotation PI-ball
			if(sqrt(v1*v1+v2*v2+v3*v3)-SQRT3*nodeRot.w/2 > PI)
			{
				continue;
			}

			// Convert angle-axis rotation into a rotation matrix
			t = sqrt(v1*v1 + v2*v2 + v3*v3);
			if(t > 0)
			{
				v1 /= t;
				v2 /= t;
				v3 /= t;

				ct = cos(t);
				ct2 = 1 - ct;
				st = sin(t);
				st2 = 1 - st;

				tmp121 = v1*v2*ct2; tmp122 = v3*st;
				tmp131 = v1*v3*ct2; tmp132 = v2*st;
				tmp231 = v2*v3*ct2; tmp232 = v1*st;

				R11 = ct + v1*v1*ct2;		R12 = tmp121 - tmp122;		R13 = tmp131 + tmp132;
				R21 = tmp121 + tmp122;		R22 = ct + v2*v2*ct2;		R23 = tmp231 - tmp232;
				R31 = tmp131 - tmp132;		R32 = tmp231 + tmp232;		R33 = ct + v3*v3*ct2;

				// Rotate data points by subcube rotation matrix
				for(int i = 0; i < Nd; i++)
				{
					POINT3D& p = pData[i];
					pDataTemp[i].x = R11*p.x + R12*p.y + R13*p.z;
					pDataTemp[i].y = R21*p.x + R22*p.y + R23*p.z;
					pDataTemp[i].z = R31*p.x + R32*p.y + R33*p.z;
				}
			}
			// If t == 0, the rotation angle is 0 and no rotation is required
			else
			{
				memcpy(pDataTemp, pData, sizeof(POINT3D)*Nd);
			}

			// Upper Bound
			// Run Inner Branch-and-Bound to find rotation upper bound
			// Calculates the rotation upper bound by finding the translation upper bound for a given rotation,
			// assuming that the rotation is known (zero rotation uncertainty radius)

			//for(int iter = 0; iter < 100; iter++){
			//TRANSNODE inpnodeTrans = nodeTrans;
			//	ub = InnerBnB(NULL /*Rotation Uncertainty Radius*/, &inpnodeTrans);
			//}
			//exit(0);

			ub = InnerBnB(NULL /*Rotation Uncertainty Radius*/, &nodeTrans);

			// If the upper bound is the best so far, run ICP
			if(ub < optError)
			{
				// Update optimal error and rotation/translation nodes
				optError = ub;
				optNodeRot = nodeRot;
				optNodeTrans = nodeTrans;

				optR.val[0][0] = R11; optR.val[0][1] = R12; optR.val[0][2] = R13;
				optR.val[1][0] = R21; optR.val[1][1] = R22; optR.val[1][2] = R23;
				optR.val[2][0] = R31; optR.val[2][1] = R32; optR.val[2][2] = R33;
				optT.val[0][0] = optNodeTrans.x+optNodeTrans.w/2;
				optT.val[1][0] = optNodeTrans.y+optNodeTrans.w/2;
				optT.val[2][0] = optNodeTrans.z+optNodeTrans.w/2;

				//cout << "Error*: " << optError << endl;

				// Run ICP
				clockBeginICP = clock();
				R_icp = optR;
				t_icp = optT;
				error = ICP(R_icp, t_icp);
				//Our ICP implementation uses kdtree for closest distance computation which is slightly different from DT approximation, 
				//thus it's possible that ICP failed to decrease the DT error. This is no big deal as the difference should be very small.
				if(error < optError)
				{
					optError = error;
					optR = R_icp;
					optT = t_icp;
					
					//cout << "Error*: " << error << "(ICP " << (double)(clock() - clockBeginICP)/CLOCKS_PER_SEC << "s)" << endl;
				}

				// Discard all rotation nodes with high lower bounds in the queue
				priority_queue<ROTNODE> queueRotNew;
				while(!queueRot.empty())
				{
					ROTNODE node = queueRot.top();
					queueRot.pop();
					if(node.lb < optError)
						queueRotNew.push(node);
					else
						break;
				}
				queueRot = queueRotNew;
			}

			// Lower Bound
			// Run Inner Branch-and-Bound to find rotation lower bound
			// Calculates the rotation lower bound by finding the translation upper bound for a given rotation,
			// assuming that the rotation is uncertain (a positive rotation uncertainty radius)
			// Pass an array of rotation uncertainties for every point in data cloud at this level
			lb = InnerBnB(maxRotDis[nodeRot.l], NULL /*Translation Node*/);



			//printf("outercount: %i optError: %f ub: %f lb: %f\n",outercount,optError,ub,lb);
			outercount++;
			//if(outercount > 300){exit(0);}

			// If the best error so far is less than the lower bound, remove the rotation subcube from the queue
			if(lb >= optError)
			{
				continue;
			}


			// Update node and put it in queue
			nodeRot.ub = ub;
			nodeRot.lb = lb;
			queueRot.push(nodeRot);
		}
		//exit(0);

		clock_t  clockEnd = clock();
		double runtime = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
		if(runtime > max_time){
			printf("broke becouse of time...\n");
			omp_destroy_lock(&writelock);
			return -1;
		}
	}
	omp_destroy_lock(&writelock);
	return optError;
}

float GoICP::Register()
{
	//Initialize();
	InitializeBasic();
	InitializeData();	
	InitializeModel();
	OuterBnB();
	//Clear();
	clearData();
	clearModel();
	clearBasic();
	

	return optError;
}

};
