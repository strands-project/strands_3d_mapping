#include "RegistrationGOICP.h"

#include "ICP.h"
#include "GoICP_V1.3/src/jly_goicp.h"

#include <iostream>
#include <fstream>

namespace reglib
{

RegistrationGOICP::RegistrationGOICP(){
	only_initial_guess		= false;

	type					= PointToPlane;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;
	func					= new DistanceWeightFunction2PPR();

	visualizationLvl = 1;

	for(unsigned int i= 3; i < 6; i++){
		feature_start.push_back(i);
		feature_end.push_back(i);
		DistanceWeightFunction2PPR * f = new DistanceWeightFunction2PPR();
		f->regularization	= 0.0;
		f->maxd 			= 255;
		f->histogram_size	= 255;
		feature_func.push_back(f);
	}
}
RegistrationGOICP::~RegistrationGOICP(){}

namespace RigidMotionEstimator2 {
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	/// @param Right hand side
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
	Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
								   Eigen::MatrixBase<Derived2>& Xn,
								   Eigen::MatrixBase<Derived3>& Y,
								   Eigen::MatrixBase<Derived4>& N,
								   const Eigen::MatrixBase<Derived5>& w,
								   const Eigen::MatrixBase<Derived6>& u) {
		typedef Eigen::Matrix<double, 6, 6> Matrix66;
		typedef Eigen::Matrix<double, 6, 1> Vector6;
		typedef Eigen::Block<Matrix66, 3, 3> Block33;

		/// Normalize weight vector
		Eigen::VectorXd w_normalized = w/w.sum();
		/// De-mean
		Eigen::Vector3d X_mean;
		for(int i=0; i<3; ++i){X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();}

		Eigen::Vector3d Y_mean;
		for(int i=0; i<3; ++i){Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();}

		X.colwise() -= X_mean;
		Y.colwise() -= X_mean;
		/// Prepare LHS and RHS
		Matrix66 LHS = Matrix66::Zero();
		Vector6 RHS = Vector6::Zero();
		Block33 TL = LHS.topLeftCorner<3,3>();
		Block33 TR = LHS.topRightCorner<3,3>();
		Block33 BR = LHS.bottomRightCorner<3,3>();
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
		#pragma omp parallel
		{
			#pragma omp for
			for(int i=0; i<X.cols(); i++) {
				C.col(i) = X.col(i).cross(N.col(i));
			}
			#pragma omp sections nowait
			{
				#pragma omp section
				for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
				#pragma omp section
				for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
				#pragma omp section
				for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
				#pragma omp section
				for(int i=0; i<C.cols(); i++) {
					double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
					RHS.head<3>() += C.col(i)*dist_to_plane;
					RHS.tail<3>() += N.col(i)*dist_to_plane;
				}
			}
		}
		LHS = LHS.selfadjointView<Eigen::Upper>();
		/// Compute transformation
		Eigen::Affine3d transformation;
		Eigen::LDLT<Matrix66> ldlt(LHS);
		RHS = ldlt.solve(RHS);
		transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
						  Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
						  Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());

		Xn = transformation*Xn;

		transformation.translation() = RHS.tail<3>();
		/// Apply transformation
		X = transformation*X;
		/// Re-apply mean
		X.colwise() += X_mean;
		Y.colwise() += X_mean;

		//Eigen::Vector3d mean_diff;
		//for(int i=0; i<3; ++i){mean_diff(i) = ((X-Y).row(i).array()*w_normalized.transpose().array()).sum();}

		//std::cout << mean_diff << std::endl << std::endl;
		//std::cout << Y_mean << std::endl << std::endl;
		//std::cout << Y_mean-X_mean << std::endl << std::endl;
		//std::cout << transformation.translation() << std::endl << std::endl;
		/// Return transformation
		return transformation;
	}

	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
										  Eigen::MatrixBase<Derived2>& Xn,
										  Eigen::MatrixBase<Derived3>& Yp,
										  Eigen::MatrixBase<Derived4>& Yn,
										  const Eigen::MatrixBase<Derived5>& w) {
		return point_to_plane(X,Xn,Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
	}
}


FusionResults RegistrationGOICP::getTransform(Eigen::MatrixXd guess){

	unsigned int s_nr_data = std::min(int(src->data.cols()),int(500000));
	unsigned int d_nr_data = dst->data.cols();
	
	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Xn;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Y;

	X.resize(Eigen::NoChange,s_nr_data);
	Xn.resize(Eigen::NoChange,s_nr_data);
	Y.resize(Eigen::NoChange,d_nr_data);

	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);

	if(!only_initial_guess){
		float smeanx = 0;
		float smeany = 0;
		float smeanz = 0;
		float stot	 = 0;

		float threshold = 10;
		float threshold2 = threshold*threshold;
	for(int iter = 0; iter < 15; iter++){
		float tot	= 0;
		float sumx = 0;
		float sumy = 0;
		float sumz = 0;
		for(unsigned int i = 0; i < s_nr_data; i++){
			float x	= src->data(0,i);
			float y	= src->data(1,i);
			float z	= src->data(2,i);

			float newx	= m00*x + m01*y + m02*z + m03;
			float newy	= m10*x + m11*y + m12*z + m13;
			float newz	= m20*x + m21*y + m22*z + m23;
			if(iter == 0 || (pow(newx-smeanx,2)+pow(newy-smeany,2)+pow(newz-smeanz,2)) < threshold2){
				tot++;
				sumx += newx;
				sumy += newy;
				sumz += newz;
			}
		}
		smeanx = sumx/tot;
		smeany = sumy/tot;
		smeanz = sumz/tot;
		stot = tot;
		//printf("iter %i tot: %f\n",iter,tot);
	}

	float dmeanx = 0;
	float dmeany = 0;
	float dmeanz = 0;
	float dtot	 = 0;
	for(int iter = 0; iter < 15; iter++){
		float tot	= 0;
		float sumx = 0;
		float sumy = 0;
		float sumz = 0;
		for(unsigned int i = 0; i < d_nr_data; i++){
			float newx	= dst->data(0,i);
			float newy	= dst->data(1,i);
			float newz	= dst->data(2,i);

			if(iter == 0 || (pow(newx-dmeanx,2)+pow(newy-dmeany,2)+pow(newz-dmeanz,2)) < threshold2){
				tot++;
				sumx += newx;
				sumy += newy;
				sumz += newz;
			}
		}
		dmeanx = sumx/tot;
		dmeany = sumy/tot;
		dmeanz = sumz/tot;
		dtot = tot;
		//printf("iter %i tot: %f\n",iter,tot);
	}
	/*
		std::ofstream xfile;
		xfile.open ("X.txt");
		xfile << int(stot) << "\n";


		//unsigned int s_nr_data = src->data.cols();

		for(unsigned int i = 0; i < s_nr_data; i++){
			float x	= src->data(0,i);
			float y	= src->data(1,i);
			float z	= src->data(2,i);
			X(0,i)	= m00*x + m01*y + m02*z + m03;
			X(1,i)	= m10*x + m11*y + m12*z + m13;
			X(2,i)	= m20*x + m21*y + m22*z + m23;

			if((pow(X(0,i)-smeanx,2)+pow(X(1,i)-smeany,2)+pow(X(2,i)-smeanz,2)) >= threshold2){
				X(0,i) = 0;
				X(1,i) = 0;
				X(2,i) = 0;
			}else{
				xfile <<  (X(0,i)-smeanx)/threshold << " " << (X(1,i)-smeany)/threshold << " " <<  (X(2,i)-smeanz)/threshold << "\n";
			}
		}
		xfile.close();

		std::ofstream yfile;
		yfile.open ("Y.txt");
		yfile << int(dtot) << "\n";

		//unsigned int d_nr_data = dst->data.cols();

		for(unsigned int i = 0; i < d_nr_data; i++){
			Y(0,i)	= dst->data(0,i);
			Y(1,i)	= dst->data(1,i);
			Y(2,i)	= dst->data(2,i);
			if((pow(Y(0,i)-dmeanx,2)+pow(Y(1,i)-dmeany,2)+pow(Y(2,i)-dmeanz,2)) >= threshold2){
				Y(0,i) = 0;
				Y(1,i) = 0;
				Y(2,i) = 0;
			}else{
				yfile <<  (Y(0,i)-dmeanx)/threshold << " " << (Y(1,i)-dmeany)/threshold << " " <<  (Y(2,i)-dmeanz)/threshold << "\n";
			}
		}
		yfile.close();

		printf("s_nr_data: %i d_nr_data: %i \n",s_nr_data,d_nr_data);

		if(visualizationLvl >= 1){show(X,Y);}
	*/

		int Nm = dst->data.cols();
		int Nd = src->data.cols();

		goicp::POINT3D * pModel = new goicp::POINT3D[Nm];
		goicp::POINT3D * pData	= new goicp::POINT3D[Nd];

		int countModel = 0;
		for(unsigned int i = 0; i < d_nr_data; i++){
			float x	= dst->data(0,i);
			float y	= dst->data(1,i);
			float z	= dst->data(2,i);
			if((pow(x-dmeanx,2)+pow(y-dmeany,2)+pow(z-dmeanz,2)) >= threshold2){

			}else{
				pModel[countModel].x =	( x - dmeanx);
				pModel[countModel].y =	( y - dmeany);
				pModel[countModel].z =	( z - dmeanz);
				countModel++;
			}
		}
		Nm = countModel;

		int countData = 0;
		for(unsigned int i = 0; i < s_nr_data; i++){
			float x	= src->data(0,i);
			float y	= src->data(1,i);
			float z	= src->data(2,i);
			if((pow(x-smeanx,2)+pow(y-smeany,2)+pow(z-smeanz,2)) >= threshold2){

			}else{
				pData[countData].x =	( x - smeanx);
				pData[countData].y =	( y - smeany);
				pData[countData].z =	( z - smeanz);
				countData++;
			}
		}
		Nd = countData;

		for(unsigned int i = 0; i < Nd; i++){
			goicp::POINT3D tmp = pData[i];
			int rnd = rand()%Nd;
			pData[i] = pData[rnd];
			pData[rnd] = tmp;
		}

		goicp::GoICP goicp;

		goicp.MSEThresh = 0.1;
		goicp.initNodeRot.a = -3.1416;
		goicp.initNodeRot.b = -3.1416;
		goicp.initNodeRot.c = -3.1416;
		goicp.initNodeRot.w = 6.2832;
		goicp.initNodeTrans.x = -0.5*threshold;
		goicp.initNodeTrans.y = -0.5*threshold;
		goicp.initNodeTrans.z = -0.5*threshold;
		goicp.initNodeTrans.w = 1*threshold;

		goicp.trimFraction = 0;
		if(goicp.trimFraction < 0.001){goicp.doTrim = false;}

		goicp.dt.SIZE = 100;
		goicp.dt.expandFactor = 2;


		goicp.pModel = pModel;
		goicp.Nm = Nm;

		goicp.pData = pData;
		goicp.Nd = Nd;

		// Build Distance Transform
		std::cout << "Building Distance Transform..." << std::flush;
		clock_t clockBegin = clock();
		goicp.BuildDT();
		clock_t clockEnd = clock();
		std::cout << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << "s (CPU)" << std::endl;

		goicp.InitializeBasic();

		goicp.InitializeBasic();
		//goicp.InitializeModel();
		//for(int i = 0; i < 4; i++){

		Eigen::Matrix4d a = Eigen::Matrix4d::Identity();
		a(0,3) = -smeanx; a(1,3) = -smeany; a(2,3) = -smeanz;

		Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
		b(0,3) = dmeanx; b(1,3) = dmeany; b(2,3) = dmeanz;

		clock_t clockBegin2 = clock();
		std::cout << "Looking for intitial guess " << std::flush;
		for(int i = 0; i < 1; i++){
			int pts = 75 << i;
			pts = std::min(pts,Nd);
			//printf("nr points to use = %i\n",pts);
			goicp.Nd = pts;
			if(pts > Nd){goicp.Nd = Nd;} // Only use first NdDownsampled data points (assumes data points are randomly ordered)

			goicp.optError = 99999999999999999999999999;

			if(i == 0){
				for(float current_MSEThresh = 1.0; current_MSEThresh >= 0.01; current_MSEThresh *= 0.1){
					goicp.prove_optimal = true;
					goicp.MSEThresh = current_MSEThresh;//0.001;
					goicp.InitializeData();
					goicp.InitializeModel();
					float rv = goicp.OuterBnB();
					goicp.clearData();
					goicp.clearModel();

					if(rv < 0){
						clock_t clockEnd2 = clock();
						std::cout << (double)(clockEnd2 - clockBegin2)/CLOCKS_PER_SEC << "s (CPU)" << std::endl;

						goicp.clearBasic();
						delete[] pModel;
						delete[] pData;
						return FusionResults();
					}
				}
			}else{
				goicp.prove_optimal = true;
				goicp.MSEThresh = 0.01;
				goicp.InitializeData();
				goicp.InitializeModel();
				goicp.OuterBnB();
				goicp.clearData();
				goicp.clearModel();
			}
			//cout << "inner Optimal Rotation Matrix:" << endl;
			//cout << goicp.optR << endl;
			//cout << "inner Optimal Translation Vector:" << endl;
			//cout << goicp.optT << endl;
		}
		goicp.clearBasic();

		clock_t clockEnd2 = clock();
		std::cout << (double)(clockEnd2 - clockBegin2)/CLOCKS_PER_SEC << "s (CPU)" << std::endl;

		clock_t clockBegin3 = clock();
		//std::cout << "Refining guess " << std::flush;

		delete[] pModel;
		delete[] pData;

		guess(0,0) = goicp.optR.val[0][0];	guess(0,1) = goicp.optR.val[0][1]; guess(0,2) = goicp.optR.val[0][2];
		guess(1,0) = goicp.optR.val[1][0];	guess(1,1) = goicp.optR.val[1][1]; guess(1,2) = goicp.optR.val[1][2];
		guess(2,0) = goicp.optR.val[2][0];	guess(2,1) = goicp.optR.val[2][1]; guess(2,2) = goicp.optR.val[2][2];
		guess(0,3) = goicp.optT.val[0][0];	guess(1,3) = goicp.optT.val[1][0]; guess(2,3) = goicp.optT.val[2][0];

		guess = b*guess*a;
	}

	Eigen::Matrix<double, 3, Eigen::Dynamic> N;
	N.resize(Eigen::NoChange,d_nr_data);
	for(unsigned int i = 0; i < d_nr_data; i++){
		Y(0,i)	= dst->data(0,i);
		Y(1,i)	= dst->data(1,i);
		Y(2,i)	= dst->data(2,i);
		N(0,i)	= dst->normals(0,i);
		N(1,i)	= dst->normals(1,i);
		N(2,i)	= dst->normals(2,i);
	}

	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(X.cols());
	for(unsigned int i = 0; i < s_nr_data; i++){SRC_INORMATION(i) = src->information(0,i);}

	Eigen::VectorXd DST_INORMATION = Eigen::VectorXd::Zero(Y.cols());
	for(unsigned int i = 0; i < d_nr_data; i++){DST_INORMATION(i) = dst->information(0,i);}

	double stop		= 0.00001;

	/// Build kd-tree
	nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>                  kdtree(Y);

	/// Buffers
	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	X.cols());

//double best_score = 0;
//Eigen::Matrix4d best_transform = Eigen::Matrix4d::Identity();


//while(true)
//{
	Eigen::Matrix4d tmptest= guess;
	Eigen::Affine3d test = Eigen::Affine3d(tmptest);
	m00 = guess(0,0); m01 = guess(0,1); m02 = guess(0,2); m03 = guess(0,3);
	m10 = guess(1,0); m11 = guess(1,1); m12 = guess(1,2); m13 = guess(1,3);
	m20 = guess(2,0); m21 = guess(2,1); m22 = guess(2,2); m23 = guess(2,3);

	for(unsigned int i = 0; i < s_nr_data; i++){
		float x		= src->data(0,i);
		float y		= src->data(1,i);
		float z		= src->data(2,i);
		float xn	= src->normals(0,i);
		float yn	= src->normals(1,i);
		float zn	= src->normals(2,i);
		X(0,i)	= m00*x + m01*y + m02*z + m03;
		X(1,i)	= m10*x + m11*y + m12*z + m13;
		X(2,i)	= m20*x + m21*y + m22*z + m23;
		Xn(0,i)	= m00*xn + m01*yn + m02*zn;
		Xn(1,i)	= m10*xn + m11*yn + m12*zn;
		Xn(2,i)	= m20*xn + m21*yn + m22*zn;
	}

	Eigen::Matrix4d np;
	pcl::TransformationFromCorrespondences tfc;
/*
	printf("----%i----\n",__LINE__);
	tfc.reset();
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	np = tfc.getTransformation().matrix().cast<double>();
*/
	//std::cout << np << std::endl << std::endl;
	//std::cout << test.matrix() << std::endl;
	//printf("--------\n");
	//cout << guess << endl;

	func->reset();
	func->regularization = 0.1;

	for(unsigned int f = 0; f < feature_func.size(); f++){
		feature_func[f]->regularization = 15;
	}


	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;
	Eigen::Matrix3Xd Xo4 = X;
	Eigen::MatrixXd residuals;
	Eigen::VectorXd angles;

	std::vector<int> matchid;
	matchid.resize(s_nr_data);

	std::vector<double> total_dweight;
	total_dweight.resize(d_nr_data);
	if(visualizationLvl >= 2){show(X,Xn,Y,N);}

	/// ICP
	//for(int icp=0; icp < 50; ++icp) {
	//for(int icp=0; icp < 50; ++icp) {
	//while(true){
	for(int funcupdate=0; funcupdate < 100; ++funcupdate) {
		for(int rematching=0; rematching < 2; ++rematching) {
			//printf("funcupdate: %i rematching: %i\n",funcupdate,rematching);

			#pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {matchid[i] = kdtree.closest(X.col(i).data());}

			/// Find closest point
			#pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {
				int id = matchid[i];
				Qn.col(i) = N.col(id);
				Qp.col(i) = Y.col(id);
				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			}



			for(int outer=0; outer< 3; ++outer) {
				/// Compute weights
				switch(type) {
					case PointToPoint:	{residuals = X-Qp;} 						break;
					case PointToPlane:	{residuals = Qn.array()*(X-Qp).array();}	break;
					default:			{printf("type not set\n");}					break;
				}
				for(unsigned int i=0; i<X.cols(); ++i) {residuals.col(i) *= rangeW(i);}
				switch(type) {
					case PointToPoint:	{func->computeModel(residuals); 				} 	break;
					case PointToPlane:	{func->computeModel(residuals.colwise().norm());}	break;
					default:  			{printf("type not set\n");} break;
				}


				for(int inner=0; inner< 2; ++inner) {

					//printf("funcupdate: %i rematching: %i  outer: %i inner: %i\n",funcupdate,rematching,outer,inner);
					//printf("icp: %i outer: %i inner: %i ",icp,outer,inner);
					if(inner != 0){
						switch(type) {
							case PointToPoint:	{residuals = X-Qp;} 						break;
							case PointToPlane:	{residuals	= Qn.array()*(X-Qp).array();}	break;
							default:			{printf("type not set\n");}					break;
						}
						for(int i=0; i<X.cols(); ++i) {residuals.col(i) *= rangeW(i);}
					}

					switch(type) {
						case PointToPoint:	{W = func->getProbs(residuals); } 					break;
						case PointToPlane:	{
							W = func->getProbs(residuals.colwise().norm());
							for(int i=0; i<X.cols(); ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
						}	break;
						default:			{printf("type not set\n");} break;
					}

					Wold = W;

					float score1 = Wold.sum()/(pow(func->getNoise(),2)*float(s_nr_data));
					//printf("sum: %f noise: %f score: %f\n",Wold.sum(),func->getNoise(),score1);

					//Normalizing weights has an effect simmilar to one to one matching
					//in that it reduces the effect of border points
					if(normalize_matchweights){
						for(unsigned int i=0; i < d_nr_data; ++i) {	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
						for(unsigned int i=0; i< s_nr_data; ++i) {	total_dweight[matchid[i]] += W(i);}
						for(unsigned int i=0; i< s_nr_data; ++i) {	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
					}
					//W = fwt.array()*W.array();
					//if(visualizationLvl >= 3){show(X,Qp,W);}
					W = W.array()*rangeW.array()*rangeW.array();

					Eigen::Affine3d change;
					switch(type) {
						case PointToPoint:	{RigidMotionEstimator::point_to_point(X, Qp, W);}		break;
						case PointToPlane:	{change = RigidMotionEstimator2::point_to_plane(X, Xn, Qp, Qn, W);}	break;
						default:  			{printf("type not set\n"); } break;
					}

					double stop1 = (X-Xo1).colwise().norm().maxCoeff();
					//printf("stop: %f stop1: %f\n",stop,stop1);
					Xo1 = X;
					if(stop1 < stop) break;
				}

				double stop2 = (X-Xo2).colwise().norm().maxCoeff();
				//printf("stop: %f stop2: %f\n",stop,stop2);
				Xo2 = X;
				if(stop2 < stop) break;
			}

			double stop3 = (X-Xo3).colwise().norm().maxCoeff();
			//printf("stop: %f stop3: %f\n",stop,stop3);
			Xo3 = X;
			if(stop3 < stop) break;
		}

		//if(visualizationLvl >= 2){show(X,Xn,Y,N);}
/*
		func->update();

		double stop4 = (X-Xo4).colwise().norm().maxCoeff();
		Xo4 = X;
		if(stop4 < stop && funcupdate > 25) break;
		*/

		func->debugg_print = true;
		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		func->debugg_print = false;
		//printf("before: %5.5f after: %5.5f relative size: %5.5f\n",noise_before,noise_after,noise_after/noise_before);
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}
if(visualizationLvl >= 2){show(X,Xn,Y,N);}
	float score = Wold.sum()*pow(func->getNoise(),-1)/float(s_nr_data);
	//printf("sum: %f noise: %f score: %f\n",Wold.sum(),func->getNoise(),score);


	//printf("----%i----\n",__LINE__);
	tfc.reset();
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	np = tfc.getTransformation().matrix().cast<double>();

	//std::cout << np << std::endl << std::endl;
	//std::cout << test.matrix() << std::endl;
	//printf("--------\n");
	//exit(0);
/*
	guess = np;

	if(score > best_score){
		best_score = score;
		best_transform = np;
		//if(score > 400){show(X,Xn,Y,N);}
	}
*/
	//show(X,Xn,Y,N);
	//return FusionResults(best_transform,best_score);

	//clock_t clockEnd3 = clock();
	//std::cout << (double)(clockEnd3 - clockBegin3)/CLOCKS_PER_SEC << "s (CPU)" << std::endl;

	return FusionResults(np,score);
}

}
