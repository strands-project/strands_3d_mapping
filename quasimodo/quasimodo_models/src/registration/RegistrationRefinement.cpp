#include "registration/RegistrationRefinement.h"
#include <iostream>
#include <fstream>

//#include "registration/myhull.h"

//#include <pcl/surface/convex_hull.h>

namespace reglib
{

RegistrationRefinement::RegistrationRefinement(){
	//func = 0;
	nr_arraypoints = 0;
	arraypoints = 0;
	trees3d = 0;
	a3d = 0;
	only_initial_guess		= false;

	type					= PointToPlane;
	//type					= PointToPoint;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;

	visualizationLvl = 1;

    target_points = 250;
    allow_regularization = true;
    maxtime = 9999999;

	func = new DistanceWeightFunction2PPR2();
	func->startreg			= 0.1;
	func->debugg_print		= false;
}
RegistrationRefinement::~RegistrationRefinement(){
	if(func != 0){delete func; func = 0;}
	if(arraypoints != 0){delete arraypoints; arraypoints = 0;}
	if(trees3d != 0){delete trees3d; trees3d = 0;}
	if(a3d != 0){delete a3d; a3d = 0;}
}

void RegistrationRefinement::setDst(CloudData * dst_){
	dst = dst_;
	unsigned int d_nr_data = dst->data.cols();
	int stepy = std::max(1,int(d_nr_data)/100000);
	Y.resize(Eigen::NoChange,d_nr_data/stepy);
	N.resize(Eigen::NoChange,d_nr_data/stepy);
	ycols = Y.cols();
	total_dweight.resize(ycols);

	int count = 0;
	for(unsigned int i = 0; i < d_nr_data/stepy; i++){
		Y(0,i)	= dst->data(0,i*stepy);
		Y(1,i)	= dst->data(1,i*stepy);
		Y(2,i)	= dst->data(2,i*stepy);
		N(0,i)	= dst->normals(0,i*stepy);
		N(1,i)	= dst->normals(1,i*stepy);
		N(2,i)	= dst->normals(2,i*stepy);
		count++;
	}

	if(arraypoints != 0){delete arraypoints;}
	if(trees3d != 0){delete trees3d;}
	if(a3d != 0){delete a3d;}

	arraypoints = new double[3*count];
	nr_arraypoints = count;
	for(unsigned int c = 0; c < count; c++){
		arraypoints[3*c+0] = Y(0,c);
		arraypoints[3*c+1] = Y(1,c);
		arraypoints[3*c+2] = Y(2,c);
	}

	a3d = new ArrayData3D<double>;
	a3d->data	= arraypoints;
	a3d->rows	= count;
	trees3d	= new Tree3d(3, *a3d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	trees3d->buildIndex();

	DST_INORMATION = Eigen::VectorXd::Zero(ycols);
	for(unsigned int i = 0; i < d_nr_data/stepy; i++){DST_INORMATION(i) = dst->information(0,i*stepy);}
}

FusionResults RegistrationRefinement::getTransform(Eigen::MatrixXd guess){

//	DistanceWeightFunction2PPR2 * func = new DistanceWeightFunction2PPR2();
//	func->startreg			= 0.1;
//	func->debugg_print		= false;


	unsigned int s_nr_data = src->data.cols();
    int stepx = std::max(1,int(s_nr_data)/target_points);

	double stop		= 0.00001;

	func->reset();

	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);

	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Xn;
	X.resize(Eigen::NoChange,s_nr_data/stepx);
	Xn.resize(Eigen::NoChange,s_nr_data/stepx);
	unsigned int xcols = X.cols();


	/// Buffers
	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	xcols);
	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	xcols);
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	xcols);
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	xcols);
	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	xcols);

	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(xcols);
	for(unsigned int i = 0; i < s_nr_data/stepx; i++){
		SRC_INORMATION(i) = src->information(0,i*stepx);
		float x		= src->data(0,i*stepx);
		float y		= src->data(1,i*stepx);
		float z		= src->data(2,i*stepx);
		float xn	= src->normals(0,i*stepx);
		float yn	= src->normals(1,i*stepx);
		float zn	= src->normals(2,i*stepx);
		X(0,i)	= m00*x + m01*y + m02*z + m03;
		X(1,i)	= m10*x + m11*y + m12*z + m13;
		X(2,i)	= m20*x + m21*y + m22*z + m23;
		Xn(0,i)	= m00*xn + m01*yn + m02*zn;
		Xn(1,i)	= m10*xn + m11*yn + m12*zn;
		Xn(2,i)	= m20*xn + m21*yn + m22*zn;
	}

	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;
	Eigen::Matrix3Xd Xo4 = X;
	Eigen::MatrixXd residuals;

	std::vector<int> matchid;
	matchid.resize(xcols);
	double score = 0;
	stop = 99999;

	double start = getTime();

bool timestopped = false;
	/// ICP
	for(int funcupdate=0; funcupdate < 100; ++funcupdate) {
		if( (getTime()-start) > maxtime ){timestopped = true; break;}
		for(int rematching=0; rematching < 100; ++rematching) {
			if( (getTime()-start) > maxtime ){timestopped = true; break;}

#pragma omp parallel for
			for(unsigned int i=0; i< xcols; ++i) {
				std::vector<size_t>   ret_indexes(1);
				std::vector<double> out_dists_sqr(1);
				nanoflann::KNNResultSet<double> resultSet(1);

				double * qp = X.col(i).data();
				resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
				trees3d->findNeighbors(resultSet, qp, nanoflann::SearchParams(10));
				matchid[i] = ret_indexes[0];
			}

			/// Find closest point
#pragma omp parallel for
			for(unsigned int i=0; i< xcols; ++i) {
				int id = matchid[i];
				Qn.col(i) = N.col(id);
				Qp.col(i) = Y.col(id);
				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			}

			for(int outer=0; outer< 1; ++outer) {
				if( (getTime()-start) > maxtime ){timestopped = true; break;}
				/// Compute weights
				switch(type) {
					case PointToPoint:	{residuals = X-Qp;} 						break;
					case PointToPlane:	{
						residuals		= Eigen::MatrixXd::Zero(1,	xcols);
                        for(unsigned int i=0; i<xcols; ++i) {
							float dx = X(0,i)-Qp(0,i);
							float dy = X(1,i)-Qp(1,i);
							float dz = X(2,i)-Qp(2,i);
							float qx = Qn(0,i);
							float qy = Qn(1,i);
							float qz = Qn(2,i);
							float di = qx*dx + qy*dy + qz*dz;
							residuals(0,i) = di;
						}
					}break;
					default:			{printf("type not set\n");}					break;
				}
                for(unsigned int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}
				switch(type) {
					case PointToPoint:	{func->computeModel(residuals);} 	break;
					case PointToPlane:	{func->computeModel(residuals);}	break;
					default:  			{printf("type not set\n");} break;
				}

				for(int rematching2=0; rematching2 < 3; ++rematching2) {
					if( (getTime()-start) > maxtime ){timestopped = true; break;}
					if(rematching2 != 0){
#pragma omp parallel for
						for(unsigned int i=0; i< xcols; ++i) {
							std::vector<size_t>   ret_indexes(1);
							std::vector<double> out_dists_sqr(1);
							nanoflann::KNNResultSet<double> resultSet(1);

							double * qp = X.col(i).data();
							resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
							trees3d->findNeighbors(resultSet, qp, nanoflann::SearchParams(10));
							matchid[i] = ret_indexes[0];
						}

						/// Find closest point
#pragma omp parallel for
						for(unsigned int i=0; i< xcols; ++i) {
							int id = matchid[i];
							Qn.col(i) = N.col(id);
							Qp.col(i) = Y.col(id);
							rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
						}
					}

					for(int inner=0; inner< 40; ++inner) {
						if( (getTime()-start) > maxtime ){timestopped = true; break;}
						if(inner != 0){
							switch(type) {
								case PointToPoint:	{residuals = X-Qp;} 						break;
								case PointToPlane:	{
									residuals		= Eigen::MatrixXd::Zero(1,	xcols);
                                    for(unsigned int i=0; i< xcols; ++i) {
										float dx = X(0,i)-Qp(0,i);
										float dy = X(1,i)-Qp(1,i);
										float dz = X(2,i)-Qp(2,i);
										float qx = Qn(0,i);
										float qy = Qn(1,i);
										float qz = Qn(2,i);
										float di = qx*dx + qy*dy + qz*dz;
										residuals(0,i) = di;
									}
								}break;
								default:			{printf("type not set\n");}					break;
							}
                            for(unsigned int i=0; i<xcols; ++i) {residuals.col(i) *= rangeW(i);}
						}

						switch(type) {
							case PointToPoint:	{W = func->getProbs(residuals); } 					break;
							case PointToPlane:	{
								W = func->getProbs(residuals);
                                for(unsigned int i=0; i<xcols; ++i) {W(i) = W(i)*float((Xn(0,i)*Qn(0,i) + Xn(1,i)*Qn(1,i) + Xn(2,i)*Qn(2,i)) > 0.0);}
							}	break;
							default:			{printf("type not set\n");} break;
						}
						Wold = W;

						//Normalizing weights has an effect simmilar to one to one matching
						//in that it reduces the effect of border points
						if(normalize_matchweights){
//							for(unsigned int i=0; i < ycols; ++i)	{	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
//							for(unsigned int i=0; i< xcols; ++i)	{	total_dweight[matchid[i]] += W(i);}
//							for(unsigned int i=0; i< xcols; ++i)	{	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}

							for(unsigned int i=0; i < ycols; ++i)	{	total_dweight[i] = 99990.0000001;}//Reset to small number to avoid division by zero
							for(unsigned int i=0; i< xcols; ++i)	{	total_dweight[matchid[i]] = std::min(total_dweight[matchid[i]],residuals.col(i).norm());}
							for(unsigned int i=0; i< xcols; ++i)	{
								W(i) = (total_dweight[matchid[i]] == residuals.col(i).norm());//W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);
							}
						}

						W = W.array()*rangeW.array()*rangeW.array();

						switch(type) {
							case PointToPoint:	{
								pcl::TransformationFromCorrespondences tfc1;
								for(unsigned int c = 0; c < X.cols(); c++){tfc1.add(Eigen::Vector3f(X(0,c), X(1,c),X(2,c)),Eigen::Vector3f(Qp(0,c),Qp(1,c),Qp(2,c)),W(c));}
								Eigen::Affine3d rot = tfc1.getTransformation().cast<double>();
								X = rot*X;
								Xn = rot.rotation()*Xn;
							}break;
							case PointToPlane:	{point_to_plane2(X, Xn, Qp, Qn, W);}break;
							default:  			{printf("type not set\n"); } break;
						}

						stop = 100*func->getConvergenceThreshold();
						score = Wold.sum()/(pow(func->getNoise(),2)*float(xcols));

						double stop1 = (X-Xo1).colwise().norm().mean();
						Xo1 = X;
						if(stop1 < stop) break;
					}
					double stop2 = (X-Xo2).colwise().norm().mean();
					Xo2 = X;
					if(stop2 < stop) break;
				}
				double stop3 = (X-Xo3).colwise().norm().mean();
				Xo3 = X;
				if(stop3 < stop) break;
			}
			double stop4 = (X-Xo4).colwise().norm().mean();
			Xo4 = X;
			if(stop4 < stop) break;
		}
		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}

    if(visualizationLvl >= 2){show(X,Y);}

	pcl::TransformationFromCorrespondences tfc;
	tfc.reset();
	for(unsigned int i = 0; i < xcols; i++){
		tfc.add(Eigen::Vector3f(src->data(0,i*stepx),	src->data(1,i*stepx),	src->data(2,i*stepx)),Eigen::Vector3f (X(0,i),X(1,i),	X(2,i)));
	}
	guess = tfc.getTransformation().matrix().cast<double>();

	//delete func;
	//func = 0;

    FusionResults fr = FusionResults(guess,stop);
	fr.timeout = timestopped;
	return fr;

}

}
