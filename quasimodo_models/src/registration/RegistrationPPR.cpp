#include "RegistrationPPR.h"

#include "ICP.h"

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

namespace reglib
{

RegistrationPPR::RegistrationPPR(){
	type					= PointToPlane;
	use_PPR_weight			= true;
	use_features			= true;
	//normalize_matchweights	= true;
	func					= new DistanceWeightFunction2PPR();

	visualizationLvl = 2;

	for(unsigned int i= 3; i < 6; i++){
		feature_start.push_back(i);
		feature_end.push_back(i);
		DistanceWeightFunction2PPR * f = new DistanceWeightFunction2PPR();
		f->regularization	= 0.0;
		f->maxd 			= 255;
		f->histogram_size	= 255;
		//f.blurval			= 3;
		feature_func.push_back(f);
	}
}
RegistrationPPR::~RegistrationPPR(){}


FusionResults RegistrationPPR::getTransform(Eigen::MatrixXd guess){

	unsigned int s_nr_data = src->data.cols();
	unsigned int d_nr_data = dst->data.cols();
	
	cout << guess << endl;

	func->reset();
	func->regularization = 0.3;

    for(unsigned int f = 0; f < feature_func.size(); f++){
        feature_func[f]->regularization = 15;
    }

	Eigen::Matrix<double, 3, Eigen::Dynamic> X;
	std::vector<Eigen::MatrixXd> Xf;
	Eigen::Matrix<double, 3, Eigen::Dynamic> Y;

	std::vector<Eigen::MatrixXd> Yf;
	Eigen::Matrix<double, 3, Eigen::Dynamic> N;

	float m00 = guess(0,0); float m01 = guess(0,1); float m02 = guess(0,2); float m03 = guess(0,3);
	float m10 = guess(1,0); float m11 = guess(1,1); float m12 = guess(1,2); float m13 = guess(1,3);
	float m20 = guess(2,0); float m21 = guess(2,1); float m22 = guess(2,2); float m23 = guess(2,3);


	X.resize(Eigen::NoChange,s_nr_data);
	for(unsigned int i = 0; i < s_nr_data; i++){
		float x	= src->data(0,i);
		float y	= src->data(1,i);
		float z	= src->data(2,i);
		X(0,i)	= m00*x + m01*y + m02*z + m03;
		X(1,i)	= m10*x + m11*y + m12*z + m13;
		X(2,i)	= m20*x + m21*y + m22*z + m23;
	}


    Y.resize(Eigen::NoChange,d_nr_data);
	N.resize(Eigen::NoChange,d_nr_data);
	for(unsigned int i = 0; i < d_nr_data; i++){
		Y(0,i)	= dst->data(0,i);
		Y(1,i)	= dst->data(1,i);
		Y(2,i)	= dst->data(2,i);
		N(0,i)	= dst->normals(0,i);
		N(1,i)	= dst->normals(1,i);
		N(2,i)	= dst->normals(2,i);
	}

    Eigen::VectorXd dimweights(6);

    Eigen::Matrix<double, 6, Eigen::Dynamic>    Xf2;
    Eigen::Matrix<double, 6, Eigen::Dynamic>    Yf2;
    Xf2.resize(Eigen::NoChange,s_nr_data);
    Yf2.resize(Eigen::NoChange,d_nr_data);

    if(use_features){

		dimweights(0) = 0.01;
		dimweights(1) = 0.01;
		dimweights(2) = 0.01;
        dimweights(3) = 15.0;
        dimweights(4) = 15.0;
        dimweights(5) = 15.0;

        for(unsigned int j = 0; j < 6; j++){
            double mul = 1.0/dimweights(j);
            for(unsigned int i = 0; i < s_nr_data; i++){ Xf2(j,i) = src->data(j,i)*mul;}
            for(unsigned int i = 0; i < d_nr_data; i++){ Yf2(j,i) = dst->data(j,i)*mul;}
        }
    }

	unsigned int nr_features = feature_start.size();
	std::vector<Eigen::MatrixXd> Qf;
	if(use_features){
		Xf.resize(nr_features);
		Qf.resize(nr_features);
		Yf.resize(nr_features);
		for(unsigned int f = 0; f < nr_features; f++){
			int start			= feature_start[f];
			int end				= feature_end[f];
			Eigen::MatrixXd xf	= Eigen::MatrixXd(1+end-start,s_nr_data);
			Eigen::MatrixXd qf	= Eigen::MatrixXd(1+end-start,s_nr_data);
			Eigen::MatrixXd yf	= Eigen::MatrixXd(1+end-start,d_nr_data);
			for(unsigned int k = start; k <= end; k++){
				int ki = k-start;
				for(unsigned int i = 0; i < s_nr_data; i++){xf(ki,i)	= src->data(k,i);}
				for(unsigned int i = 0; i < d_nr_data; i++){yf(ki,i)	= dst->data(k,i);}
			}

			Xf[f] = xf;
			Qf[f] = qf;
			Yf[f] = yf;
		}
	}

	Eigen::VectorXd SRC_INORMATION = Eigen::VectorXd::Zero(X.cols());
	for(unsigned int i = 0; i < s_nr_data; i++){SRC_INORMATION(i) = src->information(0,i);}

	Eigen::VectorXd DST_INORMATION = Eigen::VectorXd::Zero(Y.cols());
	for(unsigned int i = 0; i < d_nr_data; i++){DST_INORMATION(i) = dst->information(0,i);}

	double stop		= 0.00005;
	func->p			= 0.5;

	/// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>                  kdtree(Y);

	/// Buffers
	Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	X.cols());
	Eigen::VectorXd  W		= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  Wold	= Eigen::VectorXd::Zero(	X.cols());
	Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	X.cols());

    Eigen::VectorXd  fwp	= Eigen::VectorXd::Zero(	X.cols());
    Eigen::VectorXd  fwn	= Eigen::VectorXd::Zero(	X.cols());
    Eigen::VectorXd  fwt	= Eigen::VectorXd::Zero(	X.cols());

	Eigen::Matrix3Xd Xo1 = X;
	Eigen::Matrix3Xd Xo2 = X;
	Eigen::Matrix3Xd Xo3 = X;
	Eigen::MatrixXd residuals;

	std::vector<int> matchid;
	matchid.resize(s_nr_data);

	std::vector<double> total_dweight;
	total_dweight.resize(d_nr_data);
	if(visualizationLvl >= 1){show(X,Y);}

	/// ICP
	for(int icp=0; icp < 150; ++icp) {
        if(use_features){
            dimweights(0) = func->getNoise();
            dimweights(1) = func->getNoise();
            dimweights(2) = func->getNoise();
            dimweights(3) = feature_func[0]->getNoise();
            dimweights(4) = feature_func[1]->getNoise();
            dimweights(5) = feature_func[2]->getNoise();

            for(unsigned int j = 0; j < 6; j++){
                double mul = 1.0/dimweights(j);
                if(j < 3){
                    for(unsigned int i = 0; i < s_nr_data; i++){ Xf2(j,i) = X(j,i)*mul;}
                    for(unsigned int i = 0; i < d_nr_data; i++){ Yf2(j,i) = Y(j,i)*mul;}
                }else{
                    for(unsigned int i = 0; i < s_nr_data; i++){ Xf2(j,i) = src->data(j,i)*mul;}
                    for(unsigned int i = 0; i < d_nr_data; i++){ Yf2(j,i) = dst->data(j,i)*mul;}
                }
            }

            nanoflann::KDTreeAdaptor<Eigen::Matrix<double, 6, Eigen::Dynamic>, 6, nanoflann::metric_L2> kdtree2(Yf2);//(Y);


            #pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {
                matchid[i] = kdtree2.closest(Xf2.col(i).data());
            }
        }else{
            #pragma omp parallel for
			for(unsigned int i=0; i< s_nr_data; ++i) {
                matchid[i] = kdtree.closest(X.col(i).data());
            }
        }

		/// Find closest point
		#pragma omp parallel for
		for(unsigned int i=0; i< s_nr_data; ++i) {
			int id = matchid[i];
			Qn.col(i) = N.col(id);
			Qp.col(i) = Y.col(id);
			rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			/*
			if(id < 0 || id > d_nr_data){
				printf("s_nr_data: %i",s_nr_data);
				printf("d_nr_data: %i\n",d_nr_data);
				printf("wtf... matched %i to %i\n",i,id);exit(0);
			}
			else{
				Qn.col(i) = N.col(id);
				Qp.col(i) = Y.col(id);
				rangeW(i) = 1.0/(1.0/SRC_INORMATION(i)+1.0/DST_INORMATION(id));
			}
			*/
		}

		//Deal with features
		if(use_features){
			for(unsigned int i=0; i< s_nr_data; ++i) {
                fwp(i) = 1;
                fwn(i) = 1;
            }
			Eigen::VectorXd fw_tot;
			for(unsigned int f = 0; f < nr_features; f++){

				Eigen::MatrixXd		xf	= Xf[f];
				Eigen::MatrixXd &	qf	= Qf[f];
				Eigen::MatrixXd		yf	= Yf[f];

				#pragma omp parallel for
				for(unsigned int i=0; i< s_nr_data; ++i) {
					int id = matchid[i];
					qf.col(i) = yf.col(id);
				}
				Eigen::MatrixXd res = xf-qf;

                //feature_func[f]->regularization = 0;
				feature_func[f]->computeModel(res);
                Eigen::VectorXd fw = feature_func[f]->getProbs(res);

				for(unsigned int i=0; i< s_nr_data; ++i) {
                    fwp(i) *= fw(i);
                    fwn(i) *= 1.0-fw(i);
                }
				float n = feature_func[f]->getNoise();
				//printf("feature %i noise %f\n",f,n);

			}

			for(unsigned int i=0; i< s_nr_data; ++i) {
                fwt(i) = fwp(i)/(fwp(i)+fwn(i));
            }
			//if(visualizationLvl >= 2){show(X,Qp,fwt);}
            //exit(0);
        }else{
			for(unsigned int i=0; i< s_nr_data; ++i) {
                fwt(i) = 1;
            }
        }
        
        if(visualizationLvl >= 2){show(X,Y);}

		for(int outer=0; outer< 5; ++outer) {
			/// Compute weights
			switch(type) {
				case PointToPoint:	{residuals = X-Qp;} 						break;
				case PointToPlane:	{residuals = Qn.array()*(X-Qp).array();}	break;
				default:			{printf("type not set\n");}					break;
			}
			for(unsigned int i=0; i<X.cols(); ++i) {residuals.col(i) *= rangeW(i);}
			switch(type) {
				case PointToPoint:	{func->computeModel(residuals); 				} 					break;
				case PointToPlane:	{func->computeModel(residuals.colwise().norm());}	break;
				default:  			{printf("type not set\n");} break;
			}
			

			for(int inner=0; inner< 20; ++inner) {
				printf("icp: %i outer: %i inner: %i\n",icp,outer,inner);
				if(inner != 0){
					switch(type) {
						case PointToPoint:	{residuals = X-Qp;} 						break;
						case PointToPlane:	{residuals = Qn.array()*(X-Qp).array();}	break;
						default:			{printf("type not set\n");}					break;
					}
					for(int i=0; i<X.cols(); ++i) {residuals.col(i) *= rangeW(i);}
				}
				
				switch(type) {
					case PointToPoint:	{W = func->getProbs(residuals); } 					break;
					case PointToPlane:	{W = func->getProbs(residuals.colwise().norm());}	break;
					default:			{printf("type not set\n");} break;
				}
				
				Wold = W;
				if(visualizationLvl >= 3){show(X,Qp,W);}
				//Normalizing weights has an effect simmilar to one to one matching
				//in that it reduces the effect of border points
				if(normalize_matchweights){
					for(unsigned int i=0; i < d_nr_data; ++i) {	total_dweight[i] = 0.0000001;}//Reset to small number to avoid division by zero
					for(unsigned int i=0; i< s_nr_data; ++i) {	total_dweight[matchid[i]] += W(i);}
					for(unsigned int i=0; i< s_nr_data; ++i) {	W(i) = W(i)*(W(i)/total_dweight[matchid[i]]);}
				}
                W = fwt.array()*W.array();
				if(visualizationLvl >= 3){show(X,Qp,W);}
                W = W.array()*rangeW.array()*rangeW.array();
						
				switch(type) {
					case PointToPoint:	{RigidMotionEstimator::point_to_point(X, Qp, W);}		break;
					case PointToPlane:	{RigidMotionEstimator::point_to_plane(X, Qp, Qn, W);}	break;
					default:  			{printf("type not set\n"); } break;
				}
				
				double stop1 = (X-Xo1).colwise().norm().maxCoeff();
				Xo1 = X;
				if(stop1 < stop) break;
			}
			double stop2 = (X-Xo2).colwise().norm().maxCoeff();
			Xo2 = X;
			if(stop2 < stop) break;
		}
		func->update();
        for(unsigned int f = 0; f < nr_features; f++){feature_func[f]->update();}

		double stop3 = (X-Xo3).colwise().norm().maxCoeff();
		Xo3 = X;
        if(stop3 < stop && icp > 150) break;
	}
	float score = Wold.sum()/func->getNoise()/float(s_nr_data);
	printf("sum: %f noise: %f score: %f\n",Wold.sum(),func->getNoise(),score);

	pcl::TransformationFromCorrespondences tfc;
	for(unsigned int i = 0; i < s_nr_data; i++){
		Eigen::Vector3f a (X(0,i),				X(1,i),			X(2,i));
		Eigen::Vector3f b (src->data(0,i),		src->data(1,i), src->data(2,i));
		tfc.add(b,a);
	}
	Eigen::Matrix4d np = tfc.getTransformation().matrix().cast<double>();

	if(visualizationLvl >= 1){show(X,Qp);}
	
	printf("--------------\n");
	printf("np\n");
	cout << np << endl;

	return FusionResults(np,0);
}		

}
