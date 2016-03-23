#include "MassRegistrationPPRColor.h"

#include "ICP.h"

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reglib
{

Eigen::Matrix4d getPoseTransform(Eigen::MatrixXd t){
	Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
	m.block(0,0,3,3) = t.block(0,0,3,3);
	m.block(0,3,3,1) = t.block(0,6,3,1);
	return m;
}


PointMatch::PointMatch(size_t src_, std::vector<size_t> & dst_){
	src = src_;
	dst = dst_;
}
PointMatch::~PointMatch(){}

typedef Eigen::Matrix< double,6, Eigen::Dynamic > Matrix6Xd;

namespace RigidMotionEstimator4 {


/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Confidence weights
template <typename Derived1, typename Derived2, typename Derived3>
Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X, Eigen::MatrixBase<Derived2>& Y, const Eigen::MatrixBase<Derived3>& w) {
    /// Normalize weight vector
    Eigen::VectorXd w_normalized = w/w.sum();
    /// De-mean
    Eigen::Vector3d X_mean, Y_mean;
    for(int i=0; i<3; ++i) {
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
    }
    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
        Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
        transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    } else {
        transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
    }
    transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
    /// Apply transformation
    X = transformation*X;
    /// Re-apply mean
    X.colwise() += X_mean;
    Y.colwise() += Y_mean;
    /// Return transformation
    return transformation;
}
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
template <typename Derived1, typename Derived2>
inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X, Eigen::MatrixBase<Derived2>& Y) {
    return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
}

    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5, typename Derived6>
    Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Xn,
                                   Eigen::MatrixBase<Derived3>& Y,
                                   Eigen::MatrixBase<Derived4>& Yn,
                                   const Eigen::MatrixBase<Derived5>& w,
                                   const Eigen::MatrixBase<Derived6>& u,
                                   bool dox,
                                   bool doy) {

        if(!dox && !doy){return Eigen::Affine3d::Identity();}

        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;

        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for(int i=0; i<3; ++i){X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();}
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
        /// Prepare LHS and RHS


        Matrix66 LHS1 = Matrix66::Zero();
        Vector6 RHS1 = Vector6::Zero();
        if(dox){
            Block33 TL = LHS1.topLeftCorner<3,3>();
            Block33 TR = LHS1.topRightCorner<3,3>();
            Block33 BR = LHS1.bottomRightCorner<3,3>();
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
            #pragma omp parallel
            {
                #pragma omp for
                for(int i=0; i<X.cols(); i++) {
                    C.col(i) = X.col(i).cross(Yn.col(i));
                }
                #pragma omp sections nowait
                {
                    #pragma omp section
                    for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
                    #pragma omp section
                    for(int i=0; i<X.cols(); i++) TR += (C.col(i)*Yn.col(i).transpose())*w(i);
                    #pragma omp section
                    for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(Yn.col(i), w(i));
                    #pragma omp section
                    for(int i=0; i<C.cols(); i++) {
                        double dist_to_plane = -((X.col(i) - Y.col(i)).dot(Yn.col(i)) - u(i))*w(i);
                        RHS1.head<3>() += C.col(i)*dist_to_plane;
                        RHS1.tail<3>() += Yn.col(i)*dist_to_plane;
                    }
                }
            }
            LHS1 = LHS1.selfadjointView<Eigen::Upper>();
        }

        Matrix66 LHS2 = Matrix66::Zero();
        Vector6 RHS2 = Vector6::Zero();
        if(doy){
            Block33 TL = LHS2.topLeftCorner<3,3>();
            Block33 TR = LHS2.topRightCorner<3,3>();
            Block33 BR = LHS2.bottomRightCorner<3,3>();
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,Y.cols());
            #pragma omp parallel
            {
                #pragma omp for
                for(int i=0; i<Y.cols(); i++) {
                    C.col(i) = Y.col(i).cross(Xn.col(i));
                }
                #pragma omp sections nowait
                {
                    #pragma omp section
                    for(int i=0; i<Y.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
                    #pragma omp section
                    for(int i=0; i<Y.cols(); i++) TR += (C.col(i)*Xn.col(i).transpose())*w(i);
                    #pragma omp section
                    for(int i=0; i<Y.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(Xn.col(i), w(i));
                    #pragma omp section
                    for(int i=0; i<C.cols(); i++) {
                        double dist_to_plane = -((Y.col(i) - X.col(i)).dot(Xn.col(i)) - u(i))*w(i);
                        RHS2.head<3>() += C.col(i)*dist_to_plane;
                        RHS2.tail<3>() += Xn.col(i)*dist_to_plane;
                    }
                }
            }
            LHS2 = LHS2.selfadjointView<Eigen::Upper>();
        }

        Matrix66 LHS = LHS1 + LHS2;
        Vector6 RHS = RHS1 - RHS2;
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
                                          const Eigen::MatrixBase<Derived5>& w,
                                          bool dox = true,
                                          bool doy = false) {
        return point_to_plane(X,Xn,Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()),dox,doy);
    }
}

MassRegistrationPPRColor::MassRegistrationPPRColor(double startreg, bool visualize){
	//type					= PointToPlane;
	type					= PointToPoint;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;
	max_matches				= 500;

	func = new DistanceWeightFunction2PPR2();
	func->update_size		= true;
	func->startreg			= startreg;
	func->debugg_print		= false;

	funcR = new DistanceWeightFunction2PPR2();
	funcR->fixed_histogram_size = true;
	funcR->startmaxd            = 2*255;
	funcR->starthistogram_size  = 2*255;
	funcR->startreg             = 60.0;
	funcR->debugg_print         = false;

	funcG = new DistanceWeightFunction2PPR2();
	funcG->fixed_histogram_size     = true;
	funcG->startmaxd                = 2*255;
	funcG->starthistogram_size      = 2*255;
	funcG->startreg                 = 60.0;
	funcG->debugg_print             = false;

	funcB = new DistanceWeightFunction2PPR2();
	funcB->fixed_histogram_size     = true;
	funcB->startmaxd                = 2*255;
	funcB->starthistogram_size      = 2*255;
	funcB->startreg                 = 60.0;
	funcB->debugg_print             = false;

	sp = nanoflann2::SearchParams(10);
    sp.eps = 0.5;

	stopval = 0.001;
    steps = 4;

	if(visualize){
		visualizationLvl = 1;

		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}else{
		visualizationLvl = 0;
	}

}
MassRegistrationPPRColor::~MassRegistrationPPRColor(){}

void MassRegistrationPPRColor::setData(std::vector<RGBDFrame*> frames_,std::vector<cv::Mat> masks_){
	frames = frames_;
	masks = masks_;

	all_matches.resize(frames.size());
	for(unsigned int i = 0; i < frames.size(); i++){
		preprocessData(i);
		for(unsigned int j = i+1; j < frames.size(); j++){connections.push_back(std::make_pair(i,j));}
		all_matches[i].resize(frames.size());
		for(unsigned int j = 0; j < frames.size(); j++){all_matches[i][j].clear();}
	}
}

void MassRegistrationPPRColor::preprocessData(int index){
	printf("preprocessData %i\n",index);
double startTime = getTime();

	unsigned char  * maskdata		= (unsigned char	*)(masks[index].data);
	unsigned char  * imgrgbdata		= (unsigned char	*)(frames[index]->rgb.data);
	unsigned short * depthdata		= (unsigned short	*)(frames[index]->depth.data);
	float		   * normalsdata	= (float			*)(frames[index]->normals.data);

	Camera * camera				= frames[index]->camera;
	const unsigned int width	= camera->width;
	const unsigned int height	= camera->height;
	const float idepth			= camera->idepth_scale;
	const float cx				= camera->cx;
	const float cy				= camera->cy;
	const float ifx				= 1.0/camera->fx;
	const float ify				= 1.0/camera->fy;

	int count = 0;

	for(unsigned int w = 0; w < width; w+=steps){
		for(unsigned int h = 0; h < height;h+=steps){
			int ind = h*width+w;
			if(maskdata[ind] == 255){
				float z = idepth*float(depthdata[ind]);
				float xn = normalsdata[3*ind+0];
				if(z > 0.2 && xn != 2){
					count++;
				}
			}
		}
	}

	double * data = new double[3*count];
	unsigned char * rgbdata = new unsigned char[3*count];
	Eigen::MatrixXd noise (count,6);
	int c = 0;
	for(unsigned int w = 0; w < width; w+=steps){
		for(unsigned int h = 0; h < height;h+=steps){
			if(c == count){continue;}
			int ind = h*width+w;
			if(maskdata[ind] == 255){
				float z = idepth*float(depthdata[ind]);
				float xn = normalsdata[3*ind+0];
				if(z > 0.2 && xn != 2){
					data[3*c+0] = (w - cx) * z * ifx;
					data[3*c+1] = (h - cy) * z * ify;
					data[3*c+2] = z;


					rgbdata[3*c+0] = imgrgbdata[3*ind+0];
					rgbdata[3*c+1] = imgrgbdata[3*ind+1];
					rgbdata[3*c+2] = imgrgbdata[3*ind+2];

					noise(c,0) = z*z;
					noise(c,1) = noise(c,0);
					noise(c,2) = noise(c,0);
					noise(c,3) = 1;
					noise(c,4) = 1;
					noise(c,5) = 1;
					c++;
				}
			}
		}
	}

		//clouds.push_back(cloud);
	ArrayData3D<double> * ad = new ArrayData3D<double>();
	ad->rows = count;
	ad->data = data;

	KDTree2 * index2 = new KDTree2(3, *ad, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
	index2->buildIndex();

	ads.push_back(ad);
	nrdatas.push_back(count);
	clouddatas.push_back(data);
	rgbdatas.push_back(rgbdata);
//std::vector< double* > clouddatas;
	treedatas.push_back(index2);

	noises.push_back(noise);

	printf("constructing the kdtree took %5.5fs\n",getTime()-startTime);
}

void MassRegistrationPPRColor::showMatches(int to, int from, Eigen::MatrixXd pose){
	double *			to_cloud		= clouddatas[to];
	int					to_nr_points	= nrdatas[to];
	Eigen::MatrixXd &	to_noise		= noises[to];

	double *			from_cloud		= clouddatas[from];
	int					from_nr_points	= nrdatas[from];
	Eigen::MatrixXd &	from_noise		= noises[from];

	const int step = std::max(1.0,double(from_nr_points)/double(max_matches));

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	for(unsigned int i = 0; i < to_nr_points; i++){
		pcl::PointXYZRGBNormal pi;
		pi.x = to_cloud[3*i+0];
		pi.y = to_cloud[3*i+1];
		pi.z = to_cloud[3*i+2];
		pi.b = 0;
		pi.g = 0;
		pi.r = 255;
		dcloud->points.push_back(pi);
	}

	Eigen::Matrix4d po = getPoseTransform(pose);
	float m00 = po(0,0); float m01 = po(0,1); float m02 = po(0,2); float m03 = po(0,3);
	float m10 = po(1,0); float m11 = po(1,1); float m12 = po(1,2); float m13 = po(1,3);
	float m20 = po(2,0); float m21 = po(2,1); float m22 = po(2,2); float m23 = po(2,3);

	for(size_t ii = 0; ii < from_nr_points; ii+=1){
		double x = from_cloud[3*ii+0];
		double y = from_cloud[3*ii+1];
		double z = from_cloud[3*ii+2];

		double tx	= m00*x + m01*y + m02*z + m03;
		double ty	= m10*x + m11*y + m12*z + m13;
		double tz	= m20*x + m21*y + m22*z + m23;
		pcl::PointXYZRGBNormal pi;
		pi.x = tx;
		pi.y = ty;
		pi.z = tz;
		pi.b = 0;
		pi.g = 255;
		pi.r = 0;
		scloud->points.push_back(pi);
	}

	std::vector< PointMatch > & matches = all_matches[to][from];
	for(int m = 0; m < matches.size(); m++){
		char buf [1024];
		for(int k = 0; k < matches[m].dst.size(); k++){
			sprintf(buf,"line%i_%i",m,k);
			viewer->addLine<pcl::PointXYZRGBNormal>(dcloud->points[matches[m].dst[k]],scloud->points[matches[m].src],buf);
		}
	}


	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();


	//exit(0);
//    printf("rematch %i %i took %5.5fs\n",from,to,getTime()-startTime);
}

void MassRegistrationPPRColor::rematchAll(std::vector<Eigen::MatrixXd> poses){
	show(poses, true);
	int nr_matches = 0;
	double sumtime = 0;
	#pragma omp parallel for
	const int nr_conns  = connections.size();
	for(int c = 0; c < nr_conns; c++){
		double startTime = getTime();
		int to = connections[c].first;
		int from = connections[c].second;

		Eigen::MatrixXd poseto = poses.at(to);
		Eigen::MatrixXd posefrom = poses.at(from);
		Eigen::MatrixXd relativePose = poseto.inverse()*posefrom;

		all_matches[to][from] = rematch(to,from,relativePose);
		//showMatches(to, from,relativePose);
		all_matches[from][to] = rematch(from,to,relativePose.inverse());

		nr_matches += all_matches[to][from].size() + all_matches[from][to].size();
		sumtime += getTime()-startTime;
		printf("rematchAll took %5.5fs averaging %10.10f matches per second\n",sumtime, double(nr_matches)/sumtime);
	}

printf("rematchAll took %5.5fs averaging %10.10f matches per second\n",sumtime, double(nr_matches)/sumtime);
}

std::vector< PointMatch > MassRegistrationPPRColor::rematch(int to, int from, Eigen::MatrixXd pose){
double startTime = getTime();

	std::vector< PointMatch > matches;

	KDTree2 * tree = treedatas[to];

	double *			to_cloud		= clouddatas[to];
	int					to_nr_points	= nrdatas[to];
	Eigen::MatrixXd &	to_noise		= noises[to];

	double *			from_cloud		= clouddatas[from];
	int					from_nr_points	= nrdatas[from];
	Eigen::MatrixXd &	from_noise		= noises[from];

	const int step = std::max(1.0,double(from_nr_points)/double(max_matches));

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	const bool debugg_matches = false;
	if(debugg_matches){
		for(unsigned int i = 0; i < to_nr_points; i++){
			pcl::PointXYZRGBNormal pi;
			pi.x = to_cloud[3*i+0];
			pi.y = to_cloud[3*i+1];
			pi.z = to_cloud[3*i+2];
			pi.b = 0;
			pi.g = 0;
			pi.r = 255;
			dcloud->points.push_back(pi);
		}
	}

	Eigen::Matrix4d po = getPoseTransform(pose);
	float m00 = po(0,0); float m01 = po(0,1); float m02 = po(0,2); float m03 = po(0,3);
	float m10 = po(1,0); float m11 = po(1,1); float m12 = po(1,2); float m13 = po(1,3);
	float m20 = po(2,0); float m21 = po(2,1); float m22 = po(2,2); float m23 = po(2,3);

	matches.reserve(max_matches);

	const size_t num_results = 1;
	for(size_t ii = 0; ii < from_nr_points; ii+=step){
		double x = from_cloud[3*ii+0];
		double y = from_cloud[3*ii+1];
		double z = from_cloud[3*ii+2];

		double tx	= m00*x + m01*y + m02*z + m03;
		double ty	= m10*x + m11*y + m12*z + m13;
		double tz	= m20*x + m21*y + m22*z + m23;
		double qp[3] = {tx,ty,tz};


		std::vector<double> out_dists_sqr(num_results);
		std::vector<size_t>   ret_indexes(num_results);
		nanoflann2::KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
		tree->findNeighbors(resultSet, qp, nanoflann::SearchParams(10));
		matches.push_back(PointMatch(ii,ret_indexes));

		if(debugg_matches){
			pcl::PointXYZRGBNormal pi;
			pi.x = tx;
			pi.y = ty;
			pi.z = tz;
			pi.b = 0;
			pi.g = 255;
			pi.r = 0;
			scloud->points.push_back(pi);

			char buf [1024];
			//printf("%i -> %f %f %f -> %f %f %f -> %f\n",ii,tx,ty,tz,to_cloud[3*ret_indexes[0]+0],to_cloud[3*ret_indexes[0]+1],to_cloud[3*ret_indexes[0]+2],sqrt(out_dists_sqr[0]));
			for(int k = 0; k < ret_indexes.size(); k++){
				sprintf(buf,"line%i_%i",ii,k);
				viewer->addLine<pcl::PointXYZRGBNormal>(dcloud->points[ret_indexes[k]],pi,buf);
			}
		}
	}

	if(debugg_matches){
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		viewer->spin();
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
	}

	printf("rematch %i %i took %5.5fs\n",from,to,getTime()-startTime);
	return matches;
}

void MassRegistrationPPRColor::recomputeFunctions(std::vector<Eigen::MatrixXd> poses){
	double startTime = getTime();

	int count = 0;
	for(int c = 0; c < connections.size(); c++){
		int i = connections[c].first;
		int j = connections[c].second;
		count += all_matches[i][j].size();
		count += all_matches[j][i].size();
	}

	Eigen::MatrixXd residualsR = Eigen::MatrixXd::Zero(1,count);
	Eigen::MatrixXd residualsG = Eigen::MatrixXd::Zero(1,count);
	Eigen::MatrixXd residualsB = Eigen::MatrixXd::Zero(1,count);
	Eigen::MatrixXd residualsP;

	printf("count %i\n",count);
	switch(type) {
		case PointToPoint:	{residualsP = Eigen::MatrixXd(3,count);}break;
		case PointToPlane:	{residualsP = Eigen::MatrixXd(1,count);}break;
		default:			{printf("type not set\n");}					break;
	}

	int current = 0;
	for(int c = 0; c < connections.size(); c++){
		int i = connections[c].first;
		int j = connections[c].second;

		Eigen::MatrixXd posei = poses.at(i);
		Eigen::MatrixXd posej = poses.at(j);

		Eigen::MatrixXd rp = posei.inverse()*posej;
		Eigen::MatrixXd irp = rp.inverse();

		std::vector< PointMatch > matches;
		matches = all_matches[i][j];

		double *			cloudi		= clouddatas[i];
		unsigned char *		rgbi		= rgbdatas[i];
		Eigen::MatrixXd &	noisei		= noises[i];

		double *			cloudj		= clouddatas[j];
		unsigned char *		rgbj		= rgbdatas[j];
		Eigen::MatrixXd &	noisej		= noises[j];


		Eigen::Matrix4d po = getPoseTransform(rp);
		float m00 = po(0,0); float m01 = po(0,1); float m02 = po(0,2); float m03 = po(0,3);
		float m10 = po(1,0); float m11 = po(1,1); float m12 = po(1,2); float m13 = po(1,3);
		float m20 = po(2,0); float m21 = po(2,1); float m22 = po(2,2); float m23 = po(2,3);


		for(unsigned int m = 0; m < matches.size(); m++){
			PointMatch & match = matches[m];

			size_t src = match.src;
			size_t dst = match.dst.front();

			double sx = cloudi[3*dst+0];
			double sy = cloudi[3*dst+1];
			double sz = cloudi[3*dst+2];

			double dx = cloudj[3*src+0];
			double dy = cloudj[3*src+1];
			double dz = cloudj[3*src+2];
			double tx	= m00*dx + m01*dy + m02*dz + m03;
			double ty	= m10*dx + m11*dy + m12*dz + m13;
			double tz	= m20*dx + m21*dy + m22*dz + m23;

			double ni = noisei(dst,0);
			double nj = noisej(src,0);

			double var = ni*ni+nj*nj;
			double nij = sqrt(var);
			switch(type) {
				case PointToPoint:	{
					residualsP(0,current) = (sx-tx)/nij;
					residualsP(1,current) = (sy-ty)/nij;
					residualsP(2,current) = (sz-tz)/nij;
				}break;
				case PointToPlane:	{}break;
				default:			{printf("type not set\n");}					break;
			}

			residualsR(0,current) = rgbi[3*dst+0]-rgbj[3*src+0];
			residualsG(0,current) = rgbi[3*dst+1]-rgbj[3*src+1];
			residualsB(0,current) = rgbi[3*dst+2]-rgbj[3*src+2];

			current++;
		}

		matches = all_matches[j][i];

		po = getPoseTransform(irp);
		m00 = po(0,0); m01 = po(0,1); m02 = po(0,2); m03 = po(0,3);
		m10 = po(1,0); m11 = po(1,1); m12 = po(1,2); m13 = po(1,3);
		m20 = po(2,0); m21 = po(2,1); m22 = po(2,2); m23 = po(2,3);


		for(unsigned int m = 0; m < matches.size(); m++){
			PointMatch & match = matches[m];

			size_t src = match.src;
			size_t dst = match.dst.front();

			double sx = cloudj[3*dst+0];
			double sy = cloudj[3*dst+1];
			double sz = cloudj[3*dst+2];

			double dx = cloudi[3*src+0];
			double dy = cloudi[3*src+1];
			double dz = cloudi[3*src+2];
			double tx	= m00*dx + m01*dy + m02*dz + m03;
			double ty	= m10*dx + m11*dy + m12*dz + m13;
			double tz	= m20*dx + m21*dy + m22*dz + m23;

			double ni = noisej(dst,0);
			double nj = noisei(src,0);

			double var = ni*ni+nj*nj;
			double nij = sqrt(var);
			switch(type) {
				case PointToPoint:	{
					residualsP(0,current) = (sx-tx)/nij;
					residualsP(1,current) = (sy-ty)/nij;
					residualsP(2,current) = (sz-tz)/nij;
				}break;
				case PointToPlane:	{}break;
				default:			{printf("type not set\n");}					break;
			}

			residualsR(0,current) = rgbj[3*dst+0]-rgbi[3*src+0];
			residualsG(0,current) = rgbj[3*dst+1]-rgbi[3*src+1];
			residualsB(0,current) = rgbj[3*dst+2]-rgbi[3*src+2];

			current++;
		}
	}
	func->computeModel(residualsP);
	funcR->computeModel(residualsR);
	funcG->computeModel(residualsG);
	funcB->computeModel(residualsB);
	printf("total matches: %i\n",count);

	printf("recomputeFunctions took %5.5fs\n",getTime()-startTime);
	exit(0);
}

bool isConverged(double stop_trans, double stop_rot, std::vector<Eigen::MatrixXd> poses1, std::vector<Eigen::MatrixXd> poses2){
	double change_trans = 0;
	double change_rot = 0;
	for(unsigned int i = 0; i < poses1.size(); i++){
		for(unsigned int j = i+1; j < poses1.size(); j++){
			Eigen::MatrixXd diff_before = poses1[i].inverse()*poses1[j];
			Eigen::MatrixXd diff_after	= poses2[i].inverse()*poses2[j];
			Eigen::Matrix4d diff		= getPoseTransform(diff_before.inverse()*diff_after);

			double dt = 0;
			for(unsigned int k = 0; k < 3; k++){
				dt += diff(k,3)*diff(k,3);
				for(unsigned int l = 0; l < 3; l++){
					if(k == l){ change_rot += fabs(1-diff(k,l));}
					else{		change_rot += fabs(diff(k,l));}
				}
			}
			change_trans += sqrt(dt);
		}
	}

	change_trans /= double(poses1.size()*(poses1.size()-1));
	change_rot	 /= double(poses1.size()*(poses1.size()-1));
	return change_trans < stop_trans && change_rot < stop_rot;
}

std::vector<Eigen::MatrixXd> MassRegistrationPPRColor::refinePoses(std::vector<Eigen::MatrixXd> poses){

	for(int outer=0; outer < 15; ++outer) {
		printf("outer: %i\n",outer);
		std::vector<Eigen::MatrixXd> posesOuter = poses;
		for(unsigned int i = 0; i < poses.size(); i++){
			unsigned int nr_match = 0;
/*
			for(unsigned int j = 0; j < poses.size(); j++){
				std::vector<PointMatch> & matchidj = matchids[j][i];
				unsigned int matchesj = matchidj.size();

				double * cloudj = clouddatas[j];
				for(unsigned int kj = 0; kj < matchesj; kj++){
					//nr_match++;
				}
			}
*/
				/*
				for(unsigned int i = 0; i < nr_pointsj; i++){
					double dx = cloudj[3*i+0];
					double dy = cloudj[3*i+1];
					double dz = cloudj[3*i+2];
					double tx	= m00*dx + m01*dy + m02*dz + m03;
					double ty	= m10*dx + m11*dy + m12*dz + m13;
					double tz	= m20*dx + m21*dy + m22*dz + m23;

					dcloud->points.push_back(pi);
				}

				std::vector< PointMatch > matches = all_matches[ii][jj];
				for(unsigned int m = 0; m < matches.size(); m++){
					char buf [1024];
					sprintf(buf,"line%i",m);
					viewer->addLine<pcl::PointXYZRGBNormal>(dcloud->points[matches[m].src],scloud->points[matches[m].dst.front()],buf);
				}
				*/
/*
				for(unsigned int ki = 0; ki < matchesi; ki++){
					int kj = matchidi[ki];

					if( kj >=  matchesj){continue;}

					if(matchidj[kj] == ki){nr_match++;}
				}
*/
/*
			Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
			Eigen::Matrix3Xd Xp_ori	= Eigen::Matrix3Xd::Zero(3,	nr_match);
			Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	nr_match);

			Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
			Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	nr_match);
			Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	nr_match);

			int count = 0;

			Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
			Eigen::Matrix<double, 3, Eigen::Dynamic> & Xi	= points[i];
			Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
			Eigen::Matrix<double, 3, Eigen::Dynamic> & Xni	= normals[i];
			Eigen::VectorXd & informationi					= informations[i];

			for(unsigned int j = 0; j < nr_frames; j++){
				Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
				Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
				Eigen::VectorXd & informationj					= informations[j];

				std::vector<int> & matchidj = matchids[j][i];
				unsigned int matchesj = matchidj.size();
				std::vector<int> & matchidi = matchids[i][j];
				unsigned int matchesi = matchidi.size();

				for(unsigned int ki = 0; ki < matchesi; ki++){
					int kj = matchidi[ki];

					if( kj >=  matchesj){continue;}
					if(matchidj[kj] == ki){
						Qp.col(count) = tXj.col(kj);
						Qn.col(count) = tXnj.col(kj);

						Xp_ori.col(count) = Xi.col(ki);
						Xp.col(count) = tXi.col(ki);

						Xn.col(count) = tXni.col(ki);
						rangeW(count) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
						count++;
					}
				}
			}printf("rematch_time: %5.5f\n",rematch_time);

			if(count == 0){break;}

			for(int inner=0; inner < 5; ++inner) {
				Eigen::MatrixXd residuals;
				switch(type) {
					case PointToPoint:	{residuals = Xp-Qp;} 						break;
					//case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
					case PointToPlane:	{
						residuals		= Eigen::MatrixXd::Zero(1,	Xp.cols());
						for(int i=0; i<Xp.cols(); ++i) {
							float dx = Xp(0,i)-Qp(0,i);
							float dy = Xp(1,i)-Qp(1,i);
							float dz = Xp(2,i)-Qp(2,i);
							float qx = Qn(0,i);
							float qy = Qn(1,i);
							float qz = Qn(2,i);
							float di = qx*dx + qy*dy + qz*dz;
							residuals(0,i) = di;
						}
					}break;
					default:			{printf("type not set\n");}					break;
				}
				for(unsigned int k=0; k < nr_match; ++k) {residuals.col(k) *= rangeW(k);}

				Eigen::VectorXd  W;

				switch(type) {
					case PointToPoint:	{W = func->getProbs(residuals); } 					break;
					case PointToPlane:	{
						W = func->getProbs(residuals);
						for(int k=0; k<nr_match; ++k) {W(k) = W(k)*float((Xn(0,k)*Qn(0,k) + Xn(1,k)*Qn(1,k) + Xn(2,k)*Qn(2,k)) > 0.0);}
					}	break;
					default:			{printf("type not set\n");} break;
				}

				W = W.array()*rangeW.array()*rangeW.array();
				Xo1 = Xp;
				switch(type) {
					case PointToPoint:	{RigidMotionEstimator4::point_to_point(Xp, Qp, W);}		break;
					case PointToPlane:	{RigidMotionEstimator4::point_to_plane(Xp, Xn, Qp, Qn, W);}	break;
					default:  			{printf("type not set\n"); } break;
				}

				double stop1 = (Xp-Xo1).colwise().norm().maxCoeff();
				//printf("funcupdate: %i rematching: %i outer: %i inner: %i stop1: %5.5f\n",funcupdate,rematching,outer,inner,stop1);

				Xo1 = Xp;
				if(stop1 < 0.001){;break; }
				else{}
			}

			pcl::TransformationFromCorrespondences tfc;
			for(unsigned int c = 0; c < nr_match; c++){
				Eigen::Vector3f a (Xp(0,c),				Xp(1,c),			Xp(2,c));
				Eigen::Vector3f b (Xp_ori(0,c),         Xp_ori(1,c),        Xp_ori(2,c));
				tfc.add(b,a);
			}
			poses[i] = tfc.getTransformation().cast<double>().matrix();
			*/
		}
		if(isConverged(0.001, 0.001, poses, posesOuter)){break;}
	}

	return poses;
}

void MassRegistrationPPRColor::show(std::vector<Eigen::MatrixXd> guess, bool color){
	printf("show\n");
	viewer->removeAllPointClouds();
	for(int ii = 0; ii < guess.size(); ii++){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		Eigen::Matrix4d po = getPoseTransform(guess[ii]);
		float m00 = po(0,0); float m01 = po(0,1); float m02 = po(0,2); float m03 = po(0,3);
		float m10 = po(1,0); float m11 = po(1,1); float m12 = po(1,2); float m13 = po(1,3);
		float m20 = po(2,0); float m21 = po(2,1); float m22 = po(2,2); float m23 = po(2,3);

		int r = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		int g = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		int b = 256*(1+(rand()%4))/4 - 1;//255*(xi & 1);

		double * datas = clouddatas[ii];
		int nr_points = nrdatas[ii];
		for(unsigned int i = 0; i < nr_points; i++){
			double dx = datas[3*i+0];
			double dy = datas[3*i+1];
			double dz = datas[3*i+2];
			double tx	= m00*dx + m01*dy + m02*dz + m03;
			double ty	= m10*dx + m11*dy + m12*dz + m13;
			double tz	= m20*dx + m21*dy + m22*dz + m23;

			pcl::PointXYZRGBNormal pi;
			pi.x = tx;
			pi.y = ty;
			pi.z = tz;
			pi.b = b;
			pi.g = g;
			pi.r = r;
			cloud->points.push_back(pi);
		}

		char buf [1024];
		sprintf(buf,"cloud%i",ii);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf);
	}
	viewer->spin();
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
}

MassFusionResults MassRegistrationPPRColor::getTransforms(std::vector<Eigen::Matrix4d> guess){
	printf("start MassRegistrationPPRColor::getTransforms(std::vector<Eigen::Matrix4d> poses)\n");


	unsigned int nr_frames = frames.size();
	if(guess.size() != nr_frames){
		printf("ERROR: poses.size() != nr_frames\n");
		return MassFusionResults();
	}

    std::vector<Eigen::MatrixXd> fullposes (nr_frames);
	for(unsigned int i = 0; i < nr_frames; i++){
		Eigen::Matrix4d g = guess[i];
		Eigen::MatrixXd pose = Eigen::MatrixXd::Identity(7,7);
		pose.block(0,0,3,3) = g.block(0,0,3,3);
		pose.block(0,6,3,1) = g.block(0,3,3,1);
		printf("--------------%i---------------\n",i);
		std::cout << g << std::endl << std::endl;
		//std::cout << pose << std::endl << std::endl;
        fullposes[i] = pose;
	}

	func->reset();
	funcR->reset();
	funcG->reset();
	funcB->reset();
	func->noiseval	= 0;
	funcR->noiseval = 0;
	funcG->noiseval = 0;
	funcB->noiseval = 0;

	//Update functions regularizations
	steps = 4;

	for(int funcupdate=0; funcupdate < 30; ++funcupdate) {
		printf("funcupdate: %i\n",funcupdate);
		for(int rematching=0; rematching < 1; ++rematching) {
			printf("funcupdate: %i rematching: %i\n",funcupdate,rematching);
			rematchAll(fullposes);
			recomputeFunctions(fullposes);
			fullposes = refinePoses(fullposes);
		}


		double noise_before  = func->getNoise();
		double noise_beforeR = funcR->getNoise();
		double noise_beforeG = funcG->getNoise();
		double noise_beforeB = funcB->getNoise();
		func->update();
		funcR->update();
		funcG->update();
		funcB->update();
		double noise_after  = func->getNoise();
		double noise_afterR = funcR->getNoise();
		double noise_afterG = funcG->getNoise();
		double noise_afterB = funcB->getNoise();

		double logdiff = log2(noise_after/noise_before);
		double logdiffR = log2(noise_afterR/noise_beforeR);
		double logdiffG = log2(noise_afterG/noise_beforeG);
		double logdiffB = log2(noise_afterB/noise_beforeB);
		printf("P before:  %f after:  %f logdiff: %f\n",noise_before, noise_after,  logdiff);
		printf("R before:  %f after:  %f logdiff: %f\n",noise_beforeR, noise_afterR,  logdiffR);
		printf("G before:  %f after:  %f logdiff: %f\n",noise_beforeG, noise_afterG,  logdiffG);
		printf("B before:  %f after:  %f logdiff: %f\n",noise_beforeB, noise_afterB,  logdiffB);
		//printf("beforeR: %f afterR: %f diff: %f\n",noise_beforeR,noise_afterR, fabs(1.0 - noise_afterR/noise_beforeR));
		//printf("beforeG: %f afterG: %f diff: %f\n",noise_beforeG,noise_afterG, fabs(1.0 - noise_afterG/noise_beforeG));
		//printf("beforeB: %f afterB: %f diff: %f\n",noise_beforeB,noise_afterB, fabs(1.0 - noise_afterB/noise_beforeB));

		double tresh = 0.1;
		if(fabs(logdiff) < tresh && fabs(logdiffR) < tresh && fabs(logdiffG) < tresh && fabs(logdiffB) < tresh){break;}
	}
/*
	double noise_before = func->getNoise();
	func->update();
	double noise_after = func->getNoise();
	//printf("before: %f after: %f\n",noise_before,noise_after);
	if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
}

printf("total_time:   %5.5f\n",getTime()-total_time_start);
printf("rematch_time: %5.5f\n",rematch_time);
printf("computeModel: %5.5f\n",computeModel_time);
*/
/*

    std::vector<Eigen::Matrix4d> poses = guess;
    std::vector<Eigen::Matrix4d> poses1; poses1.resize(poses.size());
    std::vector<Eigen::Matrix4d> poses2; poses2.resize(poses.size());
    std::vector<Eigen::Matrix4d> poses2b; poses2b.resize(poses.size());
    std::vector<Eigen::Matrix4d> poses3; poses3.resize(poses.size());
    std::vector<Eigen::Matrix4d> poses4; poses4.resize(poses.size());

    nr_matches.resize(nr_frames);
    matchids.resize(nr_frames);
    nr_datas.resize(nr_frames);
    points.resize(nr_frames);
    colors.resize(nr_frames);
    pointscolors.resize(nr_frames);
    normals.resize(nr_frames);
    transformed_points.resize(nr_frames);
    transformed_normals.resize(nr_frames);

    for(unsigned int i = 0; i < nr_frames; i++){
        //printf("start local : data loaded successfully\n");
        printf("loading data for %i\n",i);

        unsigned char  * maskdata		= (unsigned char	*)(masks[i].data);
        unsigned char  * rgbdata		= (unsigned char	*)(frames[i]->rgb.data);
        unsigned short * depthdata		= (unsigned short	*)(frames[i]->depth.data);
        float		   * normalsdata	= (float			*)(frames[i]->normals.data);

        Eigen::Matrix4d p = poses[i];
        float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
        float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
        float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

        Camera * camera				= frames[i]->camera;
        const unsigned int width	= camera->width;
        const unsigned int height	= camera->height;
        const float idepth			= camera->idepth_scale;
        const float cx				= camera->cx;
        const float cy				= camera->cy;
        const float ifx				= 1.0/camera->fx;
        const float ify				= 1.0/camera->fy;

        int count = 0;

        //for(unsigned int ind = 0; ind < width*height; ind++){count += (maskdata[ind] == 255);}
        printf("steps: %i\n",steps);
        for(unsigned int w = 0; w < width; w+=steps){
            for(unsigned int h = 0; h < height;h+=steps){
                int ind = h*width+w;
                if(maskdata[ind] == 255){
                    float z = idepth*float(depthdata[ind]);
                    float xn = normalsdata[3*ind+0];

                    if(z > 0.2 && xn != 2){
                        count++;
                    }
                }
            }
        }

        nr_datas[i] = count;
        matchids[i].resize(nr_frames);
        points[i].resize(Eigen::NoChange,count);
        colors[i].resize(Eigen::NoChange,count);
        pointscolors[i].resize(Eigen::NoChange,count);
        normals[i].resize(Eigen::NoChange,count);
        transformed_points[i].resize(Eigen::NoChange,count);
        transformed_normals[i].resize(Eigen::NoChange,count);


        Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= points[i];
        Eigen::Matrix<double, 3, Eigen::Dynamic> & C	= colors[i];
        Eigen::Matrix<double, 7, Eigen::Dynamic> & XC	= pointscolors[i];
        Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= normals[i];
        Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= transformed_points[i];
        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= transformed_normals[i];
        Eigen::VectorXd information (count);

        int c = 0;
        for(unsigned int w = 0; w < width; w+=steps){
            for(unsigned int h = 0; h < height;h+=steps){
                if(c == count){continue;}
                int ind = h*width+w;
                if(maskdata[ind] == 255){
                    float z = idepth*float(depthdata[ind]);
                    float xn = normalsdata[3*ind+0];

                    if(z > 0.2 && xn != 2){
                        float yn = normalsdata[3*ind+1];
                        float zn = normalsdata[3*ind+2];

                        float x = (w - cx) * z * ifx;
                        float y = (h - cy) * z * ify;

                        X(0,c)	= x;
                        X(1,c)	= y;
                        X(2,c)	= z;
                        Xn(0,c)	= xn;
                        Xn(1,c)	= yn;
                        Xn(2,c)	= zn;

                        tX(0,c)		= m00*x + m01*y + m02*z + m03;
                        tX(1,c)		= m10*x + m11*y + m12*z + m13;
                        tX(2,c)		= m20*x + m21*y + m22*z + m23;
                        tXn(0,c)	= m00*xn + m01*yn + m02*zn;
                        tXn(1,c)	= m10*xn + m11*yn + m12*zn;
                        tXn(2,c)	= m20*xn + m21*yn + m22*zn;

                        //printf("c: %i count: %i ",c,count);
                        information(c) = 1.0/(z*z);

                        C(0,c) = rgbdata[3*ind+0];
                        C(1,c) = rgbdata[3*ind+1];
                        C(2,c) = rgbdata[3*ind+2];

                        XC(0,c)	= x;
                        XC(1,c)	= y;
                        XC(2,c)	= z;
                        XC(3,c) = rgbdata[3*ind+0]*0.001;
                        XC(4,c) = rgbdata[3*ind+1]*0.001;
                        XC(5,c) = rgbdata[3*ind+2]*0.001;
                        XC(6,c) = 1;
                        c++;
                    }
                }
            }
        }
        informations.push_back(information);
        trees.push_back(new nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple>(X));
        trees6d.push_back(new nanoflann::KDTreeAdaptor<Eigen::Matrix<double, 6, Eigen::Dynamic>, 6, nanoflann::metric_L2_Simple>(XC.block(0,0,6,count)));
    }
    func->reset();


    Eigen::MatrixXd Xo1;

    int imgcount = 0;

    double good_rematches = 0;
    double total_rematches = 0;

    double rematch_time = 0;
    double rematch2_time = 0;
    double residuals_time = 0;
    double computeModel_time = 0;
    double total_time_start = getTime();



    for(int funcupdate=0; funcupdate < 100; ++funcupdate) {
        if(visualizationLvl == 2){
            std::vector<Eigen::MatrixXd> Xv;
            for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
            char buf [1024];
            sprintf(buf,"image%5.5i.png",imgcount++);
            show(Xv,true,std::string(buf),imgcount);
        }

        printf("funcupdate: %i\n",funcupdate);
        for(int rematching=0; rematching < 40; ++rematching) {
            printf("funcupdate: %i rematching: %i\n",funcupdate,rematching);

            if(visualizationLvl == 3){
                printf("visualize\n");
                std::vector<Eigen::MatrixXd> Xv;
                for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
                char buf [1024];
                sprintf(buf,"image%5.5i.png",imgcount++);
                show(Xv,true,std::string(buf),imgcount);
            }
//exit(0);
            for(unsigned int i = 0; i < nr_frames; i++){poses1[i] = poses[i];}

            double rematch_time_start = getTime();
            for(unsigned int i = 0; i < nr_frames; i++){
                nr_matches[i] = 0;
                for(unsigned int j = 0; j < nr_frames; j++){
                    if(i == j){continue;}
                    Eigen::Affine3d relative_pose = Eigen::Affine3d(poses[j].inverse()*poses[i]);

                    Eigen::Matrix<double, 3, Eigen::Dynamic> tX	= relative_pose*points[i];
                    nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * tree = trees[j];

                    unsigned int nr_data = nr_datas[i];
                    std::vector<int> & matchid = matchids[i][j];
                    matchid.resize(nr_data);
                    for(unsigned int k = 0; k < nr_data; ++k) {
                        int prev = matchid[k];
                        int current =  tree->closest(tX.col(k).data());
                        good_rematches += prev != current;
                        total_rematches++;
                        matchid[k] = current;
                        //printf("%i -> %i\n",k,matchid[k]);
                    }
                    //exit(0);
                    nr_matches[i] += matchid.size();
                }
            }
            rematch_time += getTime()-rematch_time_start;

printf("percentage: %5.5f (good_rematches: %f total_rematches: %f)\n",good_rematches/total_rematches,good_rematches,total_rematches);
printf("rematch_time: %5.5f average match per second: %f\n",rematch_time,total_rematches/rematch_time);


printf("funcupdate: %i\n",funcupdate);
for(int rematching2=0; rematching2 < 1; ++rematching2) {
    printf("funcupdate: %i rematching: %i\n",funcupdate,rematching2);
    for(unsigned int i = 0; i < nr_frames; i++){poses1[i] = poses[i];}
    double rematch2_time_start = getTime();

    for(unsigned int i = 0; i < nr_frames; i++){
        for(unsigned int j = 0; j < nr_frames; j++){
            if(i == j){continue;}
            Eigen::Matrix<double, 7, 7> relative_pose = fullposes[j].inverse()*fullposes[i];
            Eigen::Matrix<double, 7, Eigen::Dynamic> tX	= relative_pose*pointscolors[i];//relative_pose*pointscolors[i];

            nanoflann::KDTreeAdaptor<Eigen::Matrix<double, 6, Eigen::Dynamic>, 6, nanoflann::metric_L2_Simple> * tree = trees6d[j];

            unsigned int nr_data = nr_datas[i];
            for(unsigned int k = 0; k < nr_data; ++k) {

                // Unsorted radius search:
                const double radius = 0.05;
                std::vector<std::pair<int,double> > indices_dists;
                //nanoflann::RadiusResultSet<double,size_t> resultSet(radius,indices_dists);
                double * query_point = tX.col(k).head(6).data();

                int out_indices;
                double out_distances_sq;
                //tree->query(query_point, 1, &out_indices, &out_distances_sq);

                nanoflann::KNNResultSet<int,double> resultSet(1);
                resultSet.init(&out_indices, &out_distances_sq);
                tree->index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());


                //tree->index->radiusSearch(qp,radius, indices_dists);

                //tree->index->findNeighbors(resultSet, tX.col(k).head(6).data() , sp);

 //                   // Get worst (furthest) point, without sorting:
 //                   std::pair<size_t,num_t> worst_pair = resultSet.worst_item();
 //                   cout << "Worst pair: idx=" << worst_pair.first << " dist=" << worst_pair.second << endl;
 //               int current =  tree->closest(tX.col(k).head(6).data());
            }
        }
    }
    rematch2_time += getTime()-rematch2_time_start;
printf("rematch2_time: %5.5f average match per second: %f\n",rematch2_time,total_rematches/rematch2_time);

}

exit(0);
            for(unsigned int i = 0; i < nr_frames; i++){
                nr_matches[i] = 0;
                for(unsigned int j = 0; j < nr_frames; j++){
                    nr_matches[i] += matchids[i][j].size()+matchids[j][i].size();
                }
            }

            int total_matches = 0;
            for(unsigned int i = 0; i < nr_frames; i++){
                for(unsigned int j = 0; j < nr_frames; j++){
                    total_matches += matchids[i][j].size();
                }
            }

            //printf("total_matches: %i\n",total_matches);
            for(int lala = 0; lala < 1; lala++){
                if(visualizationLvl == 4){
                    printf("visualize\n");
                    std::vector<Eigen::MatrixXd> Xv;
                    for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
                    char buf [1024];
                    sprintf(buf,"image%5.5i.png",imgcount++);
                    show(Xv,true,std::string(buf),imgcount);
                }

                for(unsigned int i = 0; i < nr_frames; i++){poses2b[i] = poses[i];}


                double residuals_time_start = getTime();

                Eigen::MatrixXd all_residuals;
                switch(type) {
                    case PointToPoint:	{all_residuals = Eigen::Matrix3Xd::Zero(3,total_matches);}break;
                    case PointToPlane:	{all_residuals = Eigen::MatrixXd::Zero(1,total_matches);}break;
                    default:			{printf("type not set\n");}					break;
                }

                int count = 0;

                for(unsigned int i = 0; i < nr_frames; i++){
                    Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
                    Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
                    Eigen::VectorXd & informationi					= informations[i];

                    for(unsigned int j = 0; j < nr_frames; j++){
                        if(i == j){continue;}
                        std::vector<int> & matchidi = matchids[i][j];
                        unsigned int matchesi = matchidi.size();

                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
                        Eigen::VectorXd & informationj					= informations[j];

		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		//printf("before: %f after: %f\n",noise_before,noise_after);
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
                        Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
                        Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	matchesi);

                        Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
                        Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	matchesi);
                        Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	matchesi);

                        for(unsigned int ki = 0; ki < matchesi; ki++){
                            int kj = matchidi[ki];
                            if( ki >= Qp.cols() || kj < 0 || kj >= tXj.cols() ){continue;}

                            //printf("%i %i / %i %i\n",ki,kj,Qp.cols(),tXj.cols());

                            Qp.col(ki) = tXj.col(kj);
                            Qn.col(ki) = tXnj.col(kj);

                            Xp.col(ki) = tXi.col(ki);
                            Xn.col(ki) = tXni.col(ki);
                            rangeW(ki) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
                        }

                        Eigen::MatrixXd residuals;
                        switch(type) {
                            case PointToPoint:	{residuals = Xp-Qp;} 						break;
                            case PointToPlane:	{
                                residuals		= Eigen::MatrixXd::Zero(1,	Xp.cols());
                                for(int i=0; i<Xp.cols(); ++i) {
                                    float dx = Xp(0,i)-Qp(0,i);
                                    float dy = Xp(1,i)-Qp(1,i);
                                    float dz = Xp(2,i)-Qp(2,i);
                                    float qx = Qn(0,i);
                                    float qy = Qn(1,i);
                                    float qz = Qn(2,i);
                                    float di = qx*dx + qy*dy + qz*dz;

                                    residuals(0,i) = di;
                                }
                            }break;
                            default:			{printf("type not set\n");}					break;
                        }

                        for(unsigned int k=0; k < matchesi; ++k) {residuals.col(k) *= rangeW(k);}

                        all_residuals.block(0,count,residuals.rows(),residuals.cols()) = residuals;
                        count += residuals.cols();
                    }
                }

                residuals_time += getTime()-residuals_time_start;

                double computeModel_time_start = getTime();
                switch(type) {
                    case PointToPoint:	{func->computeModel(all_residuals);} 	break;
                    case PointToPlane:	{func->computeModel(all_residuals);}	break;
                    default:  			{printf("type not set\n");} break;
                }
                computeModel_time += getTime()-computeModel_time_start;
                //func->debugg_print = false;

                for(int outer=0; outer < 150; ++outer) {
                    printf("funcupdate: %i rematching: %i lala: %i outer: %i\n",funcupdate,rematching,lala,outer);
                    for(unsigned int i = 0; i < nr_frames; i++){poses2[i] = poses[i];}

                    //printf("funcupdate: %i rematching: %i outer: %i\n",funcupdate,rematching,outer);
                    for(unsigned int i = 0; i < nr_frames; i++){
                        unsigned int nr_match = 0;
                        for(unsigned int j = 0; j < nr_frames; j++){
                            std::vector<int> & matchidj = matchids[j][i];
                            unsigned int matchesj = matchidj.size();
                            std::vector<int> & matchidi = matchids[i][j];
                            unsigned int matchesi = matchidi.size();

                            for(unsigned int ki = 0; ki < matchesi; ki++){
                                int kj = matchidi[ki];

                                if( kj >=  matchesj){continue;}

                                if(matchidj[kj] == ki){nr_match++;}
                            }
                        }

                        Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
                        Eigen::Matrix3Xd Xp_ori	= Eigen::Matrix3Xd::Zero(3,	nr_match);
                        Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	nr_match);

                        Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	nr_match);
                        Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	nr_match);
                        Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	nr_match);

                        int count = 0;

                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & Xi	= points[i];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & Xni	= normals[i];
                        Eigen::VectorXd & informationi					= informations[i];

                        for(unsigned int j = 0; j < nr_frames; j++){
                            Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
                            Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
                            Eigen::VectorXd & informationj					= informations[j];

                            std::vector<int> & matchidj = matchids[j][i];
                            unsigned int matchesj = matchidj.size();
                            std::vector<int> & matchidi = matchids[i][j];
                            unsigned int matchesi = matchidi.size();

                            for(unsigned int ki = 0; ki < matchesi; ki++){
                                int kj = matchidi[ki];

                                if( kj >=  matchesj){continue;}
                                if(matchidj[kj] == ki){
                                    Qp.col(count) = tXj.col(kj);
                                    Qn.col(count) = tXnj.col(kj);

                                    Xp_ori.col(count) = Xi.col(ki);
                                    Xp.col(count) = tXi.col(ki);

                                    Xn.col(count) = tXni.col(ki);
                                    rangeW(count) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
                                    count++;
                                }
                            }
                        }printf("rematch_time: %5.5f\n",rematch_time);

                        if(count == 0){break;}

                        for(int inner=0; inner < 5; ++inner) {
                            Eigen::MatrixXd residuals;
                            switch(type) {
                                case PointToPoint:	{residuals = Xp-Qp;} 						break;
                                //case PointToPlane:	{residuals = Qn.array()*(Xp-Qp).array();}	break;
                                case PointToPlane:	{
                                    residuals		= Eigen::MatrixXd::Zero(1,	Xp.cols());
                                    for(int i=0; i<Xp.cols(); ++i) {
                                        float dx = Xp(0,i)-Qp(0,i);
                                        float dy = Xp(1,i)-Qp(1,i);
                                        float dz = Xp(2,i)-Qp(2,i);
                                        float qx = Qn(0,i);
                                        float qy = Qn(1,i);
                                        float qz = Qn(2,i);
                                        float di = qx*dx + qy*dy + qz*dz;
                                        residuals(0,i) = di;
                                    }
                                }break;
                                default:			{printf("type not set\n");}					break;
                            }
                            for(unsigned int k=0; k < nr_match; ++k) {residuals.col(k) *= rangeW(k);}

                            Eigen::VectorXd  W;

                            switch(type) {
                                case PointToPoint:	{W = func->getProbs(residuals); } 					break;
                                case PointToPlane:	{
                                    W = func->getProbs(residuals);
                                    for(int k=0; k<nr_match; ++k) {W(k) = W(k)*float((Xn(0,k)*Qn(0,k) + Xn(1,k)*Qn(1,k) + Xn(2,k)*Qn(2,k)) > 0.0);}
                                }	break;
                                default:			{printf("type not set\n");} break;
                            }

                            W = W.array()*rangeW.array()*rangeW.array();
                            Xo1 = Xp;
                            switch(type) {
                                case PointToPoint:	{RigidMotionEstimator4::point_to_point(Xp, Qp, W);}		break;
                                case PointToPlane:	{RigidMotionEstimator4::point_to_plane(Xp, Xn, Qp, Qn, W);}	break;
                                default:  			{printf("type not set\n"); } break;
                            }

                            double stop1 = (Xp-Xo1).colwise().norm().maxCoeff();
                            //printf("funcupdate: %i rematching: %i outer: %i inner: %i stop1: %5.5f\n",funcupdate,rematching,outer,inner,stop1);

                            Xo1 = Xp;
                            if(stop1 < 0.001){;break; }
                            else{}
                        }

                        pcl::TransformationFromCorrespondences tfc;
                        for(unsigned int c = 0; c < nr_match; c++){
                            Eigen::Vector3f a (Xp(0,c),				Xp(1,c),			Xp(2,c));
                            Eigen::Vector3f b (Xp_ori(0,c),         Xp_ori(1,c),        Xp_ori(2,c));
                            tfc.add(b,a);
                        }
                        poses[i] = tfc.getTransformation().cast<double>().matrix();
                    }

                    double change = 0;
                    Eigen::Matrix4d p0inv = poses[0].inverse();
                    for(unsigned int j = 0; j < nr_frames; j++){
                        poses[j] = p0inv*poses[j];

                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[j];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & Xi	= points[j];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[j];
                        Eigen::Matrix<double, 3, Eigen::Dynamic> & Xni	= normals[j];

                        Eigen::Matrix4d p = poses[j];
                        float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
                        float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
                        float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

                        for(int c = 0; c < Xi.cols(); c++){
                            float x = Xi(0,c);
                            float y = Xi(1,c);
                            float z = Xi(2,c);

                            float nx = Xni(0,c);
                            float ny = Xni(1,c);
                            float nz = Xni(2,c);

                            double tXi0c	= tXi(0,c);
                            double tXi1c	= tXi(1,c);
                            double tXi2c	= tXi(2,c);

                            tXi(0,c)		= m00*x + m01*y + m02*z + m03;
                            tXi(1,c)		= m10*x + m11*y + m12*z + m13;
                            tXi(2,c)		= m20*x + m21*y + m22*z + m23;

                            tXni(0,c)		= m00*nx + m01*ny + m02*nz;
                            tXni(1,c)		= m10*nx + m11*ny + m12*nz;
                            tXni(2,c)		= m20*nx + m21*ny + m22*nz;

                            double dx = tXi0c-tXi(0,c);
                            double dy = tXi1c-tXi(1,c);
                            double dz = tXi2c-tXi(2,c);
                            double d = dx*dx+dy*dy+dz*dz;
                            change = std::max(d,change);
                        }
                    }
                    change = sqrt(change);

                    double change_trans = 0;
                    double change_rot = 0;
                    for(unsigned int i = 0; i < nr_frames; i++){
                        for(unsigned int j = i+1; j < nr_frames; j++){
                            Eigen::Matrix4d diff_before = poses2[i].inverse()*poses2[j];
                            Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
                            Eigen::Matrix4d diff = diff_before.inverse()*diff_after;

                            double dt = 0;
                            for(unsigned int k = 0; k < 3; k++){
                                dt += diff(k,3)*diff(k,3);
                                for(unsigned int l = 0; l < 3; l++){
                                    if(k == l){ change_rot += fabs(1-diff(k,l));}
                                    else{		change_rot += fabs(diff(k,l));}
                                }
                            }
                            change_trans += sqrt(dt);
                        }
                    }

                    change_trans /= double(nr_frames*(nr_frames-1));
                    change_rot	 /= double(nr_frames*(nr_frames-1));

                    if(change_trans < stopval && change_rot < stopval){break;}
                }

                double change_trans = 0;
                double change_rot = 0;
                for(unsigned int i = 0; i < nr_frames; i++){
                    for(unsigned int j = i+1; j < nr_frames; j++){
                        Eigen::Matrix4d diff_before = poses2b[i].inverse()*poses2b[j];
                        Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
                        Eigen::Matrix4d diff = diff_before.inverse()*diff_after;

                        double dt = 0;
                        for(unsigned int k = 0; k < 3; k++){
                            dt += diff(k,3)*diff(k,3);
                            for(unsigned int l = 0; l < 3; l++){
                                if(k == l){ change_rot += fabs(1-diff(k,l));}
                                else{		change_rot += fabs(diff(k,l));}
                            }
                        }
                        change_trans += sqrt(dt);
                    }
                }

                change_trans /= double(nr_frames*(nr_frames-1));
                change_rot	 /= double(nr_frames*(nr_frames-1));

                if(change_trans < stopval && change_rot < stopval){break;}
            }

            double change_trans = 0;
            double change_rot = 0;
            for(unsigned int i = 0; i < nr_frames; i++){
                for(unsigned int j = i+1; j < nr_frames; j++){
                    Eigen::Matrix4d diff_before = poses1[i].inverse()*poses1[j];
                    Eigen::Matrix4d diff_after	= poses[i].inverse()*poses[j];
                    Eigen::Matrix4d diff = diff_before.inverse()*diff_after;

                    double dt = 0;
                    for(unsigned int k = 0; k < 3; k++){
                        dt += diff(k,3)*diff(k,3);
                        for(unsigned int l = 0; l < 3; l++){
                            if(k == l){ change_rot += fabs(1-diff(k,l));}
                            else{		change_rot += fabs(diff(k,l));}
                        }
                    }
                    change_trans += sqrt(dt);
                }
            }

            change_trans /= double(nr_frames*(nr_frames-1));
            change_rot	 /= double(nr_frames*(nr_frames-1));

            //printf("retmatch: change relative transforms: %10.10f %10.10f\n",change_trans,change_rot);
            if(change_trans < stopval && change_rot < stopval){break;}
        }

        //std::vector<Eigen::MatrixXd> Xv;
        //for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
        //char buf [1024];
        //sprintf(buf,"image%5.5i.png",imgcount++);
        //show(Xv,true,std::string(buf),imgcount);

        double noise_before = func->getNoise();
        func->update();
        double noise_after = func->getNoise();
        //printf("before: %f after: %f\n",noise_before,noise_after);
        if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
    }

    printf("total_time:   %5.5f\n",getTime()-total_time_start);
    printf("rematch_time: %5.5f\n",rematch_time);
    printf("computeModel: %5.5f\n",computeModel_time);
*/

//	if(visualizationLvl > 0){
//		std::vector<Eigen::MatrixXd> Xv;
//		for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
//		char buf [1024];
//		sprintf(buf,"image%5.5i.png",imgcount++);
//		show(Xv,true,std::string(buf),imgcount);
//	}

//	Eigen::Matrix4d firstinv = poses.front().inverse();
//	for(int i = 0; i < nr_frames; i++){
//		poses[i] = firstinv*poses[i];
//	}
//if(nr_frames >= 3){exit(0);}
	printf("stop MassRegistrationPPRColor::getTransforms(std::vector<Eigen::Matrix4d> guess)\n");
    exit(0);
	//exit(0);
	return MassFusionResults(guess,-1);
}

}
