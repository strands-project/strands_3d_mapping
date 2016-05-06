#include "registration/MassRegistrationPPR.h"

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reglib
{

MassRegistrationPPR::MassRegistrationPPR(double startreg, bool visualize){
	type					= PointToPlane;
	//type					= PointToPoint;
	use_PPR_weight			= true;
	use_features			= true;
	normalize_matchweights	= true;

	DistanceWeightFunction2PPR2 * dwf = new DistanceWeightFunction2PPR2();
	dwf->update_size		= true;
	dwf->startreg			= startreg;
	dwf->debugg_print		= false;
	func					= dwf;
	
	fast_opt				= true;

	nomask = true;
	maskstep = 1;
	nomaskstep = 100000;

	stopval = 0.001;
	steps = 4;

	timeout = 6000;

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
MassRegistrationPPR::~MassRegistrationPPR(){
	for(unsigned int i = 0; i < arraypoints.size(); i++){delete[] arraypoints[i];}
	for(unsigned int i = 0; i < trees3d.size(); i++){delete trees3d[i];}
	for(unsigned int i = 0; i < a3dv.size(); i++){delete a3dv[i];}
	delete func;
}

//double getTime(){
//	struct timeval start1;
//	gettimeofday(&start1, NULL);
//	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
//}

void point_to_plane3(		Eigen::Matrix3Xd & X,
							Eigen::Matrix3Xd & Xn,
							Eigen::Matrix3Xd & Y,
							Eigen::Matrix3Xd & Yn,
							Eigen::VectorXd & W){
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;

	Matrix6d ATA;
	Vector6d ATb;
	ATA.setZero ();
	ATb.setZero ();

	Matrix6d ATA2;
	Vector6d ATb2;
	ATA2.setZero ();
	ATb2.setZero ();

	double dsum  = 0;
	unsigned int xcols = X.cols();
	for(unsigned int i=0; i < xcols; i++) {
		const double & sx = X(0,i);
		const double & sy = X(1,i);
		const double & sz = X(2,i);
		const double & dx = Y(0,i);
		const double & dy = Y(1,i);
		const double & dz = Y(2,i);
		const double & nx = Xn(0,i);
		const double & ny = Xn(1,i);
		const double & nz = Xn(2,i);
		const double & weight = W(i);

		double a = nz*sy - ny*sz;
		double b = nx*sz - nz*sx;
		double c = ny*sx - nx*sy;

		ATA.coeffRef (0) += weight * a * a;
		ATA.coeffRef (1) += weight * a * b;
		ATA.coeffRef (2) += weight * a * c;
		ATA.coeffRef (3) += weight * a * nx;
		ATA.coeffRef (4) += weight * a * ny;
		ATA.coeffRef (5) += weight * a * nz;
		ATA.coeffRef (7) += weight * b * b;
		ATA.coeffRef (8) += weight * b * c;
		ATA.coeffRef (9) += weight * b * nx;
		ATA.coeffRef (10) += weight * b * ny;
		ATA.coeffRef (11) += weight * b * nz;
		ATA.coeffRef (14) += weight * c * c;
		ATA.coeffRef (15) += weight * c * nx;
		ATA.coeffRef (16) += weight * c * ny;
		ATA.coeffRef (17) += weight * c * nz;
		ATA.coeffRef (21) += weight * nx * nx;
		ATA.coeffRef (22) += weight * nx * ny;
		ATA.coeffRef (23) += weight * nx * nz;
		ATA.coeffRef (28) += weight * ny * ny;
		ATA.coeffRef (29) += weight * ny * nz;
		ATA.coeffRef (35) += weight * nz * nz;

		double d = weight * (nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz);
		dsum += d;
		ATb.coeffRef (0) += a * d;
		ATb.coeffRef (1) += b * d;
		ATb.coeffRef (2) += c * d;
		ATb.coeffRef (3) += nx * d;
		ATb.coeffRef (4) += ny * d;
		ATb.coeffRef (5) += nz * d;
	}

	for(unsigned int i=0; i < xcols; i++) {
		const double & sx = Y(0,i);
		const double & sy = Y(1,i);
		const double & sz = Y(2,i);
		const double & dx = X(0,i);
		const double & dy = X(1,i);
		const double & dz = X(2,i);
		const double & nx = Yn(0,i);
		const double & ny = Yn(1,i);
		const double & nz = Yn(2,i);
		const double & weight = W(i);

		double a = nz*sy - ny*sz;
		double b = nx*sz - nz*sx;
		double c = ny*sx - nx*sy;

		ATA.coeffRef (0) += weight * a * a;
		ATA.coeffRef (1) += weight * a * b;
		ATA.coeffRef (2) += weight * a * c;
		ATA.coeffRef (3) += weight * a * nx;
		ATA.coeffRef (4) += weight * a * ny;
		ATA.coeffRef (5) += weight * a * nz;
		ATA.coeffRef (7) += weight * b * b;
		ATA.coeffRef (8) += weight * b * c;
		ATA.coeffRef (9) += weight * b * nx;
		ATA.coeffRef (10) += weight * b * ny;
		ATA.coeffRef (11) += weight * b * nz;
		ATA.coeffRef (14) += weight * c * c;
		ATA.coeffRef (15) += weight * c * nx;
		ATA.coeffRef (16) += weight * c * ny;
		ATA.coeffRef (17) += weight * c * nz;
		ATA.coeffRef (21) += weight * nx * nx;
		ATA.coeffRef (22) += weight * nx * ny;
		ATA.coeffRef (23) += weight * nx * nz;
		ATA.coeffRef (28) += weight * ny * ny;
		ATA.coeffRef (29) += weight * ny * nz;
		ATA.coeffRef (35) += weight * nz * nz;

		double d = weight * (nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz);
		dsum += d;
		ATb.coeffRef (0) -= a * d;
		ATb.coeffRef (1) -= b * d;
		ATb.coeffRef (2) -= c * d;
		ATb.coeffRef (3) -= nx * d;
		ATb.coeffRef (4) -= ny * d;
		ATb.coeffRef (5) -= nz * d;
	}



	ATA.coeffRef (6) = ATA.coeff (1);
	ATA.coeffRef (12) = ATA.coeff (2);
	ATA.coeffRef (13) = ATA.coeff (8);
	ATA.coeffRef (18) = ATA.coeff (3);
	ATA.coeffRef (19) = ATA.coeff (9);
	ATA.coeffRef (20) = ATA.coeff (15);
	ATA.coeffRef (24) = ATA.coeff (4);
	ATA.coeffRef (25) = ATA.coeff (10);
	ATA.coeffRef (26) = ATA.coeff (16);
	ATA.coeffRef (27) = ATA.coeff (22);
	ATA.coeffRef (30) = ATA.coeff (5);
	ATA.coeffRef (31) = ATA.coeff (11);
	ATA.coeffRef (32) = ATA.coeff (17);
	ATA.coeffRef (33) = ATA.coeff (23);
	ATA.coeffRef (34) = ATA.coeff (29);

	// Solve A*x = b
	Vector6d x = static_cast<Vector6d> (ATA.inverse () * ATb);
	Eigen::Affine3d transformation = Eigen::Affine3d(constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0)));

	X = transformation*X;
	transformation(0,3) = 0;
	transformation(1,3) = 0;
	transformation(2,3) = 0;
	Xn = transformation*Xn;
	//std::cout << x << std::endl;

	//// Construct the transformation matrix from x
	//constructTransformationMatrix (x (0), x (1), x (2), x (3), x (4), x (5), transformation_matrix);
}

bool okVal(double v){return !std::isnan(v) && !(v == std::numeric_limits<double>::infinity());}

bool isValidPoint(pcl::PointXYZRGBNormal p){
	return	!okVal (p.x)		&& !okVal (p.y)			&& !okVal (p.z) &&			//No nans or inf in position
			!okVal (p.normal_x) && !okVal (p.normal_y)	&& !okVal (p.normal_z) &&	//No nans or inf in normal
			!(p.x == 0			&& p.y == 0				&& p.z == 0 ) &&						//not a zero point
			!(p.normal_x == 0	&& p.normal_y == 0		&& p.normal_z == 0);					//not a zero normal
}


void MassRegistrationPPR::setData(std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > all_clouds){
	unsigned int nr_frames = all_clouds.size();

	if(arraypoints.size() > 0){
		for(unsigned int i = 0; i < arraypoints.size(); i++){
			delete[] arraypoints[i];
		}
	}

	if(a3dv.size() > 0){
		for(unsigned int i = 0; i < a3dv.size(); i++){
			delete a3dv[i];
		}
	}

	if(trees3d.size() > 0){
		for(unsigned int i = 0; i < trees3d.size(); i++){
			delete trees3d[i];
		}
	}

	nr_matches.resize(nr_frames);
	matchids.resize(nr_frames);
	nr_datas.resize(nr_frames);
	points.resize(nr_frames);
	colors.resize(nr_frames);
	normals.resize(nr_frames);
	transformed_points.resize(nr_frames);
	transformed_normals.resize(nr_frames);
	informations.resize(nr_frames);

	nr_arraypoints.resize(nr_frames);
	arraypoints.resize(nr_frames);
	trees3d.resize(nr_frames);
	a3dv.resize(nr_frames);
	is_ok.resize(nr_frames);

	for(unsigned int i = 0; i < nr_frames; i++){
		printf("loading data for %i\n",i);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = all_clouds[i];
		int count = 0;
		for(unsigned int i = 0; i < cloud->points.size(); i++){
			if (isValidPoint(cloud->points[i])){
				count++;
			}
		}

		if(count < 10){
			is_ok[i] = false;
			continue;
		}else{
			is_ok[i] = true;
		}

		double * ap = new double[3*count];
		nr_datas[i] = count;
		matchids[i].resize(nr_frames);
		points[i].resize(Eigen::NoChange,count);
		colors[i].resize(Eigen::NoChange,count);
		normals[i].resize(Eigen::NoChange,count);
		transformed_points[i].resize(Eigen::NoChange,count);
		transformed_normals[i].resize(Eigen::NoChange,count);

		nr_arraypoints[i] = count;
		arraypoints[i] = ap;

		Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & C	= colors[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= normals[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= transformed_points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= transformed_normals[i];
		Eigen::VectorXd information (count);

		int c = 0;
		for(unsigned int i = 0; i < cloud->points.size(); i++){
			pcl::PointXYZRGBNormal p = cloud->points[i];
			if (isValidPoint(p)){

				float xn = p.normal_x;
				float yn = p.normal_y;
				float zn = p.normal_z;

				float x = p.x;
				float y = p.y;
				float z = p.z;

				ap[3*c+0] =x;
				ap[3*c+1] =y;
				ap[3*c+2] =z;

				X(0,c)	= x;
				X(1,c)	= y;
				X(2,c)	= z;
				Xn(0,c)	= xn;
				Xn(1,c)	= yn;
				Xn(2,c)	= zn;

				information(c) = 1.0/(z*z);
				C(0,c) = p.r;
				C(1,c) = p.g;
				C(2,c) = p.b;
				c++;

			}
		}

		informations[i] = information;

		ArrayData3D<double> * a3d = new ArrayData3D<double>;
		a3d->data	= ap;
		a3d->rows	= count;
		a3dv[i]		= a3d;
		trees3d[i]	= new Tree3d(3, *a3d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
		trees3d[i]->buildIndex();
	}
}

void MassRegistrationPPR::setData(std::vector<RGBDFrame*> frames_,std::vector<ModelMask *> mmasks_){
	frames = frames_;
	mmasks = mmasks_;

	unsigned int nr_frames = frames.size();

	if(arraypoints.size() > 0){
		for(unsigned int i = 0; i < arraypoints.size(); i++){
			delete[] arraypoints[i];
		}
	}

	if(a3dv.size() > 0){
		for(unsigned int i = 0; i < a3dv.size(); i++){
			delete a3dv[i];
		}
	}

	if(trees3d.size() > 0){
		for(unsigned int i = 0; i < trees3d.size(); i++){
			delete trees3d[i];
		}
	}

	nr_matches.resize(nr_frames);
	matchids.resize(nr_frames);
	nr_datas.resize(nr_frames);
	points.resize(nr_frames);
	colors.resize(nr_frames);
	normals.resize(nr_frames);
	transformed_points.resize(nr_frames);
	transformed_normals.resize(nr_frames);
	informations.resize(nr_frames);

	nr_arraypoints.resize(nr_frames);
	arraypoints.resize(nr_frames);
	trees3d.resize(nr_frames);
	a3dv.resize(nr_frames);
	is_ok.resize(nr_frames);

	for(unsigned int i = 0; i < nr_frames; i++){
		printf("loading data for %i\n",i);
		bool * maskvec		= mmasks[i]->maskvec;
		unsigned char  * rgbdata		= (unsigned char	*)(frames[i]->rgb.data);
		unsigned short * depthdata		= (unsigned short	*)(frames[i]->depth.data);
		float		   * normalsdata	= (float			*)(frames[i]->normals.data);

//		Eigen::Matrix4d p = poses[i];
//		float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
//		float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
//		float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

		Camera * camera				= frames[i]->camera;
		const unsigned int width	= camera->width;
		const unsigned int height	= camera->height;
		const float idepth			= camera->idepth_scale;
		const float cx				= camera->cx;
		const float cy				= camera->cy;
		const float ifx				= 1.0/camera->fx;
		const float ify				= 1.0/camera->fy;


		int count1 = 0;
		for(unsigned int w = 0; w < width; w++){
			for(unsigned int h = 0; h < height; h++){
				int ind = h*width+w;
				if(maskvec[ind]){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];
					if(z > 0.2 && xn != 2){count1++;}
				}
			}
		}

		int count = 0;
		for(unsigned int w = 0; w < width; w+=maskstep){
			for(unsigned int h = 0; h < height; h+=maskstep){
				int ind = h*width+w;
				if(maskvec[ind]){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];
					if(z > 0.2 && xn != 2){count++;}
				}
			}
		}

		if(count < 10){
			is_ok[i] = false;
			continue;
		}else{
			is_ok[i] = true;
		}

		double * ap = new double[3*count];
		nr_datas[i] = count;
		matchids[i].resize(nr_frames);
		points[i].resize(Eigen::NoChange,count);
		colors[i].resize(Eigen::NoChange,count);
		normals[i].resize(Eigen::NoChange,count);
		transformed_points[i].resize(Eigen::NoChange,count);
		transformed_normals[i].resize(Eigen::NoChange,count);

		nr_arraypoints[i] = count;
		arraypoints[i] = ap;

		Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & C	= colors[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= normals[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= transformed_points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= transformed_normals[i];
		Eigen::VectorXd information (count);

		int c = 0;
		for(unsigned int w = 0; w < width; w+=maskstep){
			for(unsigned int h = 0; h < height;h+=maskstep){
				if(c == count){continue;}
				int ind = h*width+w;
				if(maskvec[ind]){
					float z = idepth*float(depthdata[ind]);
					float xn = normalsdata[3*ind+0];

					if(z > 0.2 && xn != 2){
						float yn = normalsdata[3*ind+1];
						float zn = normalsdata[3*ind+2];

						float x = (w - cx) * z * ifx;
						float y = (h - cy) * z * ify;
						ap[3*c+0] =x;
						ap[3*c+1] =y;
						ap[3*c+2] =z;

						X(0,c)	= x;
						X(1,c)	= y;
						X(2,c)	= z;
						Xn(0,c)	= xn;
						Xn(1,c)	= yn;
						Xn(2,c)	= zn;

						information(c) = 1.0/(z*z);
						C(0,c) = rgbdata[3*ind+0];
						C(1,c) = rgbdata[3*ind+1];
						C(2,c) = rgbdata[3*ind+2];
						c++;
					}
				}
			}
		}
		informations[i] = information;

		ArrayData3D<double> * a3d = new ArrayData3D<double>;
		a3d->data	= ap;
		a3d->rows	= count;
		a3dv[i]		= a3d;
		trees3d[i]	= new Tree3d(3, *a3d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
		trees3d[i]->buildIndex();
	}
}

int testcount = 0;
MassFusionResults MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses){
	printf("start MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> poses)\n");

	unsigned int nr_frames = informations.size();
	if(poses.size() != nr_frames){
		printf("ERROR: poses.size() != informations.size()\n");
		return MassFusionResults();
	}
	bool internatldebug = testcount++ > 2;

	fast_opt = false;
	if(fast_opt){
		printf("debugging... setting nr frames to 3: %s :: %i\n",__FILE__,__LINE__);
		nr_frames = 3;
	}

	for(unsigned int i = 0; i < nr_frames; i++){
		printf("loading data for %i\n",i);
		Eigen::Matrix4d p = poses[i];
		float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
		float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
		float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

		if(!is_ok[i]){continue;}

		Eigen::Matrix<double, 3, Eigen::Dynamic> & X	= points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn	= normals[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tX	= transformed_points[i];
		Eigen::Matrix<double, 3, Eigen::Dynamic> & tXn	= transformed_normals[i];
		int count = nr_datas[i];
		for(int c = 0; c < count; c++){
			float x = X(0,c);
			float y = X(1,c);
			float z = X(2,c);
			float xn = Xn(0,c);
			float yn = Xn(1,c);
			float zn = Xn(2,c);

			tX(0,c)		= m00*x + m01*y + m02*z + m03;
			tX(1,c)		= m10*x + m11*y + m12*z + m13;
			tX(2,c)		= m20*x + m21*y + m22*z + m23;
			tXn(0,c)	= m00*xn + m01*yn + m02*zn;
			tXn(1,c)	= m10*xn + m11*yn + m12*zn;
			tXn(2,c)	= m20*xn + m21*yn + m22*zn;
		}
	}

	func->reset();

	Eigen::MatrixXd Xo1;

	int imgcount = 0;

	double good_rematches = 0;
	double total_rematches = 0;

	double good_opt = 0;
	double bad_opt = 0;

	double rematch_time = 0;
	double residuals_time = 0;
	double computeModel_time = 0;
	double setup_matches_time = 0;
	double setup_equation_time = 0;
	double setup_equation_time2 = 0;
	double solve_equation_time = 0;
	double total_time_start = getTime();

	int savecounter = 0;

	bool onetoone = true;

	char buf [1024];
	if(visualizationLvl > 0){
		std::vector<Eigen::MatrixXd> Xv;
		for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
		sprintf(buf,"image%5.5i.png",imgcount++);
		show(Xv,false,std::string(buf),imgcount);
	}

	for(int funcupdate=0; funcupdate < 100; ++funcupdate) {
		if(getTime()-total_time_start > timeout){break;}
		if(visualizationLvl == 2){std::vector<Eigen::MatrixXd> Xv;for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}sprintf(buf,"image%5.5i.png",imgcount++);show(Xv,false,std::string(buf),imgcount);}

		for(int rematching=0; rematching < 10; ++rematching) {
			if(visualizationLvl == 3){std::vector<Eigen::MatrixXd> Xv;for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}sprintf(buf,"image%5.5i.png",imgcount++);show(Xv,false,std::string(buf),imgcount);}
			std::vector<Eigen::Matrix4d> poses1 = poses;
			double rematch_time_start = getTime();

			double new_good_rematches = 0;
			double new_total_rematches = 0;
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				nr_matches[i] = 0;

				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					if(i == j){continue;}
					Eigen::Affine3d rp = Eigen::Affine3d(poses[j].inverse()*poses[i]);
					Eigen::Matrix<double, 3, Eigen::Dynamic> tX	= rp*points[i];

					unsigned int nr_data = nr_datas[i];
					std::vector<int> & matchid = matchids[i][j];
					matchid.resize(nr_data);
					Tree3d * t3d = trees3d[j];

					for(unsigned int k = 0; k < nr_data; ++k) {
						int prev = matchid[k];
						double * qp = tX.col(k).data();
						size_t ret_index;
						double out_dist_sqr;
						nanoflann::KNNResultSet<double> resultSet(1);
						resultSet.init(&ret_index, &out_dist_sqr );
						t3d->findNeighbors(resultSet, qp, nanoflann::SearchParams(10));
						int current = ret_index;
						new_good_rematches += prev != current;
						new_total_rematches++;
						matchid[k] = current;
					}
					nr_matches[i] += matchid.size();
				}
			}

			good_rematches += new_good_rematches;
			total_rematches += new_total_rematches;

			rematch_time += getTime()-rematch_time_start;
		//	printf("rematch_time: %f\n",rematch_time);
		//	printf("new percentage: %5.5f (good_rematches: %f total_rematches: %f)\n",new_good_rematches/new_total_rematches,new_good_rematches,new_total_rematches);
		//	printf("tot percentage: %5.5f (good_rematches: %f total_rematches: %f)\n",good_rematches/total_rematches,good_rematches,total_rematches);
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				nr_matches[i] = 0;
				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					nr_matches[i] += matchids[i][j].size()+matchids[j][i].size();
				}
			}

			int total_matches = 0;
			for(unsigned int i = 0; i < nr_frames; i++){
				if(!is_ok[i]){continue;}
				for(unsigned int j = 0; j < nr_frames; j++){
					if(!is_ok[j]){continue;}
					total_matches += matchids[i][j].size();
				}
			}

			for(int lala = 0; lala < 1; lala++){
				if(visualizationLvl == 4){std::vector<Eigen::MatrixXd> Xv;for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}sprintf(buf,"image%5.5i.png",imgcount++);show(Xv,false,std::string(buf),imgcount);}
				std::vector<Eigen::Matrix4d> poses2b = poses;
				double residuals_time_start = getTime();
				Eigen::MatrixXd all_residuals;
				switch(type) {
					case PointToPoint:	{all_residuals = Eigen::Matrix3Xd::Zero(3,total_matches);}break;
					case PointToPlane:	{all_residuals = Eigen::MatrixXd::Zero(1,total_matches);}break;
					default:			{printf("type not set\n");}					break;
				}

				int count = 0;
				for(unsigned int i = 0; i < nr_frames; i++){
					if(!is_ok[i]){continue;}
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
					Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
					Eigen::VectorXd & informationi					= informations[i];
					for(unsigned int j = 0; j < nr_frames; j++){
						if(!is_ok[j]){continue;}
						if(i == j){continue;}
						std::vector<int> & matchidi = matchids[i][j];
						unsigned int matchesi = matchidi.size();
						Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
						Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
						Eigen::VectorXd & informationj					= informations[j];
						Eigen::Matrix3Xd Xp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
						Eigen::Matrix3Xd Xn		= Eigen::Matrix3Xd::Zero(3,	matchesi);
						Eigen::Matrix3Xd Qp		= Eigen::Matrix3Xd::Zero(3,	matchesi);
						Eigen::Matrix3Xd Qn		= Eigen::Matrix3Xd::Zero(3,	matchesi);
						Eigen::VectorXd  rangeW	= Eigen::VectorXd::Zero(	matchesi);

						for(unsigned int ki = 0; ki < matchesi; ki++){
							int kj = matchidi[ki];
							if( ki >= Qp.cols() || kj < 0 || kj >= tXj.cols() ){continue;}
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
				func->computeModel(all_residuals);
				computeModel_time += getTime()-computeModel_time_start;

				if(fast_opt){
					double setup_matches_time_start = getTime();

					std::vector< Eigen::Matrix4d > localposes = poses;
					std::vector< std::vector < std::vector< std::pair<int,int	> > > > current_matches;
					std::vector< std::vector < std::vector<			double		  > > > rangeW;

					current_matches.resize(nr_frames);
					rangeW.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						current_matches[i].resize(nr_frames);
						rangeW[i].resize(nr_frames);
						if(!is_ok[i]){continue;}

						Eigen::VectorXd & informationi					= informations[i];
						for(unsigned int j = 0; j < nr_frames; j++){
							if(!is_ok[j]){continue;}
							if(i == j){continue;}
							Eigen::VectorXd & informationj					= informations[j];

							std::vector<int> & matchidj = matchids[j][i];
							unsigned int matchesj = matchidj.size();
							std::vector<int> & matchidi = matchids[i][j];
							unsigned int matchesi = matchidi.size();

							std::vector<std::pair<int,int> > & cm = current_matches[i][j];
							std::vector<double > & rw = rangeW[i][j];

							for(unsigned int ki = 0; ki < matchesi; ki++){
								int kj = matchidi[ki];
								if( kj < 0 || kj >=  matchesj){continue;} //Make sure that failed searches dont screw things up
								if(matchidj[kj] != ki){continue;}//Only 1-to-1 matching

								cm.push_back(std::make_pair(ki,kj));
								rw.push_back(1.0/(1.0/informationi(ki)+1.0/informationj(kj)));
							}
						}
					}

					setup_matches_time += getTime()-setup_matches_time_start;

					typedef Eigen::Matrix<double, 6, 1> Vector6d;
					typedef Eigen::Matrix<double, 6, 6> Matrix6d;

					std::vector<std::vector<Matrix6d>> A;
					std::vector<std::vector<Vector6d>> b;
					A.resize(nr_frames);
					b.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						A[i].resize(nr_frames);
						b[i].resize(nr_frames);
					}

					std::vector<std::vector<Matrix6d>> A2;
					std::vector<std::vector<Vector6d>> b2;
					A2.resize(nr_frames);
					b2.resize(nr_frames);
					for(unsigned int i = 0; i < nr_frames; i++){
						A2[i].resize(nr_frames);
						b2[i].resize(nr_frames);
						for(unsigned int j = 0; j < nr_frames; j++){
							Matrix6d & ATA = A[i][j];
							Vector6d & ATb = b[i][j];
							ATA.setZero ();
							ATb.setZero ();
						}
					}

					for(int iteration = 0; iteration < 5; iteration++){
						printf("iteration: %i\n",iteration);

						double total_score = 0;

						double setup_equation_time_start = getTime();
						for(unsigned int i = 0; i < nr_frames; i++){
							if(!is_ok[i]){continue;}
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
								if(i == j){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];

								std::vector<std::pair<int,int> > & cm = current_matches[i][j];
								std::vector<double > & rw = rangeW[i][j];
								unsigned int current_nr_matches = cm.size();

								Matrix6d & ATA = A[i][j];
								Vector6d & ATb = b[i][j];
								ATA.setZero ();
								ATb.setZero ();

								//Matches from ki to kj
								for(unsigned int k = 0; k < current_nr_matches; k++){
									unsigned int ki = cm[k].first;
									unsigned int kj = cm[k].second;
									double rwij = rw[k];

									const float & sx = tXi(0,ki);
									const float & sy = tXi(1,ki);
									const float & sz = tXi(2,ki);

									const float & dx = tXj(0,kj);
									const float & dy = tXj(1,kj);
									const float & dz = tXj(2,kj);

									const float & nx = tXnj(0,kj);
									const float & ny = tXnj(1,kj);
									const float & nz = tXnj(2,kj);


									double a = nz*sy - ny*sz;
									double b = nx*sz - nz*sx;
									double c = ny*sx - nx*sy;

									//    0  1  2  3  4  5
									//    6  7  8  9 10 11
									//   12 13 14 15 16 17
									//   18 19 20 21 22 23
									//   24 25 26 27 28 29
									//   30 31 32 33 34 35

									ATA.coeffRef (0) += a * a;
									ATA.coeffRef (1) += a * b;
									ATA.coeffRef (2) += a * c;
									ATA.coeffRef (3) += a * nx;
									ATA.coeffRef (4) += a * ny;
									ATA.coeffRef (5) += a * nz;
									ATA.coeffRef (7) += b * b;
									ATA.coeffRef (8) += b * c;
									ATA.coeffRef (9) += b * nx;
									ATA.coeffRef (10) += b * ny;
									ATA.coeffRef (11) += b * nz;
									ATA.coeffRef (14) += c * c;
									ATA.coeffRef (15) += c * nx;
									ATA.coeffRef (16) += c * ny;
									ATA.coeffRef (17) += c * nz;
									ATA.coeffRef (21) += nx * nx;
									ATA.coeffRef (22) += nx * ny;
									ATA.coeffRef (23) += nx * nz;
									ATA.coeffRef (28) += ny * ny;
									ATA.coeffRef (29) += ny * nz;
									ATA.coeffRef (35) += nz * nz;

									double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
									total_score += d*d;
									ATb.coeffRef (0) += a * d;
									ATb.coeffRef (1) += b * d;
									ATb.coeffRef (2) += c * d;
									ATb.coeffRef (3) += nx * d;
									ATb.coeffRef (4) += ny * d;
									ATb.coeffRef (5) += nz * d;

								}
								ATA.coeffRef (6) = ATA.coeff (1);
								ATA.coeffRef (12) = ATA.coeff (2);
								ATA.coeffRef (13) = ATA.coeff (8);
								ATA.coeffRef (18) = ATA.coeff (3);
								ATA.coeffRef (19) = ATA.coeff (9);
								ATA.coeffRef (20) = ATA.coeff (15);
								ATA.coeffRef (24) = ATA.coeff (4);
								ATA.coeffRef (25) = ATA.coeff (10);
								ATA.coeffRef (26) = ATA.coeff (16);
								ATA.coeffRef (27) = ATA.coeff (22);
								ATA.coeffRef (30) = ATA.coeff (5);
								ATA.coeffRef (31) = ATA.coeff (11);
								ATA.coeffRef (32) = ATA.coeff (17);
								ATA.coeffRef (33) = ATA.coeff (23);
								ATA.coeffRef (34) = ATA.coeff (29);
							}
						}
						setup_equation_time += getTime()-setup_equation_time_start;

						double setup_equation_time_start2 = getTime();
						for(unsigned int i = 0; i < nr_frames; i++){
							if(!is_ok[i]){continue;}
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXi	= transformed_points[i];
							Eigen::Matrix<double, 3, Eigen::Dynamic> & tXni	= transformed_normals[i];
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
								if(i == j){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];

								std::vector<std::pair<int,int> > & cm = current_matches[i][j];
								std::vector<double > & rw = rangeW[i][j];
								unsigned int current_nr_matches = cm.size();

								Matrix6d & ATA = A2[i][j];
								Vector6d & ATb = b2[i][j];
								ATA.setZero ();
								ATb.setZero ();

								//Matches from ki to kj
								for(unsigned int k = 0; k < current_nr_matches; k++){
									unsigned int ki = cm[k].first;
									unsigned int kj = cm[k].second;
									double rwij = rw[k];



									const float & sx = tXi(0,ki);
									const float & sy = tXi(1,ki);
									const float & sz = tXi(2,ki);

									const float & snx = tXni(0,ki);
									const float & sny = tXni(1,ki);
									const float & snz = tXni(2,ki);

									const float & dx = tXj(0,kj);
									const float & dy = tXj(1,kj);
									const float & dz = tXj(2,kj);

									const float & nx = tXnj(0,kj);
									const float & ny = tXnj(1,kj);
									const float & nz = tXnj(2,kj);


									double a = nz*sy - ny*sz;
									double b = nx*sz - nz*sx;
									double c = ny*sx - nx*sy;


									double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;

									double angle = nx*snx + ny*sny + nz*snz;
									if(angle < 0){continue;}

									double prob = func->getProb(d);
									double weight = prob*rwij;
									//    0  1  2  3  4  5
									//    6  7  8  9 10 11
									//   12 13 14 15 16 17
									//   18 19 20 21 22 23
									//   24 25 26 27 28 29
									//   30 31 32 33 34 35

									ATA.coeffRef (0) += weight * a * a;
									ATA.coeffRef (1) += weight * a * b;
									ATA.coeffRef (2) += weight * a * c;
									ATA.coeffRef (3) += weight * a * nx;
									ATA.coeffRef (4) += weight * a * ny;
									ATA.coeffRef (5) += weight * a * nz;
									ATA.coeffRef (7) += weight * b * b;
									ATA.coeffRef (8) += weight * b * c;
									ATA.coeffRef (9) += weight * b * nx;
									ATA.coeffRef (10) += weight * b * ny;
									ATA.coeffRef (11) += weight * b * nz;
									ATA.coeffRef (14) += weight * c * c;
									ATA.coeffRef (15) += weight * c * nx;
									ATA.coeffRef (16) += weight * c * ny;
									ATA.coeffRef (17) += weight * c * nz;
									ATA.coeffRef (21) += weight * nx * nx;
									ATA.coeffRef (22) += weight * nx * ny;
									ATA.coeffRef (23) += weight * nx * nz;
									ATA.coeffRef (28) += weight * ny * ny;
									ATA.coeffRef (29) += weight * ny * nz;
									ATA.coeffRef (35) += weight * nz * nz;

									ATb.coeffRef (0) += weight * a * d;
									ATb.coeffRef (1) += weight * b * d;
									ATb.coeffRef (2) += weight * c * d;
									ATb.coeffRef (3) += weight * nx * d;
									ATb.coeffRef (4) += weight * ny * d;
									ATb.coeffRef (5) += weight * nz * d;

								}
								ATA.coeffRef (6) = ATA.coeff (1);
								ATA.coeffRef (12) = ATA.coeff (2);
								ATA.coeffRef (13) = ATA.coeff (8);
								ATA.coeffRef (18) = ATA.coeff (3);
								ATA.coeffRef (19) = ATA.coeff (9);
								ATA.coeffRef (20) = ATA.coeff (15);
								ATA.coeffRef (24) = ATA.coeff (4);
								ATA.coeffRef (25) = ATA.coeff (10);
								ATA.coeffRef (26) = ATA.coeff (16);
								ATA.coeffRef (27) = ATA.coeff (22);
								ATA.coeffRef (30) = ATA.coeff (5);
								ATA.coeffRef (31) = ATA.coeff (11);
								ATA.coeffRef (32) = ATA.coeff (17);
								ATA.coeffRef (33) = ATA.coeff (23);
								ATA.coeffRef (34) = ATA.coeff (29);
							}
						}
						setup_equation_time2 += getTime()-setup_equation_time_start2;

						printf("total_score: %f\n",total_score);

						if(false){
							Eigen::Matrix4d p0inv = poses[0].inverse();
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
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

									tXi(0,c)		= m00*x + m01*y + m02*z + m03;
									tXi(1,c)		= m10*x + m11*y + m12*z + m13;
									tXi(2,c)		= m20*x + m21*y + m22*z + m23;

									tXni(0,c)		= m00*nx + m01*ny + m02*nz;
									tXni(1,c)		= m10*nx + m11*ny + m12*nz;
									tXni(2,c)		= m20*nx + m21*ny + m22*nz;
								}
							}

							std::vector<Eigen::MatrixXd> Xv;
							for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
							sprintf(buf,"image%5.5i.png",imgcount++);
							show(Xv,false,std::string(buf),imgcount);
						}

						double solve_equation_time_start = getTime();
						Eigen::MatrixXd fullA = Eigen::MatrixXd::Identity (6 * (nr_frames - 1), 6 * (nr_frames - 1));
						fullA *= 0.00001;
						Eigen::VectorXd fullB = Eigen::VectorXd::Zero (6 * (nr_frames - 1));

						for(unsigned int i = 0; i < nr_frames; i++){
							for(unsigned int j = 0; j == 0 && j < nr_frames; j++){
								if(i == j){continue;}
								Matrix6d & ATA = A[i][j];
								Vector6d & ATb = b[i][j];
								Vector6d x = ATA.inverse () * ATb;
								//printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
								//								Eigen::Matrix4d m = constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
								//								std::cout << m << std::endl << std::endl;
								//								printf("%i %i ->%i %i\n",i,j,6*(i-1), 6*j);
								if (i > 0){
									//fullA.block (6*(i-1), 6*j,6,6) -= ATA;
									fullA.block (6*(i-1),6*(i-1),6,6) += ATA;
									fullB.segment (6*(i-1), 6) += ATb;
								}
								//G.block (6 * (vi - 1), 6 * (vi - 1), 6, 6) += (*slam_graph_)[e].cinv_;
								//B.segment (6 * (vi - 1), 6) += (present1 ? 1 : -1) * (*slam_graph_)[e].cinvd_;
							}
						}
						//						std::cout << fullA << std::endl << std::endl;
						//						std::cout << fullB << std::endl << std::endl;
						//printf("--------------------------------------------------\n");
						Eigen::VectorXd fullX = fullA.inverse()*fullB;
						for(unsigned int i = 0; i < nr_frames; i++){
							Eigen::Matrix4d m;
							if(i == 0){
								m = Eigen::Matrix4d::Identity();
							}else{
								m = constructTransformationMatrix(fullX(6*(i-1)+0),fullX(6*(i-1)+1),fullX(6*(i-1)+2),fullX(6*(i-1)+3),fullX(6*(i-1)+4),fullX(6*(i-1)+5));
							}
							//std::cout << m << std::endl << std::endl;
						}

						//					    // Start at 1 because 0 is the reference pose
						//					    for (int vi = 1; vi != n; ++vi)
						//					    {
						//					      for (int vj = 0; vj != n; ++vj)
						//					      {
						//					        // Attempt to use the forward edge, otherwise use backward edge, otherwise there was no edge
						//					        Edge e;
						//					        bool present1, present2;
						//					        boost::tuples::tie (e, present1) = edge (vi, vj, *slam_graph_);
						//					        if (!present1)
						//					        {
						//					          boost::tuples::tie (e, present2) = edge (vj, vi, *slam_graph_);
						//					          if (!present2)
						//					            continue;
						//					        }

						//					        // Fill in elements of G and B
						//					        if (vj > 0)
						//					          G.block (6 * (vi - 1), 6 * (vj - 1), 6, 6) = -(*slam_graph_)[e].cinv_;
						//					        G.block (6 * (vi - 1), 6 * (vi - 1), 6, 6) += (*slam_graph_)[e].cinv_;
						//					        B.segment (6 * (vi - 1), 6) += (present1 ? 1 : -1) * (*slam_graph_)[e].cinvd_;
						//					      }
						//					    }

						//					    // Computation of the linear equation system: GX = B
						//					    // TODO investigate accuracy vs. speed tradeoff and find the best solving method for our type of linear equation (sparse)
						//					    Eigen::VectorXf X = G.colPivHouseholderQr ().solve (B);

						//					    // Update the poses
						//					    float sum = 0.0;
						//					    for (int vi = 1; vi != n; ++vi)
						//					    {
						//					      Eigen::Vector6f difference_pose = static_cast<Eigen::Vector6f> (-incidenceCorrection (getPose (vi)).inverse () * X.segment (6 * (vi - 1), 6));
						//					      sum += difference_pose.norm ();
						//					      setPose (vi, getPose (vi) + difference_pose);
						//					    }

						//						for(unsigned int i = 0; i < nr_frames; i++){
						//							for(unsigned int j = 0; j < nr_frames; j++){
						//								if(i == j){continue;}
						//								Matrix6d & ATA = A[i][j];
						//								Vector6d & ATb = b[i][j];
						//								Vector6d x = ATA.inverse () * ATb;
						//								printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));

						//								Matrix6d & ATA2 = A2[i][j];
						//								Vector6d & ATb2 = b2[i][j];
						//								Vector6d x2 = ATA2.inverse () * ATb2;
						//								printf("%i %i x: %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n",i,j,x2(0,0),x2(1,0),x2(2,0),x2(3,0),x2(4,0),x2(5,0));

						//								Eigen::Matrix4d m = constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0));
						//								std::cout << m << std::endl << std::endl;
						//							}
						//						}

						solve_equation_time += getTime()-solve_equation_time_start;

						if(false){
							Eigen::Matrix4d p0inv = poses[0].inverse();
							for(unsigned int j = 0; j < nr_frames; j++){
								if(!is_ok[j]){continue;}
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

									tXi(0,c)		= m00*x + m01*y + m02*z + m03;
									tXi(1,c)		= m10*x + m11*y + m12*z + m13;
									tXi(2,c)		= m20*x + m21*y + m22*z + m23;

									tXni(0,c)		= m00*nx + m01*ny + m02*nz;
									tXni(1,c)		= m10*nx + m11*ny + m12*nz;
									tXni(2,c)		= m20*nx + m21*ny + m22*nz;
								}
							}

							std::vector<Eigen::MatrixXd> Xv;
							for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
							sprintf(buf,"image%5.5i.png",imgcount++);
							show(Xv,false,std::string(buf),imgcount);
						}
						//Recover poses from solution
						//Recompute points

						//if(isconverged(poses, localposes, stopval, stopval)){break;}
					}

					printf("total_time:           %5.5f\n",getTime()-total_time_start);
					printf("rematch_time:         %5.5f\n",rematch_time);
					printf("computeModel:         %5.5f\n",computeModel_time);
					printf("setup_matches_time:   %5.5f\n",setup_matches_time);
					printf("setup_equation_time:  %5.5f\n",setup_equation_time);
					printf("setup_equation_time2: %5.5f\n",setup_equation_time2);
					printf("solve_equation_time:  %5.5f\n",solve_equation_time);
					exit(0);
				}else{
					for(int outer=0; outer < 30; ++outer) {
						if(getTime()-total_time_start > timeout){break;}

						printf("funcupdate: %i rematching: %i lala: %i outer: %i\n",funcupdate,rematching,lala,outer);
						std::vector<Eigen::Matrix4d> poses2 = poses;
						for(unsigned int i = 0; i < nr_frames; i++){
							if(getTime()-total_time_start > timeout){break;}
							if(!is_ok[i]){continue;}
							unsigned int nr_match = 0;
							{
								for(unsigned int j = 0; j < nr_frames; j++){
									if(!is_ok[j]){continue;}
									std::vector<int> & matchidj = matchids[j][i];
									unsigned int matchesj = matchidj.size();
									std::vector<int> & matchidi = matchids[i][j];
									unsigned int matchesi = matchidi.size();

									for(unsigned int ki = 0; ki < matchesi; ki++){
										int kj = matchidi[ki];
										if( kj == -1 ){continue;}
										if( kj >=  matchesj){continue;}
										if(!onetoone || matchidj[kj] == ki){	nr_match++;}
									}
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
								if(!is_ok[j]){continue;}
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXj	= transformed_points[j];
								Eigen::Matrix<double, 3, Eigen::Dynamic> & tXnj	= transformed_normals[j];
								Eigen::VectorXd & informationj					= informations[j];

								std::vector<int> & matchidj = matchids[j][i];
								unsigned int matchesj = matchidj.size();
								std::vector<int> & matchidi = matchids[i][j];
								unsigned int matchesi = matchidi.size();

								for(unsigned int ki = 0; ki < matchesi; ki++){
									int kj = matchidi[ki];
									if( kj == -1 ){continue;}
									if( kj >=  matchesj){continue;}
									if(!onetoone || matchidj[kj] == ki){
										Qp.col(count) = tXj.col(kj);
										Qn.col(count) = tXnj.col(kj);

										Xp_ori.col(count) = Xi.col(ki);
										Xp.col(count) = tXi.col(ki);

										Xn.col(count) = tXni.col(ki);
										rangeW(count) = 1.0/(1.0/informationi(ki)+1.0/informationj(kj));
										count++;
									}
								}
							}

							if(count == 0){break;}

							//showMatches(Xp,Qp);
							for(int inner=0; inner < 5; ++inner) {
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


								for(int k=0; k<nr_match; ++k) {
									if(W(k) > 0.1){good_opt++;}
									else{bad_opt++;}
								}

								W = W.array()*rangeW.array()*rangeW.array();
								Xo1 = Xp;
								switch(type) {
									case PointToPoint:	{
										//RigidMotionEstimator::point_to_point(Xp, Qp, W);
										pcl::TransformationFromCorrespondences tfc1;
										for(unsigned int c = 0; c < nr_match; c++){tfc1.add(Eigen::Vector3f(Xp(0,c), Xp(1,c),Xp(2,c)),Eigen::Vector3f(Qp(0,c),Qp(1,c),Qp(2,c)),W(c));}
										Eigen::Affine3d rot = tfc1.getTransformation().cast<double>();
										Xp = rot*Xp;
										Xn = rot.rotation()*Xn;
									}		break;
									case PointToPlane:	{
										point_to_plane2(Xp, Xn, Qp, Qn, W);
									}	break;
								default:  			{printf("type not set\n"); } break;
								}

								double stop1 = (Xp-Xo1).colwise().norm().maxCoeff();
								Xo1 = Xp;
								if(stop1 < 0.001){break; }
							}
							//exit(0);
							pcl::TransformationFromCorrespondences tfc;
							for(unsigned int c = 0; c < nr_match; c++){tfc.add(Eigen::Vector3f(Xp_ori(0,c),Xp_ori(1,c),Xp_ori(2,c)),Eigen::Vector3f(Xp(0,c),Xp(1,c),Xp(2,c)));}
							poses[i] = tfc.getTransformation().cast<double>().matrix();
						}

						Eigen::Matrix4d p0inv = poses[0].inverse();
						for(unsigned int j = 0; j < nr_frames; j++){
							if(!is_ok[j]){continue;}
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

								tXi(0,c)		= m00*x + m01*y + m02*z + m03;
								tXi(1,c)		= m10*x + m11*y + m12*z + m13;
								tXi(2,c)		= m20*x + m21*y + m22*z + m23;

								tXni(0,c)		= m00*nx + m01*ny + m02*nz;
								tXni(1,c)		= m10*nx + m11*ny + m12*nz;
								tXni(2,c)		= m20*nx + m21*ny + m22*nz;
							}
						}
						if(isconverged(poses, poses2, stopval, stopval)){break;}
					}
				}
				if(isconverged(poses, poses2b, stopval, stopval)){break;}
			}
			if(isconverged(poses, poses1, stopval, stopval)){break;}
		}

		double noise_before = func->getNoise();
		func->update();
		double noise_after = func->getNoise();
		if(fabs(1.0 - noise_after/noise_before) < 0.01){break;}
	}

	printf("total_time:          %5.5f\n",getTime()-total_time_start);
	printf("rematch_time:        %5.5f\n",rematch_time);
	printf("computeModel:        %5.5f\n",computeModel_time);
	printf("setup_matches_time:  %5.5f\n",setup_matches_time);
	printf("setup_equation_time: %5.5f\n",setup_equation_time);
	printf("solve_equation_time: %5.5f\n",solve_equation_time);
	printf("good opt: %f bad opt: %f ratio: %f\n",good_opt,bad_opt,good_opt/(good_opt+bad_opt));

	if(visualizationLvl > 0){
		std::vector<Eigen::MatrixXd> Xv;
		for(unsigned int j = 0; j < nr_frames; j++){Xv.push_back(transformed_points[j]);}
		sprintf(buf,"image%5.5i.png",imgcount++);
		show(Xv,false,std::string(buf),imgcount);
	}

	Eigen::Matrix4d firstinv = poses.front().inverse();
	for(int i = 0; i < nr_frames; i++){poses[i] = firstinv*poses[i];}

	printf("stop MassRegistrationPPR::getTransforms(std::vector<Eigen::Matrix4d> guess)\n");
	return MassFusionResults(poses,-1);
}

}
