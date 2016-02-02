#include "model2.h"
namespace modellib
{


unsigned int model_id_counter		= 0;
const float depth_scale				= 1000;
const float idepth_scale			= 0.001;

double getCurrentTime(){
	struct timeval start;
	gettimeofday(&start, NULL);
	double sec = start.tv_sec;
	double usec = start.tv_usec;
	double t = sec+usec*0.000001;
	return t;
}



Model2::Model2(std::vector< InputFrame * > & frames_, std::vector<Eigen::Matrix4f> & poses_){
	id				= model_id_counter++;

	maxd			= 0.1;
	smoothing		= 30.00;
	histogram_size	= 500;
	histogram		= new double[histogram_size+1];

	for(unsigned int i = 0; i < frames_.size(); i++){			add(	frames_.at(i),	poses_.at(i));}
//	for(unsigned int i = 0; i < frames_.size(); i++){pruneFromFrame(	frames_.at(i),	poses_.at(i));}
}

Model2::Model2(){
	id				= model_id_counter++;
	maxd			= 0.1;
	smoothing		= 30.00;
	histogram_size	= 500;
	histogram		= new double[histogram_size+1];
}
Model2::~Model2(){}

void Model2::merge( Model2 * m, Eigen::Matrix4f & pose){
	for(unsigned int i = 0; i < m->frames.size(); i++){add(m->frames.at(i),pose.inverse()*m->poses.at(i));}
	//for(unsigned int i = 0; i < frames.size(); i++){pruneFromFrame(frames.at(i),poses.at(i));}
/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = getFusedCloud();
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud1), "cloud1");
	viewer->spin();
	viewer->removeAllPointClouds();
*/
}

double * blurHist(double * hist, int nr_data = 512, double stdval = 0.5){
	double * tmp = new double[nr_data];
	double info = pow(stdval,-2);
	for(int i = 0; i < nr_data; i++){
		double sumV = 0;
		double sumW = 0;
		for(int j = 0; j < nr_data; j++){
			double dx = i-j;
			double w = exp(-0.5*dx*dx/(stdval));
			double v = hist[j];
			sumV += w*v;
			sumW += w;
		}
		tmp[i] = sumV/sumW;
	}
	return tmp;
}

class gaussian2 {
	public:
	gaussian2(double mean, double mul, double x, double y) : mean_(mean), mul_(mul), x_(x), y_(y) {}
	bool operator()(const double* const p,double* residuals) const {
		 double stddiv = p[0];
		if(mul_ > 0){
			double dx = x_-mean_;
			residuals[0]  = 0;
			residuals[0] += mul_*exp(-0.5*dx*dx/(stddiv*stddiv));
			residuals[0] -= y_;
			if(residuals[0] > 0){	residuals[0] =  sqrt( 5.0*residuals[0]);}
			else{					residuals[0] = -sqrt(-residuals[0]);}
		}else{residuals[0] = 99999999999999999999999.0;}
		return true;
	}
	private:
	const double mean_;
	const double mul_;
	const double x_;
	const double y_;
};

void getFit(double & mul, double & mean, double & stdval, double * hist, int nr_data = 512){
	mul = 0;
	mean = 0;
	for(unsigned int i = 0; i < nr_data;i++){
		double fgi = hist[i];
		if(fgi > mul){
			mul = fgi;
			mean = i;
		}
	}

	Problem problem;
	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	for(unsigned int i = 0; i < nr_data;i++){
		CostFunction* cost_function = new ceres::NumericDiffCostFunction<gaussian2, ceres::CENTRAL, 1, 1>(new gaussian2(mean,mul,i,hist[i]));
		problem.AddResidualBlock(cost_function,NULL,&stdval);
	}

	Solver::Summary summary;
	Solve(options, &problem, &summary);
}

void Model2::getInds(unsigned int * inds, std::vector<float> & distances){
	unsigned int nr_pixels = distances.size();
	for(unsigned int k = 0; k < nr_pixels; k++){
		inds[k] = histogram_size;
		float d = distances.at(k);
		int ind = histogram_size/2 + (0.5+d/(2*maxd));
		if(ind >= 0 && ind < histogram_size){inds[k] = ind;}
	}
}

void Model2::buildHist(unsigned int * inds, unsigned int nr_pixels,double * hist,  unsigned int nr_bins){
	for(unsigned int k = 0; k < nr_bins; k++){			hist[k] = 0;}
	for(unsigned int k = 0; k < nr_pixels; k++){		hist[inds[k]]++;}
}

void Model2::add(InputFrame * frame, Eigen::Matrix4f pose){
	frames.push_back(frame);
	poses.push_back(pose);

	float m00 = pose(0,0); float m01 = pose(0,1); float m02 = pose(0,2); float m03 = pose(0,3);
	float m10 = pose(1,0); float m11 = pose(1,1); float m12 = pose(1,2); float m13 = pose(1,3);
	float m20 = pose(2,0); float m21 = pose(2,1); float m22 = pose(2,2); float m23 = pose(2,3);
	unsigned int width					= frame->camera->width;
	unsigned int height					= frame->camera->height;
	float cx							= frame->camera->cx;
	float cy							= frame->camera->cy;
	float ifx							= 1.0/frame->camera->fx;
	float ify							= 1.0/frame->camera->fy;
	unsigned char	*	rgb				= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth			= (unsigned short	*)(frame->depth.data);

	int r = (rand()%4)*255/4; int g = (rand()%4)*255/4; int b = (rand()%4)*255/4;

	std::vector< Point * > current;
	for(unsigned int w = 0; w < width; w+=2){
		for(unsigned int h = 0; h < height; h+=2){
			unsigned int k = h*width+w;
			float z = idepth_scale*float(depth[k]);
			if(z != 0 && z < 3){
				float x		= (w - cx) * z * ifx;
				float y		= (h - cy) * z * ify;

				Point * p = new Point;
				p->x = m00*x + m01*y + m02*z + m03;
				p->y = m10*x + m11*y + m12*z + m13;
				p->z = m20*x + m21*y + m22*z + m23;
				p->b = r;	p->g = g;	p->r = b;	p->weight = 1;
				p->weight = 1.0/(z*z);
				current.push_back(p);
			}
		}
	}
	nonfused_points.push_back(current);
}

void Model2::getModel(double & mul, double & mean, double & stdval, double * hist,  unsigned int nr_bins){
	mul = 0;
	mean = 0;

	for(unsigned int k = 0; k < nr_bins; k++){
		double fgi = hist[k];
		if(fgi > mul){
			mul = fgi;
			mean = k;
		}
	}

	Problem problem;
	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	for(unsigned int k = 0; k < nr_bins; k++){
		CostFunction* cost_function = new ceres::NumericDiffCostFunction<gaussian2, ceres::CENTRAL, 1, 1>(new gaussian2(mean,mul,k,hist[k]));
		problem.AddResidualBlock(cost_function,NULL,&stdval);
	}
	Solver::Summary summary;
	Solve(options, &problem, &summary);
}

void Model2::getProbs(double & mul, double & mean, double & stdval, double * probs, double * hist,  unsigned int nr_bins){
	for(unsigned int k = 0; k < nr_bins; k++){
		double dx = double(k)-mean;
		double mu = mul				+1.0;
		double hs = hist[k]	+1.0;
		probs[k] = std::min(1.0 , (mu/hs)*exp(-0.5*dx*dx/(stdval*stdval)));
	}
}


	//For all points
		//0 repoject to compute dz
	//For all points
		//1 Compute bin id
	//2 Compute histogram
	//3 Compute noise model
	//4 if(bin id large change)
	// redo from 1
	//5 Compute probs
	//For all points
		//if infront
		// Remove
	//For all points
		//0 repoject to compute dz
void Model2::pruneFromFrame(InputFrame * frame, Eigen::Matrix4f pose){
	printf("void Model2::pruneFromFrame(InputFrame * frame, Eigen::Matrix4f pose){\n");
	unsigned char	*	rgb		= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);

	Eigen::Matrix4f transform	= pose.inverse();
	float m00 = transform(0,0); float m01 = transform(0,1); float m02 = transform(0,2); float m03 = transform(0,3);
	float m10 = transform(1,0); float m11 = transform(1,1); float m12 = transform(1,2); float m13 = transform(1,3);
	float m20 = transform(2,0); float m21 = transform(2,1); float m22 = transform(2,2); float m23 = transform(2,3);

	float cx				= frame->camera->cx; float cy				= frame->camera->cy;
	float fx				= frame->camera->fx; float fy				= frame->camera->fy;
	unsigned int width		= frame->camera->width;
	unsigned int height		= frame->camera->height;
	unsigned int width1		= frame->camera->width-1;
	unsigned int height1	= frame->camera->height-1;

	std::vector<float> distances;
	std::vector< Point *> points;

	const bool interpolate = true;

	for(unsigned int k = 0; k < nonfused_points.size(); k++){
		if(frames.at(k) == frame){continue;}
		std::vector<Point *> & current = nonfused_points.at(k);
		unsigned int nr_points = current.size();
		for(unsigned int i = 0; i < nr_points; i++){
			Point * p		= current.at(i);
			float x			= p->x;
			float y			= p->y;
			float z			= p->z;
			float tz		= m20*x + m21*y + m22*z + m23;

			if(tz > 0){
				float tx		= m00*x + m01*y + m02*z + m03;
				float ty		= m10*x + m11*y + m12*z + m13;
				float itz	= 1.0/tz;
				float w	= fx*tx*itz + cx;
				float h	= fy*ty*itz + cy;

				//check reprojection to ensure its within the image plane
				if((w > 1) && (h > 1) && (w < width1) && (h < height1)){
					float rz = 0;
					if(interpolate){
						unsigned int w0 = unsigned(w);
						unsigned int w1 = unsigned(w+1);
						float ww = w-float(w0);
						unsigned int h0 = unsigned(h);
						unsigned int h1 = unsigned(h+1);
						float hw = h-float(h0);

						float w00 = (1-ww)*(1-hw);
						float w01 = (1-ww)*hw;
						float w10 = ww*(1-hw);
						float w11 = ww*hw;

						rz = idepth_scale*(w00*depth[w0+h0*width] + w01*depth[w0+h1*width] + w10*depth[w1+h0*width] + w11*depth[w1+h1*width]);
					}else{
						unsigned int k2 = unsigned(h+0.5) * width + unsigned(w+0.5);
						rz = idepth_scale*depth[k2];
					}
					if(rz > 0){
						float dz = (rz-tz)/(sqrt(rz)*rz);
						points.push_back(p);
						distances.push_back(dz);
					}
				}
			}
		}
	}

	unsigned int nr_pixels = distances.size();
	unsigned int * inds = new unsigned int[nr_pixels];
	double * prob = new double[histogram_size+1];
	prob[histogram_size] = 0;

	bool run_loop = true;
	while(run_loop){
		getInds(inds, distances);
		buildHist(inds,nr_pixels,histogram,histogram_size);
		double * hist_smooth = blurHist(histogram, histogram_size, smoothing);

		double mul = 0;
		double mean = 0;
		double stdval = 10;

		getModel(mul,mean,stdval,hist_smooth,histogram_size);
		double divmax = stdval*20.0/float(histogram_size);
		maxd *= divmax;

		if(fabs(1-divmax) < 0.1){
			stdval *= 1.025;
			getProbs(mul, mean, stdval, prob,hist_smooth,histogram_size);
			run_loop = false;
		}
		delete[] hist_smooth;
	}

	const bool visualize = true;

	if(visualize){
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
		viewer->addCoordinateSystem();
		viewer->setBackgroundColor(1.0,0.0,1.0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		for(unsigned int k = 0; k < points.size(); k++){
			Point * po = points.at(k);
			double pr = prob[inds[k]];
			pcl::PointXYZRGB p;
			p.x = po->x;
			p.y = po->y;
			p.z = po->z;
			p.r = po->r;//255;
			p.g = po->g;//255;
			p.b = po->b;//255;
			if(distances.at(k) > 0){
				p.r = 255*pr;
				p.g = 255*pr;
				p.b = 255*pr;
			}else{
				p.r = 255*(0.5+0.5*pr);
				p.g = 255*(0.5+0.5*pr);
				p.b = 255*(0.5+0.5*pr);
			}
			cloud->points.push_back(p);
		}
		printf("line: %i\n",__LINE__);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
		viewer->spin();
		viewer->removeAllPointClouds();
	}

	for(unsigned int k = 0; k < points.size(); k++){
		Point * po = points.at(k);
		double pr = prob[inds[k]];
		if(distances.at(k) > 0 && pr < 0.2f){po->weight = -1;}//Mark for deletion
	}
	for(unsigned int k = 0; k < nonfused_points.size(); k++){
		std::vector<Point *> & current = nonfused_points.at(k);
		for(unsigned int i = 0; i < current.size(); i++){
			Point * po = current.at(i);
			if(po->weight == -1){
				delete po;
				current.at(i) = current.back();
				current.pop_back();
				i--;
			}
		}
	}
}

Eigen::Matrix4f Model2::align( Model2 * m, Eigen::Matrix4f pose, bool show = false){
	return align(m->frames,m->poses,pose,show);
}

void getMatchesProj(Eigen::Matrix4f tr, InputFrame * frame, std::vector<Point *> & current, std::vector<float> & distances, std::vector<float> & scaling, std::vector< Point *> & points, std::vector< Point *> & matchpoints){

	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);

	Eigen::Matrix4f itr	= tr.inverse();
	float im00 = itr(0,0); float im01 = itr(0,1); float im02 = itr(0,2); float im03 = itr(0,3);
	float im10 = itr(1,0); float im11 = itr(1,1); float im12 = itr(1,2); float im13 = itr(1,3);
	float im20 = itr(2,0); float im21 = itr(2,1); float im22 = itr(2,2); float im23 = itr(2,3);

	unsigned char	*	rgb		= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);
	float cx					= frame->camera->cx; 	float cy	= frame->camera->cy;
	float fx					= frame->camera->fx; 	float fy	= frame->camera->fy;
	unsigned int width			= frame->camera->width;
	unsigned int height			= frame->camera->height;
	unsigned int width1			= frame->camera->width-1;
	unsigned int height1		= frame->camera->height-1;
	
	unsigned int nr_points 		= current.size();
	
	bool interpolate = true;
	for(unsigned int i = 0; i < nr_points; i++){
		Point * p		= current.at(i);
		float x			= p->x;
		float y			= p->y;
		float z			= p->z;
		float tz		= im20*x + im21*y + im22*z + im23;

		if(tz > 0){
			float tx		= im00*x + im01*y + im02*z + im03;
			float ty		= im10*x + im11*y + im12*z + im13;
			float itz	= 1.0/tz;
			float w	= fx*tx*itz + cx;
			float h	= fy*ty*itz + cy;

			//check reprojection to ensure its within the image plane
			if((w > 1) && (h > 1) && (w < width1) && (h < height1)){
				float rz = 0;

				if(interpolate){
					unsigned int w0 = unsigned(w);
					unsigned int w1 = unsigned(w+1);
					float ww = w-float(w0);
					unsigned int h0 = unsigned(h);
					unsigned int h1 = unsigned(h+1);
					float hw = h-float(h0);

					float w00 = (1-ww)*(1-hw);		float w01 = (1-ww)*hw;			float w10 = ww*(1-hw);			float w11 = ww*hw;
					float z00 = depth[w0+h0*width];	float z01 = depth[w0+h1*width];	float z10 = depth[w1+h0*width];	float z11 = depth[w1+h1*width];

					float maxz = std::max(std::max(z00,z01),std::max(z10,z11));
					float minz = std::min(std::min(z00,z01),std::min(z10,z11));
					float validmarker = ((z00 > 0) + (z01 > 0) + (z10 > 0) + (z11 > 0));

					rz = float(idepth_scale*(maxz-minz) < 0.15)*float(validmarker == 4)*idepth_scale*(w00*z00 + w01*z01 + w10*z10 + w11*z11);
				}else{
					unsigned int k2 = unsigned(h+0.5) * width + unsigned(w+0.5);
					rz = idepth_scale*depth[k2];
				}
				if(rz > 0){
					float sc = rz*rz;//sqrt(rz)*rz;
					float dz = (rz-tz)/sc;
					float z2	= rz;
					float x2	= (w - cx) * z2 / fx;
					float y2	= (h - cy) * z2 / fy;

					Point * p2 = new Point;
					p2->x = m00*x2 + m01*y2 + m02*z2 + m03;
					p2->y = m10*x2 + m11*y2 + m12*z2 + m13;
					p2->z = m20*x2 + m21*y2 + m22*z2 + m23;
					
					points.push_back(p);
					matchpoints.push_back(p2);
					scaling.push_back(sc);
					distances.push_back(dz);
				}
			}
		}
	}
}

float minv(std::vector<float> & v){
	float val = v[0];
	for(unsigned int i = 1; i < v.size(); i++){val = std::min(v[0],val);}
	return val;
}

float maxv(std::vector<float> & v){
	float val = v[0];
	for(unsigned int i = 1; i < v.size(); i++){val = std::min(v[i],val);}
	return val;
}

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> getOctree(std::vector<Point *> current, float resolution = 0.025){
	unsigned int nr_points 		= current.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = nr_points;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for(unsigned int i = 0; i < nr_points; i+=1){
		Point * p				= current.at(i);
		pcl::PointXYZ & pclp = cloud->points[i];
		pclp.x = p->x;
		pclp.y = p->y;
		pclp.z = p->z;
	}

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud();
	return octree;
}


pcl::KdTreeFLANN<pcl::PointXYZ> getKDTree(std::vector<Point *> & current){
	unsigned int nr_points 		= current.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = nr_points;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for(unsigned int i = 0; i < nr_points; i+=1){
		Point * p				= current.at(i);
		pcl::PointXYZ & pclp = cloud->points[i];
		pclp.x = p->x;
		pclp.y = p->y;
		pclp.z = p->z;
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);
	return kdtree;
}

void getPointsToMatch(InputFrame * frame, std::vector<Point *> & points, std::vector<float> & scaling, int step){
	unsigned char	*	rgb		= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);
	float cx					= frame->camera->cx; 	float cy	= frame->camera->cy;
	float fx					= frame->camera->fx; 	float fy	= frame->camera->fy;
	unsigned int width			= frame->camera->width;
	unsigned int height			= frame->camera->height;
	unsigned int width1			= frame->camera->width-1;
	unsigned int height1		= frame->camera->height-1;
	
	for(unsigned int w = 0; w < width; w+=5){
		for(unsigned int h = 0; h < height; h+=5){
			unsigned int k = h * width + w;
			float z	= idepth_scale*depth[k];
			if(z > 0){
				float x	= (w - cx) * z / fx;
				float y	= (h - cy) * z / fy;
				Point * p = new Point;	p->x = x;	p->y = y;	p->z = z;
				scaling.push_back(z*z);
				points.push_back(p);
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getImageClouds(InputFrame * frame){

	unsigned char	*	rgb		= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);
	float cx					= frame->camera->cx; 	float cy	= frame->camera->cy;
	float fx					= frame->camera->fx; 	float fy	= frame->camera->fy;
	unsigned int width			= frame->camera->width;
	unsigned int height			= frame->camera->height;
	unsigned int width1			= frame->camera->width-1;
	unsigned int height1		= frame->camera->height-1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr imgcloud (new pcl::PointCloud<pcl::PointXYZ>);
	imgcloud->width = 640;
	imgcloud->height = 480;
	imgcloud->points.resize (imgcloud->width * imgcloud->height);
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height; h++){
			unsigned int k = h * width + w;
			float z	= idepth_scale*depth[k];
			pcl::PointXYZ & p = imgcloud->points[k];
			if(z > 0){
				p.z = z;
				p.x	= (w - cx) * z / fx;
				p.y	= (h - cy) * z / fy;
			}else{
				p.z = std::numeric_limits<double>::quiet_NaN();
				p.x	= std::numeric_limits<double>::quiet_NaN();
				p.y	= std::numeric_limits<double>::quiet_NaN();
			}
		}
	}

	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor(0.025f);
	ne.setNormalSmoothingSize(20.0f);
	ne.setInputCloud(imgcloud);
	ne.compute(*normals);
/*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr imgcloud2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	imgcloud2->width = 640;
	imgcloud2->height = 480;
	imgcloud2->points.resize (imgcloud2->width * imgcloud2->height);
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height; h++){
			unsigned int k = h * width + w;
			pcl::PointXYZRGBNormal & p1 = imgcloud2->points[k];
			pcl::PointXYZ & p2 = imgcloud->points[k];
			pcl::Normal & p3 = imgcloud->points[k];
		}
	}
*/
/*
	cv::Mat img = cv::Mat::zeros(480,640, CV_8UC3);
	unsigned char * imgdata = (unsigned char*)img.data;
	
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height; h++){
			unsigned int k = h * width + w;
			pcl::Normal & p = normals->points[k];
			imgdata[3*k+0] = 0.5*(1.0+p.normal_x)*255.0;
			imgdata[3*k+1] = 0.5*(1.0+p.normal_y)*255.0;
			imgdata[3*k+2] = 0.5*(1.0+p.normal_z)*255.0;
		}
	}
	
	cv::Mat img2 = cv::Mat::zeros(480,640, CV_8UC3);
	unsigned char * imgdata2 = (unsigned char*)img2.data;
	
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height; h++){
			unsigned int k = h * width + w;
			pcl::Normal & p = normals->points[k];
			pcl::PointXYZ & p2 = imgcloud->points[k];
			//if(w % 10 == 0 && h % 10 == 0){printf("%i %i -> %f\n",w,h,p.curvature);}
			imgdata2[3*k+0] = 10.0*p.curvature*255.0/p2.z/p2.z;
			imgdata2[3*k+1] = 10.0*p.curvature*255.0/p2.z/p2.z;
			imgdata2[3*k+2] = 10.0*p.curvature*255.0/p2.z/p2.z;
		}
	}
	
	cv::namedWindow( "img", cv::WINDOW_AUTOSIZE );	// Create a window for display.
    cv::imshow( "img", img );						// Show our image inside it.
    
	cv::namedWindow( "img2", cv::WINDOW_AUTOSIZE );	// Create a window for display.
    cv::imshow( "img2", img2 );						// Show our image inside it.
    cv::waitKey(0);									// Wait for a keystroke in the window
*/
}

/*
void getMatches(Eigen::Matrix4f tr,
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree1, std::vector< Point *>	& search_points,	std::vector<float> scaling, 
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree0, std::vector<Point *> & current, 			std::vector<float> & distances, 
	std::vector< Point *> & points, std::vector< Point *> & matchpoints
){

	int K = 1;
	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
	
	Eigen::Matrix4f itr	= tr.inverse();
	float im00 = itr(0,0); float im01 = itr(0,1); float im02 = itr(0,2); float im03 = itr(0,3);
	float im10 = itr(1,0); float im11 = itr(1,1); float im12 = itr(1,2); float im13 = itr(1,3);
	float im20 = itr(2,0); float im21 = itr(2,1); float im22 = itr(2,2); float im23 = itr(2,3);
	
	unsigned int nr_points 		= search_points.size();
	for(unsigned int i = 0; i < nr_points; i++){
		Point * p0	= search_points[i];
		float sc	= scaling[i];
		
		float x 	= p0->x;
		float y		= p0->y;
		float z		= p0->z;
		float tz	= m20*x + m21*y + m22*z + m23;
		float tx	= m00*x + m01*y + m02*z + m03;
		float ty	= m10*x + m11*y + m12*z + m13;
		
		pcl::PointXYZ sp0;
		sp0.x = tx;
		sp0.y = ty;
		sp0.z = tz;
		std::vector<int> pointIdxNKNSearch0;
		std::vector<float> pointNKNSquaredDistance0;

		if (tree0.nearestKSearch (sp0, K, pointIdxNKNSearch0, pointNKNSquaredDistance0) > 0){
			Point * p1 = current.at(pointIdxNKNSearch0[0]);
			
			float x1 	= p1->x;
			float y1	= p1->y;
			float z1	= p1->z;
			float itz	= im20*x1 + im21*y1 + im22*z1 + im23;
			float itx	= im00*x1 + im01*y1 + im02*z1 + im03;
			float ity	= im10*x1 + im11*y1 + im12*z1 + im13;
			
			pcl::PointXYZ sp1;
			sp1.x = itx;
			sp1.y = ity;
			sp1.z = itz;
			std::vector<int> pointIdxNKNSearch1;
			std::vector<float> pointNKNSquaredDistance1;
			
			if (tree1.nearestKSearch (sp1, K, pointIdxNKNSearch1, pointNKNSquaredDistance1) > 0){
				if(i == pointIdxNKNSearch1[0]){
					//printf("---------------------\n");
					printf("%i -> %i -> %i\n",i,pointIdxNKNSearch0[0],pointIdxNKNSearch1[0]);
					
					Point * p2 = new Point;	p2->x = tx;	p2->y = ty;	p2->z = tz;
					
					float dx = tx - x1;
					float dy = ty - y1;
					float dz = tz - z1;
					
					float d = sqrt(dx*dx+dy*dy+dz*dz)/sc;
					
					points.push_back(p1);
					matchpoints.push_back(p2);
					distances.push_back(d);
					
				}
			}
		}
	}
}
*/
void getMatches(Eigen::Matrix4f tr, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree,  InputFrame * frame, std::vector<Point *> & current, std::vector<float> & distances, std::vector<float> & scaling, std::vector< Point *> & points, std::vector< Point *> & matchpoints){
//void getMatches(Eigen::Matrix4f tr, pcl::KdTreeFLANN<pcl::PointXYZ> tree, InputFrame * frame, std::vector<Point *> & current, std::vector<float> & distances, std::vector<float> & scaling, std::vector< Point *> & points, std::vector< Point *> & matchpoints){

	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);

	unsigned char	*	rgb		= (unsigned char	*)(frame->rgb.data);
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);
	float cx					= frame->camera->cx; 	float cy	= frame->camera->cy;
	float fx					= frame->camera->fx; 	float fy	= frame->camera->fy;
	unsigned int width			= frame->camera->width;
	unsigned int height			= frame->camera->height;
	unsigned int width1			= frame->camera->width-1;
	unsigned int height1		= frame->camera->height-1;

	unsigned int nr_points 		= current.size();

	double start_time = getCurrentTime();
	for(unsigned int w = 0; w < width; w+=4){
		for(unsigned int h = 0; h < height; h+=4){
			unsigned int k = h * width + w;
			float z	= idepth_scale*depth[k];
			if(z > 0){
				float x	= (w - cx) * z / fx;
				float y	= (h - cy) * z / fy;
				
				float tz		= m20*x + m21*y + m22*z + m23;
				float tx		= m00*x + m01*y + m02*z + m03;
				float ty		= m10*x + m11*y + m12*z + m13;
				
				pcl::PointXYZ sp;
				sp.x = tx;
				sp.y = ty;
				sp.z = tz;
				int K = 1;
				std::vector<int> pointIdxNKNSearch;
				std::vector<float> pointNKNSquaredDistance;
				if (tree.nearestKSearch (sp, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
					float sc = z*z;
					Point * p1 = current.at(pointIdxNKNSearch[0]);
					Point * p2 = new Point;	p2->x = tx;	p2->y = ty;	p2->z = tz;
					
					float dx = tx - p1->x;
					float dy = ty - p1->y;
					float dz = tz - p1->z;
					
					float d = sqrt(dx*dx+dy*dy+dz*dz)/sc;
					
					points.push_back(p1);
					matchpoints.push_back(p2);
					scaling.push_back(sc);
					distances.push_back(d);
				}else{
					printf("WARNING, POINT NOT MATCHED\n");exit(0);
				}
			}
		}
	}
}

typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OTREE;
using namespace std;
using namespace Eigen;
std::vector< int > getMatches(Matrix4f tr, OTREE tree0, vector<Point *> & points0, OTREE tree1, vector<Point *> & points1){
	int K = 1;
	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
	
	Eigen::Matrix4f itr	= tr.inverse();
	float im00 = itr(0,0); float im01 = itr(0,1); float im02 = itr(0,2); float im03 = itr(0,3);
	float im10 = itr(1,0); float im11 = itr(1,1); float im12 = itr(1,2); float im13 = itr(1,3);
	float im20 = itr(2,0); float im21 = itr(2,1); float im22 = itr(2,2); float im23 = itr(2,3);

	unsigned int nr_points 		= points0.size();
	std::vector< int > matches;
	matches.resize(nr_points);
	
	for(unsigned int i = 0; i < nr_points; i++){
		matches[i]	= -1;
		Point * p0	= points0[i];

		float x 	= p0->x;
		float y		= p0->y;
		float z		= p0->z;
		
		pcl::PointXYZ sp0;
		sp0.x = m00*x + m01*y + m02*z + m03;
		sp0.y = m10*x + m11*y + m12*z + m13;
		sp0.z = m20*x + m21*y + m22*z + m23;
		std::vector<int> pointIdxNKNSearch0;
		std::vector<float> pointNKNSquaredDistance0;

		if (tree0.nearestKSearch (sp0, K, pointIdxNKNSearch0, pointNKNSquaredDistance0) > 0){

			Point * p1 = points1.at(pointIdxNKNSearch0[0]);
			
			float x1 	= p1->x;
			float y1	= p1->y;
			float z1	= p1->z;
	
			pcl::PointXYZ sp1;
			sp1.x = im00*x1 + im01*y1 + im02*z1 + im03;
			sp1.y = im10*x1 + im11*y1 + im12*z1 + im13;
			sp1.z = im20*x1 + im21*y1 + im22*z1 + im23;
			std::vector<int> pointIdxNKNSearch1;
			std::vector<float> pointNKNSquaredDistance1;
			if (tree1.nearestKSearch (sp1, K, pointIdxNKNSearch1, pointNKNSquaredDistance1) > 0){
				printf("%i -> %i -> %i\n",i,pointIdxNKNSearch0[0],pointIdxNKNSearch1[0]);
/*
				if(i == pointIdxNKNSearch1[0]){
					//printf("---------------------\n");
					printf("%i -> %i -> %i\n",i,pointIdxNKNSearch0[0],pointIdxNKNSearch1[0]);
					
					Point * p2 = new Point;	p2->x = tx;	p2->y = ty;	p2->z = tz;
					
					float dx = tx - x1;
					float dy = ty - y1;
					float dz = tz - z1;
					
					float d = sqrt(dx*dx+dy*dy+dz*dz)/sc;
					
					points.push_back(p1);
					matchpoints.push_back(p2);
					distances.push_back(d);
					
				}
*/
			}
		}
	}
	
	return matches;
}

std::vector< pair <int,int> > getMatches(Matrix4f tr, vector<Point *> points0, vector<Point *> points1){

	double start_time = getCurrentTime();
	OTREE tree0 = getOctree(points0,0.05);
	OTREE tree1 = getOctree(points1,0.05);
	printf("setup trees time: %f\n",getCurrentTime()-start_time);

	unsigned int nr_points0 		= points0.size();
	unsigned int nr_points1 		= points1.size();
	std::vector< pair <int,int> > matches;

	int K = 1;
	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
	
	Eigen::Matrix4f itr	= tr.inverse();
	float im00 = itr(0,0); float im01 = itr(0,1); float im02 = itr(0,2); float im03 = itr(0,3);
	float im10 = itr(1,0); float im11 = itr(1,1); float im12 = itr(1,2); float im13 = itr(1,3);
	float im20 = itr(2,0); float im21 = itr(2,1); float im22 = itr(2,2); float im23 = itr(2,3);
	
/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(1.0,1.0,1.0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(unsigned int i = 0; i < nr_points0; i++){
		Point * p0	= points0[i];
		float x 	= p0->x;
		float y		= p0->y;
		float z		= p0->z;
		
		pcl::PointXYZRGB sp0;
		sp0.x = m00*x + m01*y + m02*z + m03;
		sp0.y = m10*x + m11*y + m12*z + m13;
		sp0.z = m20*x + m21*y + m22*z + m23;
		sp0.r = 0;	sp0.g = 0;	sp0.b = 255;
		
		cloud->points.push_back(sp0);
	}

	for(unsigned int i = 0; i < nr_points1; i++){cloud->points.push_back(points1[i]->getPCLPoint(255,0,0));}

	
	
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
	viewer->spin();	
	cloud->points.clear();
*/

	start_time = getCurrentTime();
//for(int k = 0; k < 100; k++){
	for(unsigned int i = 0; i < nr_points0; i++){
		Point * p0	= points0[i];

		float x0 	= p0->x;
		float y0	= p0->y;
		float z0	= p0->z;
		
		pcl::PointXYZ sp0;
		sp0.x = m00*x0 + m01*y0 + m02*z0 + m03;
		sp0.y = m10*x0 + m11*y0 + m12*z0 + m13;
		sp0.z = m20*x0 + m21*y0 + m22*z0 + m23;
		std::vector<int> pointIdxNKNSearch0;
		std::vector<float> pointNKNSquaredDistance0;

		if (tree1.nearestKSearch (sp0, K, pointIdxNKNSearch0, pointNKNSquaredDistance0) > 0){
			unsigned int j = pointIdxNKNSearch0[0];

			Point * p1	= points1.at(j);
			float x1 	= p1->x;
			float y1	= p1->y;
			float z1	= p1->z;
			
			pcl::PointXYZ sp1;
			sp1.x = im00*x1 + im01*y1 + im02*z1 + im03;
			sp1.y = im10*x1 + im11*y1 + im12*z1 + im13;
			sp1.z = im20*x1 + im21*y1 + im22*z1 + im23;
			std::vector<int> pointIdxNKNSearch1;
			std::vector<float> pointNKNSquaredDistance1;
			if (tree0.nearestKSearch (sp1, K, pointIdxNKNSearch1, pointNKNSquaredDistance1) > 0){
				if(i == pointIdxNKNSearch1[0]){
					matches.push_back(make_pair(i,j));
/*
					pcl::PointXYZRGB bsp0;
					bsp0.x = sp0.x;
					bsp0.y = sp0.y;
					bsp0.z = sp0.z;
					bsp0.r = 0;	bsp0.g = 0;	bsp0.b = 255;
					cloud->points.push_back(bsp0);
				
					pcl::PointXYZRGB basp0;
					basp0.x = x1;
					basp0.y = y1;
					basp0.z = z1;
					basp0.r = 0;	basp0.g = 255;	basp0.b = 0;
					cloud->points.push_back(basp0);
*/
				}
			}
		}
	}
//}
	printf("match time: %f\n",getCurrentTime()-start_time);
/*
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
	viewer->spin();	
*/
	return matches;
}

void applyTransformation(Eigen::Matrix4f & tr, std::vector< Point *> & points){
	unsigned int nr_points = points.size();
	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
	for(unsigned int k = 0; k < nr_points; k++){
		Point * p = points[k];
		float x	= p->x;
		float y	= p->y;
		float z	= p->z;

		p->x		= m00*x + m01*y + m02*z + m03;
		p->y		= m10*x + m11*y + m12*z + m13;
		p->z		= m20*x + m21*y + m22*z + m23;
	}
}

void Model2::align(bool show){

	for(unsigned int i = 0; i < 1; i++){
		for(unsigned int f = 1; f < frames.size(); f++){
			std::vector<InputFrame * > tmp_frames;tmp_frames.push_back(frames[f]);
			std::vector<Eigen::Matrix4f  > tmp_poses;tmp_poses.push_back(poses[f]);
			Eigen::Matrix4f tr = align(tmp_frames, tmp_poses, Eigen::Matrix4f::Identity(),show).inverse();
			cout << tr << endl;

			float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
			float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
			float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
			poses[f] *= tr;
			
			vector<Point *> & points	= nonfused_points.at(f);
			unsigned int nr_points		= points.size();

			for(unsigned int k = 0; k < nr_points; k++){
				Point * p = points[k];
				float x	= p->x;
				float y	= p->y;
				float z	= p->z;

				p->x		= m00*x + m01*y + m02*z + m03;
				p->y		= m10*x + m11*y + m12*z + m13;
				p->z		= m20*x + m21*y + m22*z + m23;
			}
/**/
		}
	}

}

std::vector< Point *> getPointsVec(InputFrame * frame,int step){
	unsigned short	*	depth	= (unsigned short	*)(frame->depth.data);
	float cx					= frame->camera->cx;		float cy	= frame->camera->cy;
	float fx					= frame->camera->fx;		float fy	= frame->camera->fy;
	unsigned int width			= frame->camera->width;
	unsigned int height			= frame->camera->height;

	std::vector< Point *> 	points0;
	for(unsigned int w = 0; w < width; w+=step){
		for(unsigned int h = 0; h < height; h+=step){
			float z	= idepth_scale*depth[h * width + w];
			if(z > 0){
				Point * p2 = new Point;
				p2->x = (w - cx) * z / fx;
				p2->y = (h - cy) * z / fy;
				p2->z = z;
				p2->weight = 1.0/(z*z);
				points0.push_back(p2);
			}
		}
	}
	return points0;
}

float getPairWeight(Point * p0,Point * p1){
	return p0->weight;//+p1->weight;
}

MatrixXf getEigenFromPoints(vector<Point *> points){
	unsigned int nr_points = points.size();
	MatrixXf mat = MatrixXf(nr_points,3);
	for(unsigned int i = 0; i < nr_points; i++){
		Point * p	= points[i];
		mat(i,0)	= p->x;
		mat(i,1)	= p->y;
		mat(i,2)	= p->z;
	}
	return mat;
}

vector< pair <int,int> > getMatchesEigen(MatrixXf a, MatrixXf b){
	vector< pair <int,int> > matches;
	return matches;
}
/*
MatrixXf transformEigen(MatrixXf points, MatrixXf transform){
	unsigned int nr_points = points.size();
	MatrixXf mat = MatrixXf(nr_points,3);
	for(unsigned int i = 0; i < nr_points; i++){
		Point * p0	= points0[i];
		mat(i,0) = p0->x;
		mat(i,1) = p0->y;
		mat(i,2) = p0->z;
	}
	return mat;
}
*/
/*
MatrixXf transformEigen(MatrixXf points, MatrixXf transform){
	unsigned int nr_points = points.size();
	MatrixXf mat = MatrixXf(nr_points,3);
	for(unsigned int i = 0; i < nr_points; i++){
		Point * p0	= points0[i];
		mat(i,0) = p0->x;
		mat(i,1) = p0->y;
		mat(i,2) = p0->z;
	}
	return mat;
}
*/
MatrixXf getDiffs(vector< pair <int,int> > matches, Matrix4f tr, vector<Point *> points0, vector<Point *> points1){
	float m00 = tr(0,0); float m01 = tr(0,1); float m02 = tr(0,2); float m03 = tr(0,3);
	float m10 = tr(1,0); float m11 = tr(1,1); float m12 = tr(1,2); float m13 = tr(1,3);
	float m20 = tr(2,0); float m21 = tr(2,1); float m22 = tr(2,2); float m23 = tr(2,3);
	
	Eigen::Matrix4f itr	= tr.inverse();
	float im00 = itr(0,0); float im01 = itr(0,1); float im02 = itr(0,2); float im03 = itr(0,3);
	float im10 = itr(1,0); float im11 = itr(1,1); float im12 = itr(1,2); float im13 = itr(1,3);
	float im20 = itr(2,0); float im21 = itr(2,1); float im22 = itr(2,2); float im23 = itr(2,3);
	
	unsigned int nr_matches = matches.size();
	
	MatrixXf mat = MatrixXf(nr_matches,3);
	for(unsigned int i = 0; i < nr_matches; i++){
		pair<int,int> & match = matches[i];
		
		Point * p0	= points0[match.first];
		float x 	= p0->x;
		float y		= p0->y;
		float z		= p0->z;
		
		Point * p1	= points1[match.second];
		
		float pair_information = getPairWeight(p0,p1);
		mat(i,0) = (m00*x + m01*y + m02*z + m03 - p1->x)*pair_information;
		mat(i,1) = (m10*x + m11*y + m12*z + m13 - p1->y)*pair_information;
		mat(i,2) = (m20*x + m21*y + m22*z + m23 - p1->z)*pair_information;		
	}
	return mat;
}

//std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > <---- SNABBARE!
Eigen::Matrix4f Model2::align( std::vector<InputFrame * > & frames_, std::vector< Eigen::Matrix4f > & poses_, Eigen::Matrix4f pose, bool show = false){
	DistanceWeightFunctionPPR * weightfunction = new DistanceWeightFunctionPPR();
	DistanceWeightFunction2PPR * wf = new DistanceWeightFunction2PPR();
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if(show){
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
		viewer->addCoordinateSystem();
		viewer->setBackgroundColor(1.0,0.0,1.0);
	}

	std::vector< pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> > trees0;
	for(unsigned int k = 0; k < nonfused_points.size(); k++){trees0.push_back(getOctree(nonfused_points.at(k),0.025));}

	std::vector< std::vector< Point *> >	points0_vector;
	for(unsigned int f = 0; f < frames_.size(); f++){points0_vector.push_back(getPointsVec(frames_.at(f),5));}

	float regu = 0.05;
	for(unsigned int it = 0; it < 200; it++){
		weightfunction->regularization = regu;
		wf->regularization = regu;
		regu *= 0.8;

		if(show){
			printf("--------------------------------------------\n");
			printf("%i regularization: %f\n",it,weightfunction->regularization);
		}

		std::vector<float> probs;
		std::vector<float> distances;
		std::vector<float> scaling;
		std::vector< Point *> points;
		std::vector< Point *> matchpoints;
		

		vector<vector< pair <int,int> >> all_matches;
		vector<pair <int,int> > all_combinations;
		
		//Find most likeley pairs
		double start_time = getCurrentTime();
		for(unsigned int f = 0; f < frames_.size(); f++){
			InputFrame * frame	= frames_.at(f);
			Eigen::Matrix4f tr	= pose * poses_.at(f);
			for(unsigned int k = 0; k < 10000 && k < nonfused_points.size(); k++){
				if(frames.at(k) == frame){continue;}
				all_matches.push_back(getMatches(tr,points0_vector[f],nonfused_points[k]));
				all_combinations.push_back(make_pair(f,k));
			}
		}
		if(show){printf("nn search time: %f\n",getCurrentTime()-start_time);}
		
		//Compute probabiletys using matches and transform 
		vector<VectorXf> all_probs;
		std::vector<float> all_noise;
		for(unsigned int m = 0; m < all_combinations.size(); m++){
			unsigned int f = all_combinations[m].first;
			unsigned int k = all_combinations[m].second;
			Eigen::Matrix4f tr	= pose * poses_.at(f);
			
			MatrixXf ei0 = getEigenFromPoints(points0_vector[f]);
			MatrixXf ei1 = getEigenFromPoints(nonfused_points[k]);
			
			MatrixXf diffmat = getDiffs(all_matches[m],tr,points0_vector[f],nonfused_points[k]);
			all_probs.push_back(wf->getProbs(diffmat));
			all_noise.push_back(wf->getNoise());
		}
		start_time = getCurrentTime();
		
		//Compute transformation using matches
		pcl::TransformationFromCorrespondences tfc;
		for(unsigned int m = 0; m < all_combinations.size(); m++){
			unsigned int f = all_combinations[m].first;
			unsigned int k = all_combinations[m].second;
			
			VectorXf probs = all_probs[m];
			float noise = all_noise[m];
			float information = 1.0/(noise*noise);
			vector< pair <int,int> > matches = all_matches[m];
			std::vector< Point *> points0 = points0_vector[f];
			std::vector< Point *> points1 = nonfused_points[k];
			for(unsigned int i = 0; i < matches.size(); i++){
				Point * p0 = points0[matches[i].first];
				Point * p1 = points1[matches[i].second];
				float prob = probs[i];
				float pair_weight = getPairWeight(p0,p1);
				float pair_information = pair_weight*pair_weight;
				
				tfc.add(p0->getEIGEN(poses_.at(f)),p1->getEIGEN(),prob*information*pair_information);
			}
		}
		Eigen::Matrix4f new_pose = tfc.getTransformation().matrix().cast<float>();
		pose = new_pose;

	//visualize
	if(show){
		for(unsigned int m = 0; m < all_combinations.size(); m++){
			unsigned int f = all_combinations[m].first;
			unsigned int k = all_combinations[m].second;
			
			std::vector< Point *> points0 = points0_vector[f];
			std::vector< Point *> points1 = nonfused_points[k];
			
			Eigen::Matrix4f tr	= pose*poses_.at(f);//pose.inverse() * poses_.at(f);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			for(unsigned int i = 0; i < points0.size(); i++){
				if(rand() % 2 == 0){
					cloud->points.push_back(points0.at(i)	->getPCLPoint(tr,0,0,255));
				}
			}
			for(unsigned int i = 0; i < points1.size(); i++){
				if(rand() % 5 == 0){
					cloud->points.push_back(points1.at(i)	->getPCLPoint(255,0,0));
				}
			}
			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
			viewer->spin();
/*
			VectorXf probs = all_probs[m];
			float noise = all_noise[m];
			float information = 1.0/(noise*noise);
			vector< pair <int,int> > matches = all_matches[m];
			MatrixXf diffmat	= getDiffs(all_matches[m],tr,points0_vector[f],nonfused_points[k]);
			viewer->removeAllShapes();
			for(unsigned int i = 0; i < matches.size(); i++){
				Point * p0 = points0[matches[i].first];
				Point * p1 = points1[matches[i].second];
				
				float pair_weight = getPairWeight(p0,p1);//+p1->weight;
				float pair_information = pair_weight*pair_weight;//*pair_weight;
				float w = probs[i]*pair_information;

				//char buf[1024];
				//sprintf(buf,"%i",i);
				//if(i % 20 == 0){
					//float d = prob;//sqrt(diffmat(i,0)*diffmat(i,0) + diffmat(i,1)*diffmat(i,1)+diffmat(i,2)*diffmat(i,2));
					//printf("%f\n",d);
					//d *= 100;
					//viewer->addLine<pcl::PointXYZRGB>(p0->getPCLPoint(new_pose,0,0,0), p1->getPCLPoint(0,0,0),d,d,d,buf);
					//viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, buf );
				//}

				cloud->points.push_back(p0->getPCLPoint(tr,255*w,255*w,255*w));
				//cloud->points.push_back(p1->getPCLPoint(0,255,0));
			}

			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
			viewer->spin();
*/
		}
	}


		
		//for(unsigned int i = 0; i < matchpoints.size(); i++){tfc.add(points.at(i)->getEIGEN(),matchpoints.at(i)->getEIGEN(),probs.at(i)/scaling.at(i)/scaling.at(i));}
		//Eigen::Matrix4f change = tfc.getTransformation().matrix().cast<float>();
		//pose *= change;
		
		//exit(0);

/*
		start_time = getCurrentTime();
		probs = weightfunction->getProbs(distances,scaling,points,matchpoints);
		if(show){printf("compute weights time: %f\n",getCurrentTime()-start_time);}
		
		start_time = getCurrentTime();
		pcl::TransformationFromCorrespondences tfc;
		for(unsigned int i = 0; i < matchpoints.size(); i++){tfc.add(points.at(i)->getEIGEN(),matchpoints.at(i)->getEIGEN(),probs.at(i)/scaling.at(i)/scaling.at(i));}
		Eigen::Matrix4f change = tfc.getTransformation().matrix().cast<float>();
		pose *= change;
*/
/*
		if(show){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			float mp = 0;for(unsigned int i = 0; i < probs.size(); i++){mp = std::max(mp,probs.at(i)/scaling.at(i));}
			cloud->points.clear();
			for(unsigned int i = 0; i < matchpoints.size(); i++){
				if(i % 1 == 0){cloud->points.push_back(matchpoints.at(i)	->getPCLPoint(0,0,255));}
				cloud->points.push_back(points.at(i)		->getPCLPoint(255 * (probs.at(i)/scaling.at(i))/mp));
			}
			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
			viewer->spin();	
			//if(it % 50 == 0){				viewer->spin();		}
			//else{							viewer->spinOnce();	}
		}
*/
//		for(unsigned int i = 0; i < matchpoints.size(); i++){delete matchpoints.at(i);}

	}
	cout << pose << endl;

	return pose;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model2::getFusedCloud(){
	int r = rand()%256; int g = rand()%256; int b = rand()%256;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(unsigned int k = 0; k < nonfused_points.size(); k++){
		std::vector<Point *> & current = nonfused_points.at(k);
		for(unsigned int i = 0; i < current.size(); i++){
			Point * po = current.at(i);
			pcl::PointXYZRGB p;
			p.x = po->x;
			p.y = po->y;
			p.z = po->z;
			p.r = po->r;
			p.g = po->g;
			p.b = po->b;
			cloud->points.push_back(p);
		}
	}
	return cloud;
}

}
