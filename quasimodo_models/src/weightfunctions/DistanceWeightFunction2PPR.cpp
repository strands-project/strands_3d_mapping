#include "DistanceWeightFunction2PPR.h"

namespace reglib{

DistanceWeightFunction2PPR::DistanceWeightFunction2PPR(	double maxd_, int histogram_size_){
	regularization		= 0.1;
	maxd 				= maxd_;
	histogram_size		= histogram_size_;
	blurval				= 5;
	stdval				= blurval;
	stdgrow				= 1.1;

	//printf("maxd: %5.5f histogram_size:%i\n",maxd,histogram_size);
	noiseval = 100.0;
	
	prob.resize(histogram_size+1);
	prob[histogram_size] = 0;
	
	histogram.resize(histogram_size+1);
	blur_histogram.resize(histogram_size+1);
	noise.resize(histogram_size+1);

	startreg = 0.05;
	threshold = false;
	uniform_bias = true;
	debugg_print = false;
	scale_convergence = true;
	nr_inliers = 1;
}
DistanceWeightFunction2PPR::~DistanceWeightFunction2PPR(){}

class Gaussian {
	public:
	double mul;
	double mean;
	double stdval;
	double scaledinformation;
	void update(){scaledinformation = -0.5/(stdval*stdval);}
	
	Gaussian(double mul_, double mean_,	double stdval_){
		mul = mul_;
		mean = mean_;
		stdval = stdval_;
		update();
	}
	
	double getval(double x){
		double dx = mean-x;
		return mul*exp(dx*dx*scaledinformation);
	}
};

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

double getCurrentTime2(){
	struct timeval start;
	gettimeofday(&start, NULL);
	double sec = start.tv_sec;
	double usec = start.tv_usec;
	double t = sec+usec*0.000001;
	return t;
}


//multiple optimization obvious
void blurHistogram(std::vector<float> & blur_hist,  std::vector<float> & hist, float stdval){
	int nr_data = blur_hist.size();
	double info = -0.5/(stdval*stdval);
	
	double weights[nr_data];
	for(int i = 0; i < nr_data; i++){weights[i] = exp(i*i*info);}
	
	int offset = 3.0*stdval;
	offset = std::max(3,offset);

	for(int i = 0; i < nr_data; i++){
		double sumV = 0;
		double sumW = 0;
		int start	= std::max(0,i-offset);
		int stop	= std::min(nr_data,i+offset+1);
		for(int j = start; j < stop; j++){
			double w = weights[abs(i-j)];//exp(dx*dx*info);
			double v = hist[j];
			sumV += w*v;
			sumW += w;
		}
		blur_hist[i] = sumV/sumW;
	}
}


double scoreCurrent(double bias, double mul, double mean, double stddiv, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	double info = -0.5/(stddiv*stddiv);
	double sum = 0;
	for(unsigned int i = 0; i < nr_data; i++){
		double dx = X[i] - mean;
		double diff = mul*exp(info*dx*dx) - Y[i] - bias;
		if(diff > 0){
			sum += 3.0*diff;
		}else{
			sum -= diff;//*diff;
		}
	}
	return sum;
}

double fitStdval(double bias, double mul, double mean, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	int iter = 25;
	double h = 0.000000001;

	double ysum = 0;
	for(unsigned int i = 0; i < nr_data; i++){
		ysum += fabs(Y[i]);
	}

	double std_mid = 0;
	for(unsigned int i = 0; i < nr_data; i++){
		std_mid += (X[i]-mean)*(X[i]-mean)*fabs(Y[i]-bias)/ysum;
	}
	std_mid = sqrt(std_mid);
	double std_max = std_mid*2;
	double std_min = 0;

	for(int i = 0; i < iter; i++){
		std_mid = (std_max+std_min)/2;
		double std_neg = scoreCurrent(bias,mul,mean,std_mid-h,X,Y,nr_data);
		double std_pos = scoreCurrent(bias,mul,mean,std_mid+h,X,Y,nr_data);

		if(std_neg < std_pos){
			std_max = std_mid;
		}else{
			std_min = std_mid;
		}
	}
	return std_mid;
}

double fitBias(double bias, double mul, double mean, double std_mid, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	int iter = 25;
	double h = 0.000000001;


	double bias_max = bias*2;
	double bias_min = 0;

	for(int i = 0; i < iter; i++){
		bias = (bias_max+bias_min)/2;
		double std_neg = scoreCurrent(bias-h,mul,mean,std_mid,X,Y,nr_data);
		double std_pos = scoreCurrent(bias+h,mul,mean,std_mid,X,Y,nr_data);

		if(std_neg < std_pos){
			bias_max = bias;
		}else{
			bias_min = bias;
		}
	}
	return bias;
}

double fitMean(double bias,double mul, double mean, double std_mid, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	int iter = 25;
	double h = 0.000000001;
	double mean_max = mean*2;
	double mean_min = 0;

	for(int i = 0; i < iter; i++){
		mean = (mean_max+mean_min)/2;
		double std_neg = scoreCurrent(bias,mul,mean-h,std_mid,X,Y,nr_data);
		double std_pos = scoreCurrent(bias,mul,mean+h,std_mid,X,Y,nr_data);

		if(std_neg < std_pos){
			mean_max = mean;
		}else{
			mean_min = mean;
		}
		//printf("max: %f min: %f\n",mean_max,mean_min);
	}
	return mean;
}

double fitMul(double bias, double mul, double mean, double std_mid, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	int iter = 25;
	double h = 0.000000001;
	double mul_max = mul*2;
	double mul_min = 0;

	for(int i = 0; i < iter; i++){
		mul = (mul_max+mul_min)/2;
		double std_neg = scoreCurrent(bias,mul-h,mean,std_mid,X,Y,nr_data);
		double std_pos = scoreCurrent(bias,mul+h,mean,std_mid,X,Y,nr_data);

		if(std_neg < std_pos){
			mul_max = mul;
		}else{
			mul_min = mul;
		}
		//printf("max: %f min: %f\n",mul_max,mul_min);
	}
	return mul;
}

Gaussian getModel(double & stdval,std::vector<float> & hist, bool uniform_bias){
	double mul = hist[0];
	double mean = 0;
	unsigned int nr_bins = hist.size();
	
	for(unsigned int k = 1; k < nr_bins; k++){
		if(hist[k] > mul){
			mul = hist[k];
			mean = k;
		}
	}

	std::vector<float> X;
	std::vector<float> Y;
	for(unsigned int k = 0; k < nr_bins; k++){
		if(hist[k]  > mul*0.01){
			X.push_back(k);
			Y.push_back(hist[k]);
		}
	}

	unsigned int nr_data_opt = X.size();
	double bias = 0;
	if(uniform_bias){
		stdval = 0.01;
		double ysum = 0;
		for(unsigned int i = 0; i < nr_data_opt; i++){
			bias += fabs(Y[i]);
		}
		bias /= nr_data_opt;
	}

	for(int i = 0; i < 3; i++){
		if(uniform_bias){
			bias = fitStdval(bias, mul,mean,X,Y,nr_data_opt);
			//printf("%i -> %f\n",i,scoreCurrent(bias,mul,mean,stdval,X,Y,nr_data_opt));
			//printf("%f %f %f %f\n",bias,mul,mean,stdval);
		}
		stdval = fitStdval(bias, mul,mean,X,Y,nr_data_opt);
		//printf("%i -> %f\n",i,scoreCurrent(bias,mul,mean,stdval,X,Y,nr_data_opt));
		mean = fitMean(bias, mul,mean,stdval,X,Y,nr_data_opt);
		//printf("%i -> %f\n",i,scoreCurrent(bias,mul,mean,stdval,X,Y,nr_data_opt));
		mul = fitMul(bias, mul,mean,stdval,X,Y,nr_data_opt);
		//printf("%f %f %f %f\n",bias,mul,mean,stdval);
		//printf("%i -> %f\n",i,scoreCurrent(bias,mul,mean,stdval,X,Y,nr_data_opt));
	}
	//exit(0);

	/*

	
	Problem problem;
	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;//SPARSE_QR;
	options.minimizer_progress_to_stdout = false;
	options.function_tolerance = 0.00000001;
	for(unsigned int k = 0; k < nr_bins; k++){
		if(hist[k]  > mul*0.01){
			printf("k:%i hist:%f\n",k,hist[k]);
			CostFunction* cost_function = new ceres::NumericDiffCostFunction<gaussian2, ceres::CENTRAL, 1, 1>(new gaussian2(mean,mul,k,hist[k]));
			problem.AddResidualBlock(cost_function,NULL,&stdval);
		}
	}

	printf("stdval: %f\n",stdval);
	Solver::Summary summary;
	options.minimizer_progress_to_stdout = true;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	stdval = fabs(stdval);
*/
//	printf("stdval: %f\n",stdval);
	return Gaussian(mul,mean,stdval);
}

double DistanceWeightFunction2PPR::getNoise(){return regularization+noiseval;}// + stdval*double(histogram_size)/maxd;}

void DistanceWeightFunction2PPR::computeModel(MatrixXd mat){
	//printf("start DistanceWeightFunction2PPR::computeModel\n");
	//if(debugg_print){printf("start DistanceWeightFunction2PPR::computeModel\n");}
	const unsigned int nr_data = mat.cols();
	const int nr_dim = mat.rows();
	//if(debugg_print){printf("dims: %i %i\n",nr_data,nr_dim);}
	//if(debugg_print){printf("histogram_size: %i\n",histogram_size);exit(0);}
	double start_time = getCurrentTime2();
	const float histogram_mul = float(histogram_size)/maxd;
	for(int j = 0; j < histogram_size; j++){histogram[j] = 0;}
	for(unsigned int j = 0; j < nr_data; j++){
		for(int k = 0; k < nr_dim; k++){
			int ind = fabs(mat(k,j))*histogram_mul;
			if(ind >= 0 && ind < histogram_size){histogram[ind]++;}
			//if(debugg_print){printf("%i -> %6.6f : ratio: %6.6f histogram_size: %6.6f found: %i\n",nr_dim*j+k,fabs(mat(k,j)),fabs(mat(k,j))/maxd,float(histogram_size),ind);}
		}
	}

	start_time = getCurrentTime2();
	blurHistogram(blur_histogram,histogram,blurval);
	
	Gaussian g = getModel(stdval,blur_histogram,uniform_bias);
	noiseval = maxd*g.stdval/float(histogram_size);
	stdval = g.stdval;
	g.stdval += histogram_size*regularization/maxd;
	g.update();

	//printf("mul: %f mean: %f stdval: %f g.stdval: %f\n",g.mul,g.mean,stdval,g.stdval);
	//if(debugg_print){printf("mul: %f mean: %f stdval: %f g.stdval: %f\n",g.mul,g.mean,stdval,g.stdval);}
	
	for(int k = 0; k < histogram_size; k++){	noise[k] = g.getval(k);}
	
	double maxp = 0.99;
	
	for(int k = 0; k < histogram_size; k++){
		if(k < g.mean){	prob[k] = maxp;	}
		else{
			double hs = blur_histogram[k] +0.0000001;
			prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
		}
	}

	if(debugg_print){printf("###############################################################################################################\n");}
	if(debugg_print){printf("hist = [");			for(unsigned int k = 0; k < 100 && k < histogram_size; k++){printf("%i ",int(histogram[k]));}		printf("];\n");}
	if(debugg_print){printf("noise = [");			for(unsigned int k = 0; k < 100 && k < histogram_size; k++){printf("%i ",int(noise[k]));}			printf("];\n");}
	if(debugg_print){printf("hist_smooth = [");		for(unsigned int k = 0; k < 100 && k < histogram_size; k++){printf("%i ",int(blur_histogram[k]));}	printf("];\n");}
	if(debugg_print){printf("prob = [");			for(unsigned int k = 0; k < 100 && k < histogram_size; k++){printf("%2.2f ",prob[k]);}				printf("];\n");}
	if(debugg_print){printf("###############################################################################################################\n");}
	if(debugg_print){printf("end DistanceWeightFunction2PPR::computeModel\n");exit(0);}

}

VectorXd DistanceWeightFunction2PPR::getProbs(MatrixXd mat){
	const unsigned int nr_data = mat.cols();
	const int nr_dim = mat.rows();
	const float histogram_mul = float(histogram_size)/maxd;

	nr_inliers = 0;
	VectorXd weights = VectorXd(nr_data);
	for(unsigned int j = 0; j < nr_data; j++){
		float inl  = 1;
		float ninl = 1;
		for(int k = 0; k < nr_dim; k++){
			int ind = fabs(mat(k,j))*histogram_mul;
			float p = 0;
			if(ind >= 0 && ind < histogram_size){p = prob[ind];}
			inl *= p;
			ninl *= 1.0-p;
		}
		double d = inl / (inl+ninl);
		nr_inliers += d;
		weights(j) = d;
	}
	//printf("sum: %f\n",sum);

	if(threshold){
		for(unsigned int j = 0; j < nr_data; j++){
			weights(j) = weights(j) > 0.5;
		}
	}

/*
	printf("----------------------------------------------------------\n");
	for(unsigned int j = 0; j < nr_data; j+=500){
		printf("%6.6i -> ",j);
		for(int k = 0; k < nr_dim; k++){printf(" %5.5f",mat(k,j));}
		printf(" -> weight %5.5f",weights(j));
		printf("\n");
	}
	printf("----------------------------------------------------------\n");
*/
	return weights;
}

void DistanceWeightFunction2PPR::update(){


	if(debugg_print){printf("###############################################################################################################\n");}
	if(debugg_print){printf("hist = [");			for(unsigned int k = 0; k < 100; k++){printf("%i ",int(histogram[k]));}		printf("];\n");}
	if(debugg_print){printf("noise = [");			for(unsigned int k = 0; k < 100; k++){printf("%i ",int(noise[k]));}			printf("];\n");}
	if(debugg_print){printf("hist_smooth = [");		for(unsigned int k = 0; k < 100; k++){printf("%i ",int(blur_histogram[k]));}	printf("];\n");}
	if(debugg_print){printf("prob = [");			for(unsigned int k = 0; k < 100; k++){printf("%2.2f ",prob[k]);}				printf("];\n");}
	if(debugg_print){printf("###############################################################################################################\n");}

	if(true || debugg_print){
		std::vector<float> new_prob = prob;
		std::vector<float> new_histogram = histogram;
		std::vector<float> new_blur_histogram = blur_histogram;
		std::vector<float> new_noise = noise;
		float old_sum_prob = 0;
		float old_sum = 0;
		for(unsigned int k = 0; k < histogram_size; k++){
			old_sum_prob += prob[k] * histogram[k];
			old_sum += histogram[k];
		}

		old_sum_prob/= old_sum;

		Gaussian g = getModel(stdval,blur_histogram,uniform_bias);


		while(true){

			//printf("running regularization loop with regularization = %6.6f\n",regularization);
			regularization *= 0.75;
			double change = histogram_size*regularization/maxd;
			//if(change < 0.01*stdval){printf("break becouse of convergence\n");break;}
			if(change < 0.01*stdval){break;}

			g.stdval += change;
			g.update();

			for(int k = 0; k < histogram_size; k++){	new_noise[k] = g.getval(k);}

			double maxp = 0.99;

			for(int k = 0; k < histogram_size; k++){
				if(k < g.mean){	new_prob[k] = maxp;	}
				else{
					double hs = new_blur_histogram[k] +0.0000001;
					prob[k] = std::min(maxp , new_noise[k]/hs);//never fully trust any data
				}
			}

			float new_sum_prob = 0;
			float new_sum = 0;
			for(unsigned int k = 0; k < histogram_size; k++){
				new_sum_prob += new_prob[k] * new_histogram[k];
				new_sum += histogram[k];
			}

			new_sum_prob/= new_sum;

			//printf("new_sum_prob: %15.15f old_sum_prob: %15.15f\n",new_sum_prob,old_sum_prob);

			//if(new_sum_prob < 0.95*old_sum_prob){printf("break becouse regularization had effect\n");}
			if(new_sum_prob < 0.95*old_sum_prob){break;}

			g.stdval -= change;
			g.update();
			//break;
		}
	}else{
		regularization *= 0.75;
	}
	//exit(0);
}
void DistanceWeightFunction2PPR::reset(){	regularization = startreg;}

std::string DistanceWeightFunction2PPR::getString(){
	if(startreg != 0){
		return "PPRreg";
	}else{
		return "PPR";
	}
}

double DistanceWeightFunction2PPR::getConvergenceThreshold(){

	//double change = histogram_size*regularization/maxd;
	//if(change < 0.01*stdval){printf("break becouse of convergence\n");break;}
	//if(change < 0.01*stdval){break;}
	if(scale_convergence){
		return convergence_threshold*(regularization + maxd*stdval/double(histogram_size))/sqrt(nr_inliers);
	}else{
		return convergence_threshold;
	}
	//return convergence_threshold;
}
}


