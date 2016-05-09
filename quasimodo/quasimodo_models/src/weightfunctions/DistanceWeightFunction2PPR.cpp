#include "weightfunctions/DistanceWeightFunction2PPR.h"

namespace reglib{

DistanceWeightFunction2PPR::DistanceWeightFunction2PPR(	double maxd_, int histogram_size_){
	update_size = true;

	start_maxd = maxd_;
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

	startreg = regularization;
	//startmaxd = maxd_;
	noiseval = maxd_;



	threshold = false;
	uniform_bias = true;
	debugg_print = false;
	scale_convergence = true;
	nr_inliers = 1;

	update_size = true;
	target_length = 10.0;
    data_per_bin = 50;
	blur = 0.055;

	maxp = 0.99;

	meanoffset = std::max(0.0,(maxd_ -regularization -noiseval)/target_length);

	bidir = false;
	iter = 0;

	max_under_mean = false;
	interp = false;
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

void blurHistogram(std::vector<float> & blur_hist,  std::vector<float> & hist, float stdval,bool debugg_print = false){
	int nr_data = blur_hist.size();
	int nr_data2 = 2*nr_data;

	std::vector<float> tmphist;
	tmphist.resize(nr_data2);

	std::vector<float> tmphist_blur;
	tmphist_blur.resize(nr_data2);

	for(int i = 0; i < nr_data; i++){
		tmphist[i]			= hist[nr_data-1-i];
		tmphist[nr_data+i]	= hist[i];
		tmphist_blur[i]			= 0;
		tmphist_blur[nr_data+i]	= 0;
	}

	double info = -0.5/(stdval*stdval);
	double weights[nr_data2];
	for(int i = 0; i < nr_data2; i++){
		double current = exp(i*i*info);
		weights[i] = current;
	}

	int offset = 4.0*stdval;
	offset = std::max(4,offset);
	for(int i = 0; i < nr_data2; i++){
		double v = tmphist[i];
		if(v == 0){continue;}

        int start	= std::max(0,i-offset);
        int stop	= std::min(nr_data2,i+offset+1);

		//double sum = 1;
		//for(int j = start; j < stop; j++){sum += weights[abs(i-j)];}
		for(int j = start; j < stop; j++){
			double w = weights[abs(i-j)];
			tmphist_blur[j] += v*w;///sum;
		}
	}
	for(int i = 0; i < nr_data; i++){
		blur_hist[i] = tmphist_blur[i+nr_data];
	}

	float bef = 0;
	float aft = 0;
	for(int i = 0; i < nr_data; i++){
		bef += hist[i];
		aft += blur_hist[i];
	}

	for(int i = 0; i < nr_data; i++){
		blur_hist[i] *= bef/aft;
	}
}

void blurHistogramBidir(std::vector<float> & blur_hist,  std::vector<float> & hist, float stdval,bool debugg_print = false){
	int nr_data = blur_hist.size();
	double info = -0.5/(stdval*stdval);
	double weights[nr_data];
	for(int i = 0; i < nr_data; i++){
		double current = exp(i*i*info);
		weights[i] = current;
	}

	int offset = 4.0*stdval;
	offset = std::max(4,offset);
	for(int i = 0; i < nr_data; i++){
		double v = hist[i];
		if(v == 0){continue;}

		int start	= std::max(0,i-offset);
		int stop	= std::min(nr_data,i+offset+1);

		for(int j = start; j < stop; j++){
			double w = weights[abs(i-j)];
			blur_hist[j] += v*w;///sum;
		}
	}

	float bef = 0;
	float aft = 0;
	for(int i = 0; i < nr_data; i++){
		bef += hist[i];
		aft += blur_hist[i];
	}

	for(int i = 0; i < nr_data; i++){
		blur_hist[i] *= bef/aft;
	}
}

const double step_h = 0.00001;
const unsigned int step_iter = 25;
const double cutoff_exp = -14;


double scoreCurrent(double bias, double mul, double mean, double stddiv, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	double info = -0.5/(stddiv*stddiv);
	double sum = 0;
	double sum2 = 0;
	for(unsigned int i = 0; i < nr_data; i++){
		double dx = X[i] - mean;
		double inp = info*dx*dx;

		if(inp < cutoff_exp){
			sum += Y[i];
		}else{
			double diff = mul*exp(info*dx*dx) - Y[i];
			if(diff > 0){
				sum += 3.0*diff;
			}else{
				sum -= diff;//*diff;
			}
		}

/*
		if(inp < cutoff_exp){
			sum += pow(Y[i],0.9);
		}else{
			double diff = mul*exp(info*dx*dx) - Y[i];
			if(diff > 0){
				sum += 3.0*pow(diff,0.9);
			}else{
				sum += pow(-1*diff,0.9);//*diff;
			}
		}
*/
	}
	return sum;
}



double fitStdval(double bias, double mul, double mean, std::vector<float> & X, std::vector<float> & Y, unsigned int nr_data){
	int iter = 25;

	double ysum = 0;
	for(unsigned int i = 0; i < nr_data; i++){ysum += fabs(Y[i]);}

	double std_mid = 0;
	for(unsigned int i = 0; i < nr_data; i++){std_mid += (X[i]-mean)*(X[i]-mean)*fabs(Y[i]-bias)/ysum;}

	std_mid = sqrt(std_mid);
	double std_max = std_mid*2;
	double std_min = 0;

    for(unsigned int i = 0; i < step_iter; i++){
		std_mid = (std_max+std_min)/2;
		double std_neg = scoreCurrent(bias,mul,mean,std_mid-step_h,X,Y,nr_data);
		double std_pos = scoreCurrent(bias,mul,mean,std_mid+step_h,X,Y,nr_data);

		if(std_neg < std_pos){	std_max = std_mid;}
		else{					std_min = std_mid;}
	}
	return std_mid;
}

Gaussian getModel(double & stdval,std::vector<float> & hist, bool uniform_bias, double mean = -1){
    //bool zeromean = true;
    double mul;
    unsigned int nr_bins = hist.size();
    if(mean < 0){
        mul = hist[0];
        mean = 0;

        for(unsigned int k = 1; k < nr_bins; k++){
            if(hist[k] > mul){
                mul = hist[k];
                mean = k;
            }
        }
    }else{
        mul = hist[int(mean+0.5)];
    }

	std::vector<float> X;
	std::vector<float> Y;
	for(unsigned int k = 0; k < nr_bins; k++){
		if(hist[k]  > mul*0.001){
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

	for(int i = 0; i < 1; i++){
		//if(uniform_bias){bias = fitStdval(bias, mul,mean,X,Y,nr_data_opt);}
        stdval = fitStdval(bias, mul,mean,X,Y,nr_data_opt);
		//mean = fitMean(bias, mul,mean,stdval,X,Y,nr_data_opt);
		//mul = fitMul(bias, mul,mean,stdval,X,Y,nr_data_opt);
	}

	return Gaussian(mul,mean,stdval);
}

double DistanceWeightFunction2PPR::getNoise(){return regularization+noiseval;}// + stdval*double(histogram_size)/maxd;}

void DistanceWeightFunction2PPR::computeModel(MatrixXd mat){
//printf("void DistanceWeightFunction2PPR::computeModel(MatrixXd mat)\n");
//debugg_print = true;
	const unsigned int nr_data = mat.cols();
	const int nr_dim = mat.rows();
	double start_time = getCurrentTime2();

	if(update_size){maxd = (getNoise()+meanoffset)*target_length;}

	int nr_inside = 0;
	for(unsigned int j = 0; j < nr_data; j++){
		for(int k = 0; k < nr_dim; k++){
			if(fabs(mat(k,j)) < maxd){nr_inside++;}
		}
	}

	if(update_size){
		histogram_size = std::min(int(prob.size()),std::max(10,std::min(1000,int(float(nr_inside)/data_per_bin))));
        blurval = blur*double(histogram_size)/double(target_length);
	}

float histogram_mul = 0;
if(!bidir){
	histogram_mul = float(histogram_size)/maxd;
	for(int j = 0; j < histogram_size; j++){histogram[j] = 0;}

	for(unsigned int j = 0; j < nr_data; j++){
		for(int k = 0; k < nr_dim; k++){
			int ind = fabs(mat(k,j))*histogram_mul;
			if(ind >= 0 && ind < histogram_size){histogram[ind]++;}
		}
	}
	/*
	if(!interp){
		for(unsigned int j = 0; j < nr_data; j++){
			for(int k = 0; k < nr_dim; k++){
				int ind = fabs(mat(k,j))*histogram_mul;
				if(ind >= 0 && ind < histogram_size){histogram[ind]++;}
			}
		}
	}else{
		for(unsigned int j = 0; j < nr_data; j++){
			for(int k = 0; k < nr_dim; k++){
				double ind = fabs(mat(k,j))*histogram_mul;
				double w1 = ind-int(ind);
				double w2 = 1-w1;
				if(ind >= 0 && ind < histogram_size){
					histogram[int(ind)]		+=w1;
					histogram[int(ind+1)]	+=w2;
				}
			}
		}
	}
*/
	start_time = getCurrentTime2();
    blurHistogram(blur_histogram,histogram,blurval,debugg_print);
}else{
	histogram_mul = float(histogram_size)/(2.0*maxd);
	for(int j = 0; j < histogram_size; j++){histogram[j] = 0;}
/*
	if(!interp){
		for(unsigned int j = 0; j < nr_data; j++){
			for(int k = 0; k < nr_dim; k++){
				int ind = (mat(k,j)+maxd)*histogram_mul;
				if(ind >= 0 && ind < histogram_size){histogram[ind]++;}
			}
		}
	}else{

		for(unsigned int j = 0; j < nr_data; j++){
			for(int k = 0; k < nr_dim; k++){
				double ind = (mat(k,j)+maxd)*histogram_mul;
				double w1 = ind-int(ind);
				double w2 = 1-w1;
				if(ind >= 0 && ind < histogram_size){
					histogram[int(ind)]		+=w1;
					histogram[int(ind+1)]	+=w2;
				}
			}
		}

	}
*/

	for(unsigned int j = 0; j < nr_data; j++){
		for(int k = 0; k < nr_dim; k++){
			int ind = (mat(k,j)+maxd)*histogram_mul;
			if(ind >= 0 && ind < histogram_size){histogram[ind]++;}
		}
	}

	start_time = getCurrentTime2();
    blurHistogramBidir(blur_histogram,histogram,blurval,debugg_print);
}


    Gaussian g = g = getModel(stdval,blur_histogram,uniform_bias,-1);


	stdval	= g.stdval;
	mulval	= g.mul;
	meanval	= g.mean;

	noiseval = maxd*g.stdval/float(histogram_size);
	if(!bidir){
		meanoffset = maxd*g.mean/float(histogram_size);
	}else{
		meanoffset = 2.0*maxd*g.mean/float(histogram_size) - maxd;
	}
	g.stdval += histogram_size*regularization/maxd;
	g.update();

	for(int k = 0; k < histogram_size; k++){
		blur_histogram[k] += 1.0f;
	}
	
	for(int k = 0; k < histogram_size; k++){	noise[k] = g.getval(k);}

    if(!bidir){
		if(max_under_mean){
			for(int k = 0; k < histogram_size; k++){
				if(k < meanval){prob[k] = maxp;}
				else{
					double hs = blur_histogram[k];
					prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
				}
			}
		}else{
			for(int k = 0; k < histogram_size; k++){
				double hs = blur_histogram[k];
				prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
			}
		}
    }else{
		for(int k = 0; k < histogram_size; k++){
			double hs = blur_histogram[k];
			prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
		}
		/*
		if(max_under_mean){
			for(int k = 0; k < histogram_size; k++){
				if((k-histogram_size/2) < meanval){prob[k] = maxp;}
				else{
					double hs = blur_histogram[k];
					prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
				}
			}
		}else{
			for(int k = 0; k < histogram_size; k++){
				double hs = blur_histogram[k];
				prob[k] = std::min(maxp , noise[k]/hs);//never fully trust any data
			}
		}
		*/
    }
/*
	if(debugg_print){
		for(int k = 0; k < histogram_size; k++){
			double h = blur_histogram[k];
			double n = noise[k];
			double p = std::min(maxp , n/h);//never fully trust any data
			//printf("%i -> h: %f n: %f p: %f\n",k,h,n,p);
		}
	}
*/
	double next_maxd = log2(((getNoise()+meanoffset)*target_length)/maxd);

	//if(debugg_print){printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");}
	if(debugg_print){printf("maxd = %f;\n",maxd);}
	//if(debugg_print){printf("log2(next_maxd) %f\n",next_maxd);}
	if(debugg_print){printf("regularization = %f;\n",regularization);}
	if(debugg_print){printf("estimated = %f;\n",maxd*stdval/float(histogram_size));}
	//if(debugg_print){printf("meanoffset: %f\n",meanoffset);}
    //if(debugg_print){printf("X = [");				for(unsigned int k = 0; k < histogram_size; k++){printf("%5.5f ",double(2*k)*maxd/double(histogram_size) -maxd);}		printf("];\n");}
    if(debugg_print){printf("hist = [");			for(int k = 0; k < 1000 && k < histogram_size; k++){printf("%i ",int(histogram[k]));}		printf("];\n");}
    if(debugg_print){printf("noise = [");			for(int k = 0; k < 1000 && k < histogram_size; k++){printf("%i ",int(noise[k]));}			printf("];\n");}
    if(debugg_print){printf("hist_smooth = [");		for(int k = 0; k < 1000 && k < histogram_size; k++){printf("%i ",int(blur_histogram[k]));}	printf("];\n");}
    if(debugg_print){printf("prob = [");			for(int k = 0; k < 1000 && k < histogram_size; k++){printf("%2.2f ",prob[k]);}				printf("];\n");}
	//if(debugg_print){printf("###############################################################################################################\n");}

	if(update_size ){
		if(fabs(next_maxd) > 0.1 && iter < 100){
			iter++;
			computeModel(mat);
		}else{
			iter = 0;
		}
	}

	//if(bidir){exit(0);}
}

VectorXd DistanceWeightFunction2PPR::getProbs(MatrixXd mat){
	//printf("debugg_print: %i\n",debugg_print);
	//exit(0);
	const unsigned int nr_data = mat.cols();
	const int nr_dim = mat.rows();

	nr_inliers = 0;
	VectorXd weights = VectorXd(nr_data);
	float histogram_mul = 0;
	if(!bidir){
		histogram_mul = float(histogram_size)/maxd;
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
/*
		if(!interp){
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
		}else{
			for(unsigned int j = 0; j < nr_data; j++){
				float inl  = 1;
				float ninl = 1;
				for(int k = 0; k < nr_dim; k++){
					double ind = fabs(mat(k,j))*histogram_mul;
					double w1 = ind-int(ind);
					double w2 = 1-w1;

					float p = 0;
					if(ind >= 0 && ind < histogram_size){
						p = w1*prob[int(ind)] + w2*prob[int(ind+1)];
					}
					inl *= p;
					ninl *= 1.0-p;
				}
				double d = inl / (inl+ninl);
				nr_inliers += d;
				weights(j) = d;
			}
		}
*/
	}else{
		histogram_mul = float(histogram_size)/(2.0*maxd);

		for(unsigned int j = 0; j < nr_data; j++){
			float inl  = 1;
			float ninl = 1;
			for(int k = 0; k < nr_dim; k++){
				int ind = (mat(k,j)+maxd)*histogram_mul;
				float p = 0;
				if(ind >= 0 && ind < histogram_size){p = prob[ind];}
				inl *= p;
				ninl *= 1.0-p;
			}
			double d = inl / (inl+ninl);
			nr_inliers += d;
			weights(j) = d;
		}
		/*
		if(!interp){
			for(unsigned int j = 0; j < nr_data; j++){
				float inl  = 1;
				float ninl = 1;
				for(int k = 0; k < nr_dim; k++){
					int ind = (mat(k,j)+maxd)*histogram_mul;
					float p = 0;
					if(ind >= 0 && ind < histogram_size){p = prob[ind];}
					inl *= p;
					ninl *= 1.0-p;
				}
				double d = inl / (inl+ninl);
				nr_inliers += d;
				weights(j) = d;
			}
		}else{
			for(unsigned int j = 0; j < nr_data; j++){
				float inl  = 1;
				float ninl = 1;
				for(int k = 0; k < nr_dim; k++){
					double ind = (mat(k,j)+maxd)*histogram_mul;
					double w1 = ind-int(ind);
					double w2 = 1-w1;

					float p = 0;
					if(ind >= 0 && ind < histogram_size){
						p = w1*prob[int(ind)] + w2*prob[int(ind+1)];
					}
					inl *= p;
					ninl *= 1.0-p;
				}
				double d = inl / (inl+ninl);
				nr_inliers += d;
				weights(j) = d;
			}
		}
		*/
	}

    if(debugg_print){printf("nr_inliers: %f\n",nr_inliers);}
    //exit(0);
/*
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
*/



	if(threshold){
		for(unsigned int j = 0; j < nr_data; j++){
			weights(j) = weights(j) > 0.5;
		}
	}
	return weights;
}

bool DistanceWeightFunction2PPR::update(){
	//if(debugg_print){printf("###############################################################################################################\n");}
	//if(debugg_print){printf("hist = [");			for(unsigned int k = 0; k < 100; k++){printf("%i ",int(histogram[k]));}		printf("];\n");}
	//if(debugg_print){printf("noise = [");			for(unsigned int k = 0; k < 100; k++){printf("%i ",int(noise[k]));}			printf("];\n");}
	//if(debugg_print){printf("hist_smooth = [");		for(unsigned int k = 0; k < 100; k++){printf("%i ",int(blur_histogram[k]));}	printf("];\n");}
	//if(debugg_print){printf("prob = [");			for(unsigned int k = 0; k < 100; k++){printf("%2.2f ",prob[k]);}				printf("];\n");}
	//if(debugg_print){printf("###############################################################################################################\n");}

	if(true || debugg_print){
		std::vector<float> new_histogram = histogram;
		std::vector<float> new_blur_histogram = blur_histogram;

		float old_sum_prob = 0;
        for(int k = 0; k < histogram_size; k++){old_sum_prob += prob[k] * histogram[k];}

		//double maxp = 0.99;
		Gaussian g = Gaussian(mulval,meanval,stdval);//getModel(stdval,blur_histogram,uniform_bias);

		int iteration = 0;
		while(true){
			iteration++;
            regularization *= 0.5;
			double change = histogram_size*regularization/maxd;
			if(change < 0.001*stdval){return true;}

			g.stdval += change;
			g.update();

			float new_sum_prob = 0;
			for(int k = 0; k < histogram_size; k++){
				double hs = new_blur_histogram[k] +0.0000001;
				new_sum_prob += std::min(maxp , g.getval(k)/hs) * new_histogram[k];
			}

			if(new_sum_prob < 0.999*old_sum_prob){return false;}
			g.stdval -= change;
			g.update();
		}
	}else{
		regularization *= 0.5;
	}
	//exit(0);
}
void DistanceWeightFunction2PPR::reset(){
	regularization = startreg;
	noiseval = start_maxd;
	meanoffset = std::max(0.0,(start_maxd -regularization -noiseval)/target_length);
}

std::string DistanceWeightFunction2PPR::getString(){
	char buf [1024];

	if(startreg != 0){
		sprintf(buf,"PPRreg%15.15ld",long (1000.0*start_maxd));
		//return "PPRreg";
	}else{
		sprintf(buf,"PPR%15.15ld",long (1000.0*start_maxd));
		//return "PPR";
	}
	return std::string(buf);
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


