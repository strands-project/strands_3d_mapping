#include "weightfunctions/DistanceWeightFunction2.h"

namespace reglib
{

DistanceWeightFunction2::DistanceWeightFunction2(){
	p = 0.5;
	f = PNORM;
	convergence_threshold = 0.0001;
}
DistanceWeightFunction2::~DistanceWeightFunction2(){}

void tukey_weight(Eigen::VectorXd& r, double p) {
	for(int i=0; i<r.rows(); ++i) {
		if(r(i) > p) r(i) = 0.0;
		else r(i) = std::pow((1.0 - std::pow(r(i)/p,2.0)), 2.0);
    }
}
void threshold_weight(Eigen::VectorXd& r, double p) {			for(int i=0; i<r.rows(); ++i) {r(i) = fabs(r(i)) < p;}}
void uniform_weight(Eigen::VectorXd& r) {						r = Eigen::VectorXd::Ones(r.rows());}
void pnorm_weight(Eigen::VectorXd& r, double p, double reg) {	for(int i=0; i<r.rows(); ++i) {r(i) = p/(std::pow(r(i),2-p) + reg);}}
void fair_weight(Eigen::VectorXd& r, double p) {				for(int i=0; i<r.rows(); ++i) {r(i) = 1.0/(1.0 + r(i)/p);}}
void logistic_weight(Eigen::VectorXd& r, double p) {			for(int i=0; i<r.rows(); ++i) {r(i) = (p/r(i))*std::tanh(r(i)/p);}}
struct sort_pred { bool operator()(const std::pair<int,double> &left, const std::pair<int,double> &right) {return left.second < right.second;} };

void trimmed_weight(Eigen::VectorXd& r, double p) {
    std::vector<std::pair<int, double> > sortedDist(r.rows());
    for(int i=0; i<r.rows(); ++i) {sortedDist[i] = std::pair<int, double>(i,r(i));}
    std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
    r.setZero();
    int nbV = r.rows()*p;
    for(int i=0; i<nbV; ++i) {r(sortedDist[i].first) = 1.0;}
}

void robust_weight(Function f, Eigen::VectorXd& r, double p) {
    switch(f) {
		case THRESHOLD: threshold_weight(r,p); break;
        case PNORM: pnorm_weight(r,p); break;
        case TUKEY: tukey_weight(r,p); break;
        case FAIR: fair_weight(r,p); break;
        case LOGISTIC: logistic_weight(r,p); break;
        case TRIMMED: trimmed_weight(r,p); break;
        case NONE: uniform_weight(r); break;
        default: uniform_weight(r); break;
    }
}

void DistanceWeightFunction2::computeModel(MatrixXd mat){}
VectorXd DistanceWeightFunction2::getProbs(MatrixXd mat){
	VectorXd W = mat.colwise().norm();
	robust_weight(f, W , p);
	return W;//VectorXf(mat.rows());
}

double DistanceWeightFunction2::getProb(double d){
	printf("double DistanceWeightFunction2::getProbs(double d){ not implemented\n");
	exit(0);
	return 0;
}

double DistanceWeightFunction2::getNoise(){return 1;}
bool DistanceWeightFunction2::update(){return true;}
void DistanceWeightFunction2::reset(){}

std::string DistanceWeightFunction2::getString(){
	std::string ty = "";
	switch(f) {
		case THRESHOLD: ty = "THRESHOLD"; break;
		case PNORM:		ty = "PNORM"; break;
		case TUKEY:		ty = "TUKEY"; break;
		case FAIR:		ty = "FAIR"; break;
		case LOGISTIC:	ty = "LOGISTIC"; break;
		case TRIMMED:	ty = "TRIMMED"; break;
		case NONE:		ty = "NONE"; break;
		default:		ty = "NONE"; break;
	}
	char buf [1024];
	sprintf(buf,"%s%4.4i",ty.c_str(),int(1000.0*p));
	return std::string(buf);
}

double DistanceWeightFunction2::getConvergenceThreshold(){
	return convergence_threshold;
}

}


