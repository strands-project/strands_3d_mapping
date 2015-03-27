#include "strands_sweep_registration/pair3DError.h"

int sumid = 0;

pair3DError::pair3DError(double sw, double sh, double sz,double dw, double dh, double dz, double weight) : sw(sw), sh(sh), sz(sz), dw(dw), dh(dh), dz(dz), weight(weight) \
{
    id = sumid++;
    optimizeCameraParams = false;
    information = 1.0/1.5;
}

pair3DError::~pair3DError() {}

bool pair3DError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
	const double h = 3e-7;
	const double* const scamera = parameters[0];
	const double* const dcamera = parameters[1];
	const double* const params  = parameters[2];

	double sp[3];
	double dp[3];

	double sp2[3];
	double dp2[3];

	double sc[6];
	double dc[6];
	double p [5];

	double smat[12];
	double dmat[12];

	double invfx = 1.0/540.0;
	double invfy = 1.0/540.0;
	double cx = 319.5;
	double cy = 239.5;

    if(optimizeCameraParams){
        invfx = params[0];
        invfy = params[1];
        cx = params[2];
        cy = params[3];
    }

	sp[2]	= sz;
	sp[0]	= (sw-cx) * sp[2] * invfx;
	sp[1]	= (sh-cy) * sp[2] * invfy;

	dp[2]	= dz;
	dp[0]	= (dw-cx) * dp[2] * invfx;
	dp[1]	= (dh-cy) * dp[2] * invfy;

	for(int i = 0; i < 6; i++){
		sc[i] = scamera[i];
		dc[i] = dcamera[i];
	}
	for(int i = 0; i < 4; i++){p[i] = params[i];}

	getMat(sc, smat);
	sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
	sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
	sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

	double sp20 = sp2[0];
	double sp21 = sp2[1];
	double sp22 = sp2[2];

	getMat(dc, dmat);
	dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
	dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
	dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

	residuals[0] = (sp2[0] - dp2[0]);
	residuals[1] = (sp2[1] - dp2[1]);
	residuals[2] = (sp2[2] - dp2[2]);

	double d2 = residuals[0]*residuals[0]+residuals[1]*residuals[1]+residuals[2]*residuals[2];
	double w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);

	residuals[0] *= w;
	residuals[1] *= w;
	residuals[2] *= w;

	if (jacobians != NULL && jacobians[0] != NULL) {
		//Src camera
		for(int i = 0; i < 6; i++){
			sc[i]+=h;
			getMat(sc, smat);
			sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
			sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
			sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

			double r0 = (sp2[0] - dp2[0]);
			double r1 = (sp2[1] - dp2[1]);
			double r2 = (sp2[2] - dp2[2]);
			d2 = r0*r0+r1*r1+r2*r2;
			w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
			r0 *= w;
			r1 *= w;
			r2 *= w;

			sc[i]-=2*h;
			getMat(sc, smat);
			sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
			sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
			sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];

			double r3 = (sp2[0] - dp2[0]);
			double r4 = (sp2[1] - dp2[1]);
			double r5 = (sp2[2] - dp2[2]);
			d2 = r3*r3+r4*r4+r5*r5;
			w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
			r3 *= w;
			r4 *= w;
			r5 *= w;

			jacobians[0][i+0] = (r0-r3)/(2*h);
			jacobians[0][i+6] = (r1-r4)/(2*h);
			jacobians[0][i+12] = (r2-r5)/(2*h);

			sc[i]+=h;
		}

		sp2[0] = sp20;
		sp2[1] = sp21;
		sp2[2] = sp22;
			
		//dst camera
		for(int i = 0; i < 6; i++){
			dc[i]+=h;
			getMat(dc, dmat);
			dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
			dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
			dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

			double r0 = (sp2[0] - dp2[0]);
			double r1 = (sp2[1] - dp2[1]);
			double r2 = (sp2[2] - dp2[2]);
			d2 = r0*r0+r1*r1+r2*r2;
			w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
			r0 *= w;
			r1 *= w;
			r2 *= w;

			dc[i]-=2*h;
			getMat(dc, dmat);
			dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
			dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
			dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

			double r3 = (sp2[0] - dp2[0]);
			double r4 = (sp2[1] - dp2[1]);
			double r5 = (sp2[2] - dp2[2]);
			d2 = r3*r3+r4*r4+r5*r5;
			w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
			r3 *= w;
			r4 *= w;
			r5 *= w;

			jacobians[1][i+0] = (r0-r3)/(2*h);
			jacobians[1][i+6] = (r1-r4)/(2*h);
			jacobians[1][i+12] = (r2-r5)/(2*h);

			dc[i]+=h;	
		}

		if(!optimizeCameraParams){
			for(int i = 0; i < 4; i++){
				jacobians[2][i+0] = 0;
				jacobians[2][i+4] = 0;
				jacobians[2][i+8] = 0;
			}
		}else{
			getMat(sc, smat);
			getMat(dc, dmat);

			for(int i = 0; i < 4; i++){
				p[i]+=h;

				invfx = p[0];
				invfy = p[1];
				cx = p[2];
				cy = p[3];

				sp[0]	= (sw-cx) * sp[2] * invfx;
				sp[1]	= (sh-cy) * sp[2] * invfy;
				dp[0]	= (dw-cx) * dp[2] * invfx;
				dp[1]	= (dh-cy) * dp[2] * invfy;
				sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
				sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
				sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];
				dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
				dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
				dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

				double r0 = (sp2[0] - dp2[0]);
				double r1 = (sp2[1] - dp2[1]);
				double r2 = (sp2[2] - dp2[2]);
				d2 = r0*r0+r1*r1+r2*r2;
				w = weight;// * exp(-0.25*d2*information*information);//*pow(information,7);
				r0 *= w;
				r1 *= w;
				r2 *= w;

				p[i]-=2*h;

				invfx = p[0];
				invfy = p[1];
				cx = p[2];
				cy = p[3];

				sp[0]	= (sw-cx) * sp[2] * invfx;
				sp[1]	= (sh-cy) * sp[2] * invfy;
				dp[0]	= (dw-cx) * dp[2] * invfx;
				dp[1]	= (dh-cy) * dp[2] * invfy;
				sp2[0] = sp[0]*smat[ 0] + sp[1]*smat[ 1] + sp[2]*smat[ 2] + smat[ 3];
				sp2[1] = sp[0]*smat[ 4] + sp[1]*smat[ 5] + sp[2]*smat[ 6] + smat[ 7];
				sp2[2] = sp[0]*smat[ 8] + sp[1]*smat[ 9] + sp[2]*smat[10] + smat[11];
				dp2[0] = dp[0]*dmat[ 0] + dp[1]*dmat[ 1] + dp[2]*dmat[ 2] + dmat[ 3];
				dp2[1] = dp[0]*dmat[ 4] + dp[1]*dmat[ 5] + dp[2]*dmat[ 6] + dmat[ 7];
				dp2[2] = dp[0]*dmat[ 8] + dp[1]*dmat[ 9] + dp[2]*dmat[10] + dmat[11];

				double r3 = (sp2[0] - dp2[0]);
				double r4 = (sp2[1] - dp2[1]);
				double r5 = (sp2[2] - dp2[2]);
				d2 = r3*r3+r4*r4+r5*r5;
				w = weight;// * exp(0.25*d2*information*information);//*pow(information,7);
				r3 *= w;
				r4 *= w;
				r5 *= w;

				jacobians[2][i+0] = (r0-r3)/(2*h);
				jacobians[2][i+4] = (r1-r4)/(2*h);
				jacobians[2][i+8] = (r2-r5)/(2*h);

				p[i]+=h;
			}
		}
	}

	return true;
}

