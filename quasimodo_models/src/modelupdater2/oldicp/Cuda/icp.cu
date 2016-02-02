#include "internal.h"
#include "vector_math.hpp"
#include "containers/safe_call.hpp"
//#include "cuPrintf.cu"

#include <stdio.h>

#if __CUDA_ARCH__ < 300
__inline__ __device__
float __shfl_down(float val, int offset, int width = 32)
{
    static __shared__ float shared[MAX_THREADS];
    int lane = threadIdx.x % 32;
    shared[threadIdx.x] = val;
    __syncthreads();
    val = (lane + offset < width) ? shared[threadIdx.x + offset] : 0;
    __syncthreads();
    return val;
}
#endif

#if __CUDA_ARCH__ < 350
template<typename T>
__device__ __forceinline__ T __ldg(const T* ptr)
{
    return *ptr;
}
#endif

__inline__  __device__ jtjjtr warpReduceSum(jtjjtr val)
{
    for(int offset = warpSize / 2; offset > 0; offset /= 2)
    {
        val.aa += __shfl_down(val.aa, offset);
        val.ab += __shfl_down(val.ab, offset);
        val.ac += __shfl_down(val.ac, offset);
        val.ad += __shfl_down(val.ad, offset);
        val.ae += __shfl_down(val.ae, offset);
        val.af += __shfl_down(val.af, offset);
        val.ag += __shfl_down(val.ag, offset);

        val.bb += __shfl_down(val.bb, offset);
        val.bc += __shfl_down(val.bc, offset);
        val.bd += __shfl_down(val.bd, offset);
        val.be += __shfl_down(val.be, offset);
        val.bf += __shfl_down(val.bf, offset);
        val.bg += __shfl_down(val.bg, offset);

        val.cc += __shfl_down(val.cc, offset);
        val.cd += __shfl_down(val.cd, offset);
        val.ce += __shfl_down(val.ce, offset);
        val.cf += __shfl_down(val.cf, offset);
        val.cg += __shfl_down(val.cg, offset);

        val.dd += __shfl_down(val.dd, offset);
        val.de += __shfl_down(val.de, offset);
        val.df += __shfl_down(val.df, offset);
        val.dg += __shfl_down(val.dg, offset);

        val.ee += __shfl_down(val.ee, offset);
        val.ef += __shfl_down(val.ef, offset);
        val.eg += __shfl_down(val.eg, offset);

        val.ff += __shfl_down(val.ff, offset);
        val.fg += __shfl_down(val.fg, offset);

        val.residual += __shfl_down(val.residual, offset);
        val.inliers += __shfl_down(val.inliers, offset);
    }

    return val;
}

__inline__  __device__ jtjjtr blockReduceSum(jtjjtr val)
{
    static __shared__ jtjjtr shared[32];

    int lane = threadIdx.x % warpSize;

    int wid = threadIdx.x / warpSize;

    val = warpReduceSum(val);

    //write reduced value to shared memory
    if(lane == 0)
    {
        shared[wid] = val;
    }
    __syncthreads();

    const jtjjtr zero = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0};

    //ensure we only grab a value from shared memory if that warp existed
    val = (threadIdx.x < blockDim.x / warpSize) ? shared[lane] : zero;

    if(wid == 0)
    {
        val = warpReduceSum(val);
    }

    return val;
}

__global__ void reduceSum(jtjjtr * in, jtjjtr * out, int N)
{
    jtjjtr sum = {0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0};

    for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x)
    {
        sum.add(in[i]);
    }

    sum = blockReduceSum(sum);

    if(threadIdx.x == 0)
    {
        out[blockIdx.x] = sum;
    }
}

__device__ __forceinline__ float tukey_weight(float res, float p) {
	if(res > p){return 0.0f;}
	else{		return __powf((1.0f - __powf(res/p,2.0f)), 2.0f);}
}
__device__ __forceinline__ float uniform_weight(	float res) {						return 1.0f;}
__device__ __forceinline__ float pnorm_weight(		float res, float p, float reg) {	return p/(__powf(res,2-p) + reg);}
__device__ __forceinline__ float fair_weight(		float res, float p) {				return 1.0f/(1.0f + res/p);}
__device__ __forceinline__ float logistic_weight(	float res, float p) {				return (p/res)*tanhf(res/p);}
__device__ __forceinline__ float threshold_weight(	float res, float p) {				return res < p;}
__device__ __forceinline__ float robust_weight(int f, float res, float p, float reg) {
    switch(f) {
        case 0: return uniform_weight(res);
        case 1: return pnorm_weight(res,p, reg);
        case 2: return tukey_weight(res,p);
        case 3: return fair_weight(res,p);
        case 4: return logistic_weight(res,p);
        case 5: return threshold_weight(res,p);
        default: return uniform_weight(res);
    }
}

/*
		float v00x = __ldg(&vmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+0]);
		float v01x = __ldg(&vmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+1]);
		float v10x = __ldg(&vmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+0]);
		float v11x = __ldg(&vmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+1]);

		if (isnan(v00x) || isnan(v01x) || isnan(v10x) || isnan(v11x)){return false;}
		
		float v00y = __ldg(&vmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+0]);
		float v01y = __ldg(&vmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+1]);
		float v10y = __ldg(&vmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+0]);
		float v11y = __ldg(&vmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+1]);
		
		float v00z = __ldg(&vmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+0]);
		float v01z = __ldg(&vmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+1]);
		float v10z = __ldg(&vmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+0]);
		float v11z = __ldg(&vmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+1]);
	
        float tx = xf-floor(xf);
        float ty = yf-floor(yf);
        
        float w00 = 1;//(1-ty)*(1-tx);
        float w01 = 0;//(1-ty)*(  tx);
        float w10 = 0;//(  ty)*(1-tx);
        float w11 = 0;//(  ty)*(  tx);
        
        //cuPrintf("%f %f (%f %f) -> %f %f %f %f\n",xf,yf,tx,ty,w00,w01,w10,w11);
        

		vprev_g.x = w00*v00x + w01*v01x + w10*v10x + w11*v11x;
		vprev_g.y = w00*v00y + w01*v01y + w10*v10y + w11*v11y;
		vprev_g.z = w00*v00z + w01*v01z + w10*v10z + w11*v11z;
*/
/*
        vprev_g.x = w00*__ldg(&vmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+0]) + 
        			w01*__ldg(&vmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+1]) +
        			w10*__ldg(&vmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+0]) + 
        			w11*__ldg(&vmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+1]);
        			
        vprev_g.y = w00*__ldg(&vmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+0]) + 
        			w01*__ldg(&vmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+1]) +
        			w10*__ldg(&vmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+0]) + 
        			w11*__ldg(&vmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+1]);
        			
        vprev_g.y = w00*__ldg(&vmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+0]) + 
        			w01*__ldg(&vmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+1]) +
        			w10*__ldg(&vmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+0]) + 
        			w11*__ldg(&vmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+1]);
*/ 			


/*
        nprev_g.x = w00*__ldg(&nmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+0]) + 
        			w01*__ldg(&nmap_g_prev.ptr (ukr.y+r0+0)[ukr.x+1]) +
        			w10*__ldg(&nmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+0]) + 
        			w11*__ldg(&nmap_g_prev.ptr (ukr.y+r0+1)[ukr.x+1]);
        			
        nprev_g.y = w00*__ldg(&nmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+0]) + 
        			w01*__ldg(&nmap_g_prev.ptr (ukr.y+r1+0)[ukr.x+1]) +
        			w10*__ldg(&nmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+0]) + 
        			w11*__ldg(&nmap_g_prev.ptr (ukr.y+r1+1)[ukr.x+1]);
        			
        nprev_g.y = w00*__ldg(&nmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+0]) + 
        			w01*__ldg(&nmap_g_prev.ptr (ukr.y+r2+0)[ukr.x+1]) +
        			w10*__ldg(&nmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+0]) + 
        			w11*__ldg(&nmap_g_prev.ptr (ukr.y+r2+1)[ukr.x+1]);
*/

//TODO subpixel...
//TODO PPR
	// TODO atomic to build histogram is slow and stupid atm
	// TODO smoothing of histogram is rediculusly slow and stupid atm
//TODO COLOR
struct ICPReduction
{
    Mat33 Rcurr;
    float3 tcurr;

    PtrStep<float> vmap_curr;
    PtrStep<float> nmap_curr;

    Mat33 Rprev_inv;
    float3 tprev;

    Intr intr;

    PtrStep<float> vmap_g_prev;
    PtrStep<float> nmap_g_prev;

    float distThres;
    float angleThres;

    int cols;
    int rows;
    int N;
    
    int distanceFunctionType;
    float pval;
    float rval;
    
    unsigned int histogram_size;
    float maxd;
    
    jtjjtr * out;

   __device__ __forceinline__ bool search (int & x, int & y, float3& n, float3& d, float3& s, float & noise) const {
        float3 vcurr;
        vcurr.x = vmap_curr.ptr (y       )[x];
        vcurr.y = vmap_curr.ptr (y + rows)[x];
        vcurr.z = vmap_curr.ptr (y + 2 * rows)[x];
        
        noise = vcurr.z*vcurr.z;

        float3 vcurr_g = Rcurr * vcurr + tcurr;
        float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev);         // prev camera coo space
        
        float xf = vcurr_cp.x * intr.fx / vcurr_cp.z + intr.cx;
        float yf = vcurr_cp.y * intr.fy / vcurr_cp.z + intr.cy;

        int2 ukr;         //projection
        ukr.x = __float2int_rn (xf);
        ukr.y = __float2int_rn (yf);

        if(xf < 0 || yf < 0 || xf >= (cols-1) || yf >= (rows-1) || vcurr_cp.z < 0)
            return false;

        float3 ncurr;
        ncurr.x = nmap_curr.ptr (y				)[x];
        ncurr.y = nmap_curr.ptr (y + 		rows)[x];
        ncurr.z = nmap_curr.ptr (y + 2 * 	rows)[x];
        
        float3 ncurr_g = Rcurr * ncurr;        
        
        const int r0 = 0;
        const int r1 = rows;
        const int r2 = 2*rows;
        
        float3 vprev_g, nprev_g;
		vprev_g.x = __ldg(&vmap_g_prev.ptr (ukr.y + r0)[ukr.x]);
		vprev_g.y = __ldg(&vmap_g_prev.ptr (ukr.y + r1)[ukr.x]);
		vprev_g.z = __ldg(&vmap_g_prev.ptr (ukr.y + r2)[ukr.x]);

        nprev_g.x = __ldg(&nmap_g_prev.ptr (ukr.y			)[ukr.x]);
        nprev_g.y = __ldg(&nmap_g_prev.ptr (ukr.y + 	rows)[ukr.x]);
        nprev_g.z = __ldg(&nmap_g_prev.ptr (ukr.y + 2 * rows)[ukr.x]);

        float dist = norm (vprev_g - vcurr_g)/noise;
        float sine = norm (cross (ncurr_g, nprev_g));

        n = nprev_g;
        d = vprev_g;
        s = vcurr_g;

        return (sine < angleThres && !isnan (ncurr.x) && !isnan (nprev_g.x));//(sine < angleThres && dist <= distThres && !isnan (ncurr.x) && !isnan (nprev_g.x));
    }
    

    __device__ __forceinline__ jtjjtr getProducts(int & i) const {
        int y = i / cols;
        int x = i - (y * cols);
        
        float noise;

        float3 n_cp, d_cp, s_cp;

        bool found_coresp = search (x, y, n_cp, d_cp, s_cp,noise);
        
        float noisemul = 0;

        float row[7] = {0, 0, 0, 0, 0, 0, 0};

        if(found_coresp){
            s_cp = Rprev_inv * (s_cp - tprev);         // prev camera coo space
            d_cp = Rprev_inv * (d_cp - tprev);         // prev camera coo space
            n_cp = Rprev_inv * (n_cp);                // prev camera coo space

            *(float3*)&row[0] = n_cp;
            *(float3*)&row[3] = cross (s_cp, n_cp);
            
            float res = dot (n_cp, s_cp - d_cp) ;
            row[6] = res;

            float weight = robust_weight(distanceFunctionType, fabs(res/noise), pval, rval);
            noisemul = weight/(noise*noise);
        }

        jtjjtr values = {noisemul * row[0] * row[0],
                         noisemul * row[0] * row[1],
                         noisemul * row[0] * row[2],
                         noisemul * row[0] * row[3],
                         noisemul * row[0] * row[4],
                         noisemul * row[0] * row[5],
                         noisemul * row[0] * row[6],

                         noisemul * row[1] * row[1],
                         noisemul * row[1] * row[2],
                         noisemul * row[1] * row[3],
                         noisemul * row[1] * row[4],
                         noisemul * row[1] * row[5],
                         noisemul * row[1] * row[6],

                         noisemul * row[2] * row[2],
                         noisemul * row[2] * row[3],
                         noisemul * row[2] * row[4],
                         noisemul * row[2] * row[5],
                         noisemul * row[2] * row[6],

                         noisemul * row[3] * row[3],
                         noisemul * row[3] * row[4],
                         noisemul * row[3] * row[5],
                         noisemul * row[3] * row[6],

                         noisemul * row[4] * row[4],
                         noisemul * row[4] * row[5],
                         noisemul * row[4] * row[6],

                         noisemul * row[5] * row[5],
                         noisemul * row[5] * row[6],

                         noisemul * row[6] * row[6],
                         noisemul * float(found_coresp)};

        return values;
    }

    __device__ __forceinline__ void operator () () const {
        jtjjtr sum = {0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0};
        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x){
            jtjjtr val = getProducts(i);
            sum.add(val);
        }
        sum = blockReduceSum(sum);

        if(threadIdx.x == 0){out[blockIdx.x] = sum;}
    }
    
    __device__ __forceinline__ void getHistogram(float * histogram) const {
    
        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x){
		    int y = i / cols;
		    int x = i - (y * cols);

		    float noise;
		    float3 n_cp, d_cp, s_cp;

		    bool found_coresp = search (x, y, n_cp, d_cp, s_cp,noise);

		    if(found_coresp){
		        s_cp = Rprev_inv * (s_cp - tprev);         // prev camera coo space
		        d_cp = Rprev_inv * (d_cp - tprev);         // prev camera coo space
		        n_cp = Rprev_inv * (n_cp);                // prev camera coo space
		        float res = dot (n_cp, s_cp - d_cp) ;
		        
		    	int binid = float(histogram_size)*(fabs(res/noise)/maxd);
		    	if(binid < histogram_size){
		    		atomicAdd(&histogram[binid], 1.0f);
		    	}
		    }
        }
    }
    
    __device__ __forceinline__ void smoothHistogram(float * histogram, float * histogram_smooth, float * weights) const {
    	int tid = blockIdx.x * blockDim.x + threadIdx.x;
		if(tid >= histogram_size){return ;}
		
		double sumV = 0;
		double sumW = 0;
		int start	= 0;//std::max(0,i-offset);
		int stop	= histogram_size;//std::min(nr_data,i+offset);
		for(int j = start; j < stop; j++){
			double w = weights[abs(tid-j)];//exp(dx*dx*info);
			double v = histogram[j];
			sumV += w*v;
			sumW += w;
		}
		histogram_smooth[tid] = sumV/sumW;
    }
    
/*  
    __device__ __forceinline__ void fitHistogram(float maxval, unsigned int maxval_id, float * histogram_smooth) const {
    	int tid = blockIdx.x * blockDim.x + threadIdx.x;
		if(tid >= histogram_size){return ;}
		
		double sumV = 0;
		double sumW = 0;
		int start	= 0;//std::max(0,i-offset);
		int stop	= histogram_size;//std::min(nr_data,i+offset);
		for(int j = start; j < stop; j++){
			double w = weights[abs(tid-j)];//exp(dx*dx*info);
			double v = histogram[j];
			sumV += w*v;
			sumW += w;
		}
		histogram_smooth[tid] = sumV/sumW;
    }
*/
};

__global__ void icpKernel(const ICPReduction icp) { icp(); }
__global__ void histogramKernel(const ICPReduction icp, float * histogram) { icp.getHistogram(histogram); }
__global__ void smoothHistogramKernel(const ICPReduction icp, float * histogram, float * histogram_smooth, float * weights) {
	icp.smoothHistogram(histogram, histogram_smooth, weights);
}

void icpStep(const Mat33& Rcurr,
             const float3& tcurr,
             const DeviceArray2D<float>& vmap_curr,
             const DeviceArray2D<float>& nmap_curr,
             const Mat33& Rprev_inv,
             const float3& tprev,
             const Intr& intr,
             const DeviceArray2D<float>& vmap_g_prev,
             const DeviceArray2D<float>& nmap_g_prev,
             float distThres,
             float angleThres,
             DeviceArray<jtjjtr> & sum,
             DeviceArray<jtjjtr> & out,
             float * matrixA_host,
             float * vectorB_host,
             float * residual_host,
             int threads, int blocks)
{
    int cols = vmap_curr.cols ();
    int rows = vmap_curr.rows () / 3;

    ICPReduction icp;
    icp.rval = 0.0001;
    
//    icp.distanceFunctionType = 1;
//    icp.pval = 0.5;

    icp.distanceFunctionType = 5;
    icp.pval = 0.025;

    icp.Rcurr = Rcurr;
    icp.tcurr = tcurr;

    icp.vmap_curr = vmap_curr;
    icp.nmap_curr = nmap_curr;

    icp.Rprev_inv = Rprev_inv;
    icp.tprev = tprev;

    icp.intr = intr;

    icp.vmap_g_prev = vmap_g_prev;
    icp.nmap_g_prev = nmap_g_prev;

    icp.distThres = distThres;
    icp.angleThres = angleThres;

    icp.cols = cols;
    icp.rows = rows;

    icp.N = cols * rows;
    icp.out = sum;

    unsigned int histogram_size = 1000;
    icp.maxd = 0.05;
    icp.histogram_size = histogram_size;
  
    float stdval = 5;
/*  
    float info = -0.5/(stdval*stdval);
    float * h_weights = new float[histogram_size];
    for(int i = 0; i < histogram_size; i++){h_weights[i] = exp(float(i*i)*info);}

//    for(unsigned int i = 0; i < histogram_size; i++){printf("weights: %i -> %f\n",i,h_weights[i]);}
    
//    float * d_weights;
//    cudaSafeCall(cudaMalloc(&(d_weights),			sizeof(float)*histogram_size));
//	cudaSafeCall(cudaMemcpy(d_weights, h_weights,	sizeof(float)*histogram_size, cudaMemcpyHostToDevice));

    delete[] h_weights;
*/
//    cudaSafeCall(cudaGetLastError());
//    cudaSafeCall(cudaDeviceSynchronize());
//    cudaSafeCall(cudaFree(d_weights));
/*
    
    float * d_histogram;
    float * d_histogram_smooth;
    
    cudaSafeCall(cudaMalloc(&(d_histogram),		sizeof(float)*histogram_size));
    cudaSafeCall(cudaMemset(d_histogram,	0,	sizeof(float)*histogram_size));

    cudaSafeCall(cudaMalloc(&(d_histogram_smooth),	sizeof(float)*histogram_size));
    cudaSafeCall(cudaMemset(d_histogram_smooth,	0,	sizeof(float)*histogram_size));
    
    cudaSafeCall(cudaDeviceSynchronize());
    	
    histogramKernel<<<blocks, threads>>>(icp,d_histogram);
    cudaSafeCall(cudaDeviceSynchronize());

    smoothHistogramKernel<<<3, 512>>>(icp,d_histogram,d_histogram_smooth,d_weights);
    cudaSafeCall(cudaDeviceSynchronize());
    
    float * h_histogram			= new float[histogram_size];
    float * h_histogram_smooth	= new float[histogram_size];
    cudaSafeCall(cudaMemcpy(h_histogram,		d_histogram, 		sizeof(float)*histogram_size, cudaMemcpyDeviceToHost));
	cudaSafeCall(cudaMemcpy(h_histogram_smooth, d_histogram_smooth, sizeof(float)*histogram_size, cudaMemcpyDeviceToHost)); 
	
	unsigned int maxval_id = 0;
	float maxval = h_histogram_smooth[maxval_id];
	for(unsigned int i = 1; i < histogram_size; i++){
		if(h_histogram_smooth[i] > maxval){
			maxval = h_histogram_smooth[i];
			maxval_id = i;
		}
    }
    
    for(unsigned int i = 0; i < histogram_size; i++){
    	printf("%i -> %i , %i\n",i,int(h_histogram[i]),int(h_histogram_smooth[i]));
    }
    
    printf("maxval: %f maxval_id: %i\n",maxval,maxval_id);
    
    float min_error = 1000000000000;
    float min_error_sigma = 1;
    for(float sigma = 1.0; sigma < histogram_size; sigma += 0.1f){
    	double error = 0;
		double isigma2 = -0.5*1.0f/(sigma*sigma);
		for(unsigned int i = 0; i < histogram_size; i++){
			float diff = (i-maxval_id)*(i-maxval_id);
			double e = h_histogram_smooth[i] - maxval*exp(diff*isigma2);
			error += fabs(e);
			//printf("index: %i error: %f histogram: %f estimation: %f e: %f\n",i,error,h_histogram_smooth[i],maxval*exp(diff*isigma2),e);
		}
		if(error < min_error){
			min_error = error;
			min_error_sigma = sigma;
			printf("%f -> %f\n",sigma,error);
		}
		//printf("%f -> %f\n",sigma,error);
		//exit(0);
    }
    exit(0);
//    exit(0);
    

//    exit(0);
    
    
    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());
    cudaSafeCall(cudaFree(d_histogram));
    cudaSafeCall(cudaFree(d_histogram_smooth));
*/
    icpKernel<<<blocks, threads>>>(icp);

    reduceSum<<<1, MAX_THREADS>>>(sum, out, blocks);

    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaDeviceSynchronize());

    float host_data[32];
    out.download((jtjjtr *)&host_data[0]);

    int shift = 0;
    for (int i = 0; i < 6; ++i)  //rows
    {
        for (int j = i; j < 7; ++j)    // cols + b
        {
            float value = host_data[shift++];
            if (j == 6)       // vector b
                vectorB_host[i] = value;
            else
                matrixA_host[j * 6 + i] = matrixA_host[i * 6 + j] = value;
        }
    }

    residual_host[0] = host_data[27];
    residual_host[1] = host_data[28];
}
