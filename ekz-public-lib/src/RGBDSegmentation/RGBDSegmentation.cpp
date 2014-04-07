#include "RGBDSegmentation.h"

const int undef = 99999;

using namespace std;

RGBDSegmentation::~RGBDSegmentation(){}
Segments * RGBDSegmentation::segment(FrameInput * fi){return 0;}
void RGBDSegmentation::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

void RGBDSegmentation::RGBDSegmentation::disp_xy_edge(string s , float** xedge,float** yedge, int width,int height, bool stop){
	IplImage * img 			= cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	float * data 			= (float *)(img->imageData);
	float xmax = -10000000000;
	float xmin = 10000000000;	
	float ymax = -10000000000;
	float ymin = 10000000000;
	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			if(xedge[i][j] != undef){
				if(xedge[i][j] < xmin){xmin = xedge[i][j];}
				if(xedge[i][j] > xmax){xmax = xedge[i][j];}
			}
			if(yedge[i][j] != undef){
				if(yedge[i][j] < ymin){ymin = yedge[i][j];}
				if(yedge[i][j] > ymax){ymax = yedge[i][j];}
			}
		}
	}
	xmin = 0;
	ymin = 0;
	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			data[(j * width + i)*3+0] = 0;
			data[(j * width + i)*3+1] = 0;
			data[(j * width + i)*3+2] = 0;
			if(xedge[i][j] != undef){
				//data[(j * width + i)*3+0] = 0;
				data[(j * width + i)*3+1] = (xedge[i][j]-xmin)/(xmax-xmin);
			}else{data[(j * width + i)*3+0] = 1;}
			
			if(yedge[i][j] != undef){
				//data[(j * width + i)*3+0] = 0;
				data[(j * width + i)*3+2] = (yedge[i][j]-ymin)/(ymax-ymin);
			}else{data[(j * width + i)*3+0] = 1;}
		}
	}
	cvNamedWindow(s.c_str(), CV_WINDOW_AUTOSIZE );
	cvShowImage(s.c_str(), img);
	if(stop){	cvWaitKey(0);}
	else{		cvWaitKey(30);}
	cvReleaseImage( &img );
}

void RGBDSegmentation::disp_edge(string s , float** edge, int width,int height, bool stop)
{
    IplImage * img             = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
    float * data             = (float *)(img->imageData);
    float max = -10000000000;
    float min = 10000000000;
    for(int i = 0; i < width; i++)
    {
        for(int j = 0; j < height; j++)
        {
            if(edge[i][j] != undef){
                if(edge[i][j] < min){min = edge[i][j];}
                if(edge[i][j] > max){max = edge[i][j];}
            }
        }
    }
    min = 0;
    for(int i = 0; i < width; i++)
    {
        for(int j = 0; j < height; j++)
        {
            if(edge[i][j] != undef){
                data[(j * width + i)*3+0] = (edge[i][j]-min)/(max-min);
                data[(j * width + i)*3+1] = (edge[i][j]-min)/(max-min);
                data[(j * width + i)*3+2] = (edge[i][j]-min)/(max-min);
            }else{
                data[(j * width + i)*3+0] = 0;
                data[(j * width + i)*3+1] = 0;
                data[(j * width + i)*3+2] = 1;
            }
        }
    }
    cvNamedWindow(s.c_str(), CV_WINDOW_AUTOSIZE );
    cvShowImage(s.c_str(), img);
	if(stop){	cvWaitKey(0);}
	else{		cvWaitKey(30);}
    cvReleaseImage( &img );
}

void RGBDSegmentation::disp_segments(int ** segment_id, int width, int height, bool stop)
{
	srand(0);
    IplImage * img             = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
    float * data             = (float *)(img->imageData);
    float* tmp_red = new float[1000000];
    float* tmp_green = new float[1000000];
    float* tmp_blue = new float[1000000];

	tmp_red[0] = 0;
	tmp_green[0] = 0;
	tmp_blue[0] = 0;
    for(int i = 1; i < 1000000; i ++)
    {
        tmp_red[i] = float(rand()%10000)/10000.0f;
        tmp_green[i] = float(rand()%10000)/10000.0f;
        tmp_blue[i] = float(rand()%10000)/10000.0f;
       
    }
    for(int i = 0; i < width; i++)
    {
        for(int j = 0; j < height; j++)
        {
            int seg = segment_id[i][j]+1;
            data[(j * width + i)*3+0] = tmp_blue[seg];//segment[i][j]+1];
            data[(j * width + i)*3+1] = tmp_green[seg];//segment[i][j]+1];
            data[(j * width + i)*3+2] = tmp_red[seg];//segment[i][j]+1];
        }
    }
    delete[] tmp_red;
    delete[] tmp_green;
    delete[] tmp_blue;
    cvNamedWindow("segments", CV_WINDOW_AUTOSIZE );
    cvShowImage("segments", img);
	if(stop){	cvWaitKey(0);}
	else{		cvWaitKey(30);}
    cvReleaseImage( &img );
}

float ** RGBDSegmentation::getII(float ** input , int width, int height){
	float ** iimg    = new float*[width];
	for(int i = 0; i < width; i++){iimg[i] = new float[height];}

	float v = input[0][0];if(isnan(v)){v=0;}
	iimg[0][0] = v;
	for(int i = 1; i < width; i++){
		float v = input[i][0];if(isnan(v)){v=0;}
		iimg[i][0] = v+iimg[i-1][0];
	}
	for(int j = 1; j < height; j++){
		float v = input[0][j];if(isnan(v)){v=0;}
		iimg[0][j] = v+iimg[0][j-1];
	}

	for(int i = 1; i < width; i++){
		for(int j = 1; j < height; j++){
			float v = input[i][j];if(isnan(v)){v=0;}
			iimg[i][j] = v+iimg[i-1][j]+iimg[i][j-1]-iimg[i-1][j-1];
		}
	}
	return iimg;
}

float ** RGBDSegmentation::getIIValid(float ** input , int width, int height){
	float ** iimg    = new float*[width];
	for(int i = 0; i < width; i++){iimg[i] = new float[height];}

	float v1 = 1;if(isnan(input[0][0])){v1=0;}
	iimg[0][0] = v1;
	for(int i = 1; i < width; i++){
		float v = 1;if(isnan(input[i][0])){v=0;}
		iimg[i][0] = v+iimg[i-1][0];
	}
	for(int j = 1; j < height; j++){
		float v = 1;if(isnan(input[0][j])){v=0;}
		iimg[0][j] = v+iimg[0][j-1];
	}

	for(int i = 1; i < width; i++){
		for(int j = 1; j < height; j++){
			float 			v = 1;
			if(isnan(input[i][j])){	v = 0;}
			iimg[i][j] = v+iimg[i-1][j]+iimg[i][j-1]-iimg[i-1][j-1];
		}
	}
	return iimg;
}

float RGBDSegmentation::getSum(float ** iimg, int topw,int toph, int botw, int both){
	return iimg[botw][both]-iimg[botw][toph]-iimg[topw][both]+iimg[topw][toph];
}

float RGBDSegmentation::getAvg(float ** iimg, int topw,int toph, int botw, int both, int width, int height){

	if(topw < 0){topw = 0;}
	if(botw < 0){botw = 0;}

	if(toph < 0){toph = 0;}
	if(both < 0){both = 0;}

	if(topw >= width){topw = width-1;}
	if(botw >= width){botw = width-1;}

	if(toph >= height){toph = height-1;}
	if(both >= height){both = height-1;}

	float diff_w = topw-botw;
	float diff_h = toph-both;
	float bsize = diff_w*diff_h;

	if(bsize == 0){return -1;}

	float v = iimg[botw][both]-iimg[botw][toph]-iimg[topw][both]+iimg[topw][toph];
	v/=bsize;
	return v;
}

float RGBDSegmentation::getAvg(float ** iimg,float ** iiValid, int topw,int toph, int botw, int both, int width, int height){

	if(topw < 0){topw = 0;}
	if(botw < 0){botw = 0;}

	if(toph < 0){toph = 0;}
	if(both < 0){both = 0;}

	if(topw >= width){topw = width-1;}
	if(botw >= width){botw = width-1;}

	if(toph >= height){toph = height-1;}
	if(both >= height){both = height-1;}

	float diff_w = topw-botw;
	float diff_h = toph-both;
	float bsize = iiValid[botw][both]-iiValid[botw][toph]-iiValid[topw][both]+iiValid[topw][toph];

	//if(diff_w*diff_h != bsize){printf("%f bsize\n",bsize);}
	if(bsize < 1){return -1;}

	float v = iimg[botw][both]-iimg[botw][toph]-iimg[topw][both]+iimg[topw][toph];
	v/=bsize;
	return v;
}

void RGBDSegmentation::getEdges(float ** & wedges, float ** & hedges, float *** ii, int dim , int width, int height,int s1 ,int s2){

	wedges    = new float*[width];
	hedges    = new float*[width];
	for(int i = 0; i < width; i++){
		wedges[i] = new float[height];
		hedges[i] = new float[height];
		for(int j = 0; j < height; j++){
			wedges[i][j] = undef;
			hedges[i][j] = undef;
		}
	}

	for(int i = 10; i < width-10; i++){
		for(int j = 10; j < height-10; j++){
			float wsum = 0;
			float hsum = 0;
			int is1 = i+s1;
			int is2 = i+s2;
			int js1 = j+s1;
			int js2 = j+s2;

			int ims1 = i-s1;
			int jms1 = j-s1;

			int jm1ms2 = j-1-s2;
			int im1ms2 = i-1-s2;
			
			bool ignore = false;
			for(int k = 0; k < dim; k++){
				float wk1 = getSum(ii[k],is1,js2,i,jm1ms2);
				float wk2 = getSum(ii[k],i,js2,ims1,jm1ms2);
				float wk = wk1-wk2;

				float hk1 = getSum(ii[k],is2,js1,im1ms2,j);
				float hk2 = getSum(ii[k],is2,j,im1ms2,jms1);
				float hk = hk1-hk2;
				wsum += wk*wk;
				hsum += hk*hk;
				/*
				float wk1 = getAvg(ii[k],is1,js2,i,jm1ms2,width,height);
				if(wk1 == -1){ignore = true; break;}
				float wk2 = getAvg(ii[k],i,js2,ims1,jm1ms2,width,height);
				if(wk2 == -1){ignore = true; break;}
				float wk = wk1-wk2;
				float hk1 = getAvg(ii[k],is2,js1,im1ms2,j,width,height);
				if(hk1 == -1){ignore = true; break;}
				float hk2 = getAvg(ii[k],is2,j,im1ms2,jms1,width,height);
				if(hk2 == -1){ignore = true; break;}
				float hk = hk1-hk2;
				wsum += wk*wk;
				hsum += hk*hk;
				*/
			}
			wsum = sqrt(wsum);
			hsum = sqrt(hsum);
			if(!ignore){
				wedges[i][j] = wsum/dim;
				hedges[i][j] = hsum/dim;
			}
	    }
	}
}

void RGBDSegmentation::getEdges(float ** & wedges, float ** & hedges, float *** ii, float *** iiValid, int dim , int width, int height,int s1 ,int s2){

	wedges    = new float*[width];
	hedges    = new float*[width];
	for(int i = 0; i < width; i++){
		wedges[i] = new float[height];
		hedges[i] = new float[height];
		for(int j = 0; j < height; j++){
			wedges[i][j] = undef;
			hedges[i][j] = undef;
		}
	}

	for(int i = 0; i < width-1; i++){
		for(int j = 0; j < height-1; j++){
			float wsum = 0;
			float hsum = 0;
			int is1 = i+s1;
			int is2 = i+s2;
			int js1 = j+s1;
			int js2 = j+s2;

			int ims1 = i-s1;
			int jms1 = j-s1;

			int jm1ms2 = j-1-s2;
			int im1ms2 = i-1-s2;
			
			bool ignore = false;
			for(int k = 0; k < dim; k++){
				float wk1 = getAvg(ii[k],iiValid[k],is1,js2,i,jm1ms2,width,height);
				if(wk1 == -1){ignore = true; break;}
				float wk2 = getAvg(ii[k],iiValid[k],i,js2,ims1,jm1ms2,width,height);
				if(wk2 == -1){ignore = true; break;}
				float wk = wk1-wk2;
				float hk1 = getAvg(ii[k],iiValid[k],is2,js1,im1ms2,j,width,height);
				if(hk1 == -1){ignore = true; break;}
				float hk2 = getAvg(ii[k],iiValid[k],is2,j,im1ms2,jms1,width,height);
				if(hk2 == -1){ignore = true; break;}
				float hk = hk1-hk2;
				wsum += wk*wk;
				hsum += hk*hk;
			}
			if(!ignore){
				wedges[i][j] = wsum/dim;
				hedges[i][j] = hsum/dim;
			}
	    }
	}
}



float** RGBDSegmentation::getWEdges(float *** rgb,float *** xyz,float *** nxnynz, int width, int height){
	float ** r = rgb[0];
	float ** g = rgb[1];
	float ** b = rgb[2];

	float ** iir = getII(r,width,height);
	float ** iig = getII(g,width,height);
	float ** iib = getII(b,width,height);

	float ** edges    = new float*[width];
	for(int i = 0; i < width; i++){
		edges[i] = new float[height];
		for(int j = 0; j < height; j++){
			edges[i][j] = undef;
		}
	}

	int sw = 3;
	int sh = 1;
	float div = 1/(float(sw)*(2*float(sh)+1));
	
	for(int i = sw; i < width-sw; i++){
		for(int j = sh+1; j < height-sh; j++){
			float rw = getSum(iir,i+sw,j+sh,i,j-1-sh)-getSum(iir,i,j+sh,i-sw,j-1-sh);
			rw *= div;
			float gw = getSum(iig,i+sw,j+sh,i,j-1-sh)-getSum(iig,i,j+sh,i-sw,j-1-sh);
			gw *= div;
			float bw = getSum(iib,i+sw,j+sh,i,j-1-sh)-getSum(iib,i,j+sh,i-sw,j-1-sh);
			bw *= div;
			edges[i][j] = rw*rw+gw*gw+bw*bw;
	    }
	}
	return edges;
}

float** RGBDSegmentation::getHEdges(float *** rgb,float *** xyz,float *** nxnynz, int width, int height){
	float ** r = rgb[0];
	float ** g = rgb[1];
	float ** b = rgb[2];

	float ** iir = getII(r,width,height);
	float ** iig = getII(g,width,height);
	float ** iib = getII(b,width,height);

	float ** edges    = new float*[width];
	for(int i = 0; i < width; i++){
		edges[i] = new float[height];
		for(int j = 0; j < height; j++){
			edges[i][j] = undef;
		}
	}

	int sw = 1;
	int sh = 3;
	float div = 1/(float(sw)*(2*float(sh)+1));
	
	for(int i = sw+1; i < width-sw; i++){
		for(int j = sh; j < height-sh; j++){
			float rw = getSum(iir,i+sw,j+sh,i-1-sw,j)-getSum(iir,i+sw,j,i-1-sw,j-sh);
			rw *= div;
			float gw = getSum(iig,i+sw,j+sh,i-1-sw,j)-getSum(iig,i+sw,j,i-1-sw,j-sh);
			gw *= div;
			float bw = getSum(iib,i+sw,j+sh,i-1-sw,j)-getSum(iib,i+sw,j,i-1-sw,j-sh);
			bw *= div;
			edges[i][j] = rw*rw+gw*gw+bw*bw;
	    }
	}
	return edges;
}

void RGBDSegmentation::normalize(float ** wedges, float ** hedges, int width, int height){
	float max = -1;
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
			if(!(wedges[i][j] == undef || hedges[i][j] == undef)){
				float e = sqrt(wedges[i][j]+hedges[i][j]);
				if(e > max){max = e;}
			}
        }
    }

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
			if(!(wedges[i][j] == undef || hedges[i][j] == undef)){
				wedges[i][j]/=max;
				hedges[i][j]/=max;
			}
        }
    }
}


float** RGBDSegmentation::mul(float ** edges1, float ** edges2, int width, int height){
    float ** edges    = new float*[width];
    for(int i = 0; i < width; i++){
		edges[i] = new float[height];
        for(int j = 0; j < height; j++){
			if(!(edges1[i][j] == undef || edges2[i][j] == undef)){
				edges[i][j] = edges1[i][j]*edges2[i][j];
			}else{edges[i][j] = undef;}
        }
    }
	return edges;
}


float** RGBDSegmentation::add(float ** edges1, float ** edges2, int width, int height){
    float ** edges    = new float*[width];
    for(int i = 0; i < width; i++){
		edges[i] = new float[height];
        for(int j = 0; j < height; j++){
			if(!(edges1[i][j] == undef || edges2[i][j] == undef)){
				edges[i][j] = edges1[i][j]+edges2[i][j];
			}else{edges[i][j] = undef;}
        }
    }
	return edges;
}

void RGBDSegmentation::trim(float ** edges, int width, int height){
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
			if(!(edges[i][j] == undef)){
				float v = edges[i][j];
				if(v > 0.075){v = 0.075;}
				edges[i][j] = v;
			}
        }
    }
}


float** RGBDSegmentation::mergeEdges(float ** wedges, float ** hedges, int width, int height){
    float ** edges    = new float*[width];
    for(int i = 0; i < width; i++){
		edges[i] = new float[height];
        for(int j = 0; j < height; j++){
			if(!(wedges[i][j] == undef || hedges[i][j] == undef)){
				edges[i][j] = sqrt(wedges[i][j]+hedges[i][j]);
			}else{edges[i][j] = undef;}
        }
    }

	return edges;
}

void RGBDSegmentation::getEdges(float ** & wedges, float ** & hedges, float *** rgb,float *** xyz,float *** nxnynz, int width, int height){

	wedges    = new float*[width];
	hedges    = new float*[width];
	for(int i = 0; i < width; i++){
		wedges[i] = new float[height];
		hedges[i] = new float[height];
		for(int j = 0; j < height; j++){
			wedges[i][j] = 0;//undef;
			hedges[i][j] = 0;//undef;
		}
	}

	float ** r = rgb[0];
	float ** g = rgb[1];
	float ** b = rgb[2];

	for(int i = 1; i < width-1; i++){
		for(int j = 1; j < height-1; j++){
			float rw = (r[i-1][j-1]+2*r[i-1][j]+r[i-1][j+1])-(r[i+1][j-1]+2*r[i+1][j]+r[i+1][j+1]);
			float gw = (g[i-1][j-1]+2*g[i-1][j]+g[i-1][j+1])-(g[i+1][j-1]+2*g[i+1][j]+g[i+1][j+1]);
			float bw = (b[i-1][j-1]+2*b[i-1][j]+b[i-1][j+1])-(b[i+1][j-1]+2*b[i+1][j]+b[i+1][j+1]);

			float rh = (r[i-1][j-1]+2*r[i][j-1]+r[i+1][j-1])-(r[i-1][j+1]+2*r[i][j+1]+r[i+1][j+1]);
			float gh = (g[i-1][j-1]+2*g[i][j-1]+g[i+1][j-1])-(g[i-1][j+1]+2*g[i][j+1]+g[i+1][j+1]);
			float bh = (b[i-1][j-1]+2*b[i][j-1]+b[i+1][j-1])-(b[i-1][j+1]+2*b[i][j+1]+b[i+1][j+1]);

			wedges[i][j] = sqrt(rw*rw+gw*gw+bw*bw);
			hedges[i][j] = sqrt(rh*rh+gh*gh+bh*bh);
		}
	}

/*
	float *** iic = new float**[3];
	iic[0] = getII(rgb[0],width,height);
	iic[1] = getII(rgb[1],width,height);
	iic[2] = getII(rgb[2],width,height);

	float ** cwedges;
	float ** chedges;

	getEdges(cwedges, chedges, iic, 3 , width, height,2,1);

	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			wedges[i][j] = cwedges[i][j];
			hedges[i][j] = chedges[i][j];
		}
	}
*/
/*

	float *** iip = new float**[3];
	iip[0] = getII(xyz[0],width,height);
	iip[1] = getII(xyz[1],width,height);
	iip[2] = getII(xyz[2],width,height);

	float *** iipValid = new float**[3];
	iipValid[0] = getIIValid(xyz[0],width,height);
	iipValid[1] = getIIValid(xyz[1],width,height);
	iipValid[2] = getIIValid(xyz[2],width,height);


	float *** iin = new float**[3];
	iin[0] = getII(nxnynz[0],width,height);
	iin[1] = getII(nxnynz[1],width,height);
	iin[2] = getII(nxnynz[2],width,height);

	float *** iinValid = new float**[3];
	iinValid[0] = getIIValid(nxnynz[0],width,height);
	iinValid[1] = getIIValid(nxnynz[1],width,height);
	iinValid[2] = getIIValid(nxnynz[2],width,height);

	float ** cwedges1;
	float ** chedges1;

	float ** cwedges2;
	float ** chedges2;

	float ** pwedges;
	float ** phedges;
	float ** nwedges;
	float ** nhedges;
	getEdges(cwedges1, chedges1, iic, 			3 , width, height,2,1);
	getEdges(cwedges2, chedges2, iic, 			3 , width, height,7,2);

	getEdges(pwedges, phedges, iip,	iipValid, 	3 , width, height,10,2);
	getEdges(nwedges, nhedges, iin,	iinValid, 	3 , width, height,25,3);

	float ** c1 = mergeEdges(cwedges1, chedges1, width, height);
	float ** c2 = mergeEdges(cwedges2, chedges2, width, height);
	float ** c = mul(c1,c2,width,height);//normalize(cwedges, chedges,

	normalize(pwedges, pwedges , width, height);
	float ** p = mergeEdges(pwedges, pwedges , width, height);
	trim(p, width, height);
	
	normalize(nwedges, nwedges , width, height);
	float ** n = mergeEdges(nwedges, nwedges , width, height);
	float ** pn = add(p,n,width,height);

	float ** cpn = mul(c,p,width,height);

	disp_xy_edge("color1"  , cwedges1,chedges1, width,height,false);
	disp_xy_edge("color2"  , cwedges2,chedges2, width,height,false);
	disp_xy_edge("true color"   , c,c, width,height,false);

	disp_xy_edge("p"   , p,p, width,height,false);
	disp_xy_edge("pn"   , pn,pn, width,height,false);

	disp_xy_edge("cpn"   , cpn,cpn, width,height,false);

	disp_xy_edge("pos"    , pwedges,phedges, width,height,false);
	disp_xy_edge("normal" , nwedges,nhedges, width,height,true);
*/
/*
    printf("getRGBEdges\n");
    float ** edges    = new float*[width];
    for(int i = 0; i < width; i++){
		edges[i] = new float[height];
        for(int j = 0; j < height; j++){
			edges[i][j] = undef;
        }
    }

	if(rgb != 0){
		float ** r = rgb[0];
		float ** g = rgb[1];
		float ** b = rgb[2];
		float div = 1/sqrt(255*255*3);
		for(int i = 1; i < width-1; i++){
			for(int j = 1; j < height-1; j++){
				int r_diff_w = r[i+1][j]-r[i-1][j];
				int r_diff_h = r[i][j+1]-r[i][j-1];
		        r_diff_w *= r_diff_w;
		        r_diff_h *= r_diff_h;

		        int g_diff_w = g[i+1][j]-g[i-1][j];
		        int g_diff_h = g[i][j+1]-g[i][j-1];
		        g_diff_w *= g_diff_w;
		        g_diff_h *= g_diff_h;

		        int b_diff_w = b[i+1][j]-b[i-1][j];
		        int b_diff_h = b[i][j+1]-b[i][j-1];
		        b_diff_w *= b_diff_w;
		        b_diff_h *= b_diff_h;
		        int w_sum = r_diff_w+g_diff_w+b_diff_w;
		        int h_sum = r_diff_h+g_diff_h+b_diff_h;
				rgb_edges[i][j] = sqrt(w_sum+h_sum);
		    }
		}
	}
*/
/*
	if(xyz != 0){
		printf("xyz\n");
		float w_edges[width][height];
		float h_edges[width][height];
		for(int i = 0; i < width; i++){
		    for(int j = 0; j < height; j++){
		        w_edges[i][j] = undef;
		        h_edges[i][j] = undef;
		    }
		}
		int maxunknown = 20;
		float ** z = xyz[2];
		for(int i = 1; i < width; i++){
			for(int j = 1; j < height; j++){
				if(w_edges[i][j]==undef){
					int nw = 1;
					int pw = 0;
					while(i-nw >= 0 && isnan(z[i-nw][j])){nw++;}
					while(i+pw < width && isnan(z[i+pw][j])){pw++;}

					if(i-nw >= 0 && i+pw < width && nw+pw < maxunknown){
						float e = z[i-nw][j]-z[i+pw][j];
						e*=e;
						for(int ii = i-nw; ii < i+pw; ii++){w_edges[ii][j]=e;}
					}
				}
				if(h_edges[i][j]==undef){
					int nh = 1;
					int ph = 0;
					while(j-nh >= 0 && isnan(z[i][j-nh])){nh++;}
					while(j+ph < height && isnan(z[i][j+ph])){ph++;}

					if(j-nh >= 0 && j+ph < height && nh+ph < maxunknown){
						float e = z[i][j-nh]-z[i][j+ph];
						e*=e;
						for(int jj = j-nh; jj < j+ph; jj++){h_edges[i][jj]=e;}
					}
				}
			}
		}

		float maxv = 0;
		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				if(w_edges[i][j]!=undef || h_edges[i][j]!=undef){
					xyz_edges[i][j] = 0;
					if(w_edges[i][j] != undef){		xyz_edges[i][j] += w_edges[i][j];}
					if(h_edges[i][j] != undef){		xyz_edges[i][j] += h_edges[i][j];}
					if(xyz_edges[i][j] > maxv){		maxv = xyz_edges[i][j];}
				}
			}
		}

		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				if(xyz_edges[i][j]!=undef){
					xyz_edges[i][j] /= maxv;
				}
			}
		}
	}

	if(nxnynz != 0){
		printf("nxnynz\n");
		float w_edges[width][height];
		float h_edges[width][height];
		for(int i = 0; i < width; i++){
		    for(int j = 0; j < height; j++){
		        w_edges[i][j] = undef;
		        h_edges[i][j] = undef;
		    }
		}

		int maxunknown = 20;
		float ** nx = nxnynz[0];
		float ** ny = nxnynz[1];
		float ** nz = nxnynz[2];

		for(int i = 1; i < width; i++){
			for(int j = 1; j < height; j++){
				if(w_edges[i][j]==undef){
					int nw = 1;
					int pw = 0;
					while(i-nw >= 0 && isnan(nz[i-nw][j])){nw++;}
					while(i+pw < width && isnan(nz[i+pw][j])){pw++;}

					if(i-nw >= 0 && i+pw < width && nw+pw < maxunknown){
						float ex = nx[i-nw][j]-nx[i+pw][j];
						float ey = ny[i-nw][j]-ny[i+pw][j];
						float ez = nz[i-nw][j]-nz[i+pw][j];
						float e = ex*ex+ey*ey+ez*ez;
						for(int ii = i-nw; ii < i+pw; ii++){w_edges[ii][j]=e;}
					}
				}
				if(h_edges[i][j]==undef){
					int nh = 1;
					int ph = 0;
					while(j-nh >= 0 && isnan(nz[i][j-nh])){nh++;}
					while(j+ph < height && isnan(nz[i][j+ph])){ph++;}

					if(j-nh >= 0 && j+ph < height && nh+ph < maxunknown){
						float ex = nx[i][j-nh]-nx[i][j+ph];
						float ey = ny[i][j-nh]-ny[i][j+ph];
						float ez = nz[i][j-nh]-nz[i][j+ph];
						float e = ex*ex+ey*ey+ez*ez;
						for(int jj = j-nh; jj < j+ph; jj++){h_edges[i][jj]=e;}
					}
				}
			}
		}

		float maxv = 0;
		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				if(w_edges[i][j]!=undef || h_edges[i][j]!=undef){
					nxnynz_edges[i][j] = 0;
					if(w_edges[i][j] != undef){		nxnynz_edges[i][j] += w_edges[i][j];}
					if(h_edges[i][j] != undef){		nxnynz_edges[i][j] += h_edges[i][j];}
					if(nxnynz_edges[i][j] > maxv){maxv = nxnynz_edges[i][j];}
				}
			}
		}

		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				if(nxnynz_edges[i][j]!=undef){
					nxnynz_edges[i][j] /= maxv;
				}
			}
		}

	}
*/
/*	
	for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
			if(rgb_edges[i][j] != undef || xyz_edges[i][j] != undef || nxnynz_edges[i][j] != undef){
				edges[i][j] = 0;            	
				if(rgb_edges[i][j] != undef){		edges[i][j] += rgb_edges[i][j];}
				if(xyz_edges[i][j] != undef){		edges[i][j] += xyz_edges[i][j];}
				if(nxnynz_edges[i][j] != undef){	edges[i][j] += nxnynz_edges[i][j];}
			}
        }
    }
*/
}
