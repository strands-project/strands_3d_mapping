#include "../../include/mesher/Mesh.h"

namespace reglib
{

Vertex::Vertex(float x_, float y_, float z_){
    x = x_;
    y = y_;
    z = z_;
}
Vertex::~Vertex(){}

Triangle::Triangle(Vertex * a_, Vertex * b_, Vertex * c_){
    a = a_;
    b = b_;
    c = c_;
}

Triangle::~Triangle(){}

Mesh::Mesh(){}


Mesh::Mesh(Model * model){
    Vertex ** pixelvertexes = new Vertex*[60*480];
    for(unsigned int i = 0; i < 640*480; i++){pixelvertexes[i] = 0;}


    for(unsigned int f = 0; f < model->frames.size(); f++){
        RGBDFrame * frame = model->frames[f];
        //Datastuffs
        //TODO:: Project mesh to frame init only pixels that are not already part of a mesh
        //TODO:: DEAL with borders...
        bool * maskvec = model->modelmasks[f]->maskvec;
        Eigen::Matrix4d p = model->relativeposes[f];
        unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
        unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
        float		   * normalsdata	= (float			*)(frame->normals.data);

        unsigned int frameid = frame->id;

        float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
        float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
        float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

        Camera * camera				= frame->camera;
        const unsigned int width	= camera->width;
        const unsigned int height	= camera->height;
        const float idepth			= camera->idepth_scale;
        const float cx				= camera->cx;
        const float cy				= camera->cy;
        const float ifx				= 1.0/camera->fx;
        const float ify				= 1.0/camera->fy;
        //std::vector<superpoint> pointsToAdd;
        for(unsigned int w = 0; w < width; w++){
            for(unsigned int h = 0; h < height;h++){
//                int ind = h*width+w;
//                //if(maskdata[ind] == 255 && !reprojected[ind]){// && p.z > 0 && !isnan(p.normal_x)){
//                if(maskvec[ind] && !reprojected[ind]){// && p.z > 0 && !isnan(p.normal_x)){
//                    float z = idepth*float(depthdata[ind]);
//                    float nx = normalsdata[3*ind+0];

//                    if(z > 0 && nx != 2){
//                        float ny = normalsdata[3*ind+1];
//                        float nz = normalsdata[3*ind+2];

//                        float x = (w - cx) * z * ifx;
//                        float y = (h - cy) * z * ify;

//                        float px	= m00*x + m01*y + m02*z + m03;
//                        float py	= m10*x + m11*y + m12*z + m13;
//                        float pz	= m20*x + m21*y + m22*z + m23;
//                        float pnx	= m00*nx + m01*ny + m02*nz;
//                        float pny	= m10*nx + m11*ny + m12*nz;
//                        float pnz	= m20*nx + m21*ny + m22*nz;

//                        float pb = rgbdata[3*ind+0];
//                        float pg = rgbdata[3*ind+1];
//                        float pr = rgbdata[3*ind+2];

//                        Vector3f	pxyz	(px	,py	,pz );
//                        Vector3f	pnxyz	(pnx,pny,pnz);
//                        Vector3f	prgb	(pr	,pg	,pb );
//                        float		weight	= 1.0/(z*z);
//                        points.push_back(superpoint(pxyz,pnxyz,prgb, weight, weight, frameid));
//                    }
//                }
            }
        }
    }
}

Mesh::~Mesh(){}

//void Camera::save(std::string path){
//	printf("save camera to %s\n",path.c_str());
//	unsigned long buffersize = 7*sizeof(double);
//	char* buffer = new char[buffersize];
//	double * buffer_double = (double *)buffer;
//	unsigned long * buffer_long = (unsigned long *)buffer;

//	int counter = 0;
//	buffer_long[counter++] = id;
//	buffer_long[counter++] = width;
//	buffer_long[counter++] = height;
//	buffer_double[counter++] = fx;
//	buffer_double[counter++] = fy;
//	buffer_double[counter++] = cx;
//	buffer_double[counter++] = cy;

//	char buf [1024];
//	sprintf(buf,"%s_data.txt",path.c_str());

//	std::ofstream outfile (buf,std::ofstream::binary);
//	outfile.write (buffer,buffersize);
//	outfile.close();
//	delete[] buffer;
//}

//void Camera::print(){}

//Camera * Camera::load(std::string path){
//	Camera * cam = new Camera();

//	std::streampos size;
//	char * buffer;

//	std::ifstream file (path, std::ios::in | std::ios::binary | std::ios::ate);
//	if (file.is_open()){
//		size = file.tellg();
//		buffer = new char [size];
//		file.seekg (0, std::ios::beg);
//		file.read (buffer, size);
//		file.close();

//		double *		buffer_double	= (double *)buffer;
//		unsigned long * buffer_long		= (unsigned long *)buffer;

//		int counter = 0;
//		cam->id		= buffer_long[counter++];
//		cam->width	= buffer_long[counter++];
//		cam->height = buffer_long[counter++];
//		cam->fx		= buffer_double[counter++];
//		cam->fy		= buffer_double[counter++];
//		cam->cx		= buffer_double[counter++];
//		cam->cy		= buffer_double[counter++];

//		camera_id_count = std::max(int(cam->id+1),int(camera_id_count));

//		delete[] buffer;
//	}else{std::cout << "Unable to open file" << path << std::endl;}

//	return cam;
//}

}
