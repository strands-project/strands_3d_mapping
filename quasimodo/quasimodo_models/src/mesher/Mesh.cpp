#include "../../include/mesher/Mesh.h"

#include <pcl/PolygonMesh.h>

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
Mesh::~Mesh(){}

void Mesh::build(Model * model, int type){
    if(type == 0){
        Vertex ** pixelvertexes = new Vertex*[640*480];
        for(unsigned int f = 0; f < model->frames.size(); f++){
            for(unsigned int i = 0; i < 640*480; i++){pixelvertexes[i] = 0;}
            RGBDFrame * frame = model->frames[f];

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
                    int ind = h*width+w;
                    if(maskvec[ind]){
                        float z = idepth*float(depthdata[ind]);
                        if(z > 0){
                            float x = (w - cx) * z * ifx;
                            float y = (h - cy) * z * ify;

                            float px	= m00*x + m01*y + m02*z + m03;
                            float py	= m10*x + m11*y + m12*z + m13;
                            float pz	= m20*x + m21*y + m22*z + m23;

                            pixelvertexes[ind] = new Vertex(px,py,pz);

                            //                            float pb = rgbdata[3*ind+0];
                            //                            float pg = rgbdata[3*ind+1];
                            //                            float pr = rgbdata[3*ind+2];
                        }
                    }
                }
            }

            int step = 10;

            for(unsigned int w = step; w < width; w+=step){
                for(unsigned int h = step; h < height;h+=step){
                    Vertex * a = pixelvertexes[h*width+w];
                    Vertex * b = pixelvertexes[h*width+w-step];
                    Vertex * c = pixelvertexes[(h-step)*width+w];
                    if(a && b && c){
                        Triangle * t = new Triangle(a,b,c);
                        a->triangles.push_back(t);
                        b->triangles.push_back(t);
                        c->triangles.push_back(t);
                        triangles.push_back(t);
                    }
                }
            }

            for(unsigned int w = 0; w < width-step; w+=step){
                for(unsigned int h = 0; h < height-step;h+=step){
                    Vertex * a = pixelvertexes[h*width+w];
                    Vertex * b = pixelvertexes[h*width+w+step];
                    Vertex * c = pixelvertexes[(h+step)*width+w];
                    if(a && b && c){
                        Triangle * t = new Triangle(a,b,c);
                        a->triangles.push_back(t);
                        b->triangles.push_back(t);
                        c->triangles.push_back(t);
                        triangles.push_back(t);
                    }
                }
            }

            for(unsigned int i = 0; i < 640*480; i++){
                Vertex * v = pixelvertexes[i];
                if(v){
                    if(v->triangles.size() > 0){
                        vertexes.push_back(v);
                    }else{
                        delete v;
                    }
                }
            }
        }

        delete[] pixelvertexes;
    }

}
void Mesh::show(pcl::visualization::PCLVisualizer * viewer){
    pcl::PolygonMesh mesh;
    //mesh.cloud.resize(3*triangles.size());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.resize(3*triangles.size());
    mesh.polygons.resize(triangles.size());
    pcl::PointCloud<pcl::PointXYZRGB> tmpcloud;
    tmpcloud.points.resize(3);
    viewer->removeAllShapes();
    for(unsigned int i = 0; i < triangles.size(); i++){
        Triangle * t = triangles[i];
        //        tmpcloud.points[0].x = t->a->x;
        //        tmpcloud.points[0].y = t->a->y;
        //        tmpcloud.points[0].z = t->a->z;

        //        tmpcloud.points[1].x = t->b->x;
        //        tmpcloud.points[1].y = t->b->y;
        //        tmpcloud.points[1].z = t->b->z;

        //        tmpcloud.points[2].x = t->c->x;
        //        tmpcloud.points[2].y = t->c->y;
        //        tmpcloud.points[2].z = t->c->z;

        //        char buf [1024];
        //        sprintf(buf,"%i",i);
        //        viewer->addPolygon<pcl::PointXYZRGB>(tmpcloud,std::string(buf));

        cloud->points.at(3*i).x = t->a->x;
        cloud->points.at(3*i).y = t->a->y;
        cloud->points.at(3*i).z = t->a->z;

        cloud->points.at(3*i+1).x = t->b->x;
        cloud->points.at(3*i+1).y = t->b->y;
        cloud->points.at(3*i+1).z = t->b->z;

        cloud->points.at(3*i+2).x = t->c->x;
        cloud->points.at(3*i+2).y = t->c->y;
        cloud->points.at(3*i+2).z = t->c->z;

        mesh.polygons[i].vertices.push_back(3*i);
        mesh.polygons[i].vertices.push_back(3*i+1);
        mesh.polygons[i].vertices.push_back(3*i+2);
    }
    pcl::toPCLPointCloud2(*cloud,mesh.cloud);
    viewer->addPolygonMesh(mesh, "polygon");
    //viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"polygon");
    viewer->spin();
}

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
