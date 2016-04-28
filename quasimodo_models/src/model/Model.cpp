#include "model/Model.h"
#include <map>

namespace reglib
{
using namespace Eigen;

unsigned int model_id_counter = 0;

Model::Model(){
    total_scores = 0;
    score = 0;
    id = model_id_counter++;
    last_changed = -1;
}

Model::Model(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d pose){
    total_scores = 0;
    scores.resize(1);
    scores.back().resize(1);
    scores[0][0] = 0;

    score = 0;
    id = model_id_counter++;

    last_changed = -1;

    relativeposes.push_back(pose);
    frames.push_back(frame);
    modelmasks.push_back(new ModelMask(mask));
    recomputeModelPoints();
}

void Model::recomputeModelPoints(){
//	for(unsigned int i = 0; i < frames.size(); i++){
//		bool res = testFrame(i);
//	}

    points.clear();
    for(unsigned int i = 0; i < frames.size(); i++){
        addPointsToModel(frames[i],modelmasks[i],relativeposes[i]);
    }
}

void Model::addPointsToModel(RGBDFrame * frame, ModelMask * modelmask, Eigen::Matrix4d p){
    bool * maskvec = modelmask->maskvec;
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

    for(unsigned int w = 0; w < width; w++){
        for(unsigned int h = 0; h < height;h++){
            int ind = h*width+w;
            if(maskvec[ind]){
                float z = idepth*float(depthdata[ind]);
                float nx = normalsdata[3*ind+0];

                if(z > 0 && nx != 2){
                    float ny = normalsdata[3*ind+1];
                    float nz = normalsdata[3*ind+2];

                    float x = (w - cx) * z * ifx;
                    float y = (h - cy) * z * ify;

                    float px	= m00*x + m01*y + m02*z + m03;
                    float py	= m10*x + m11*y + m12*z + m13;
                    float pz	= m20*x + m21*y + m22*z + m23;
                    float pnx	= m00*nx + m01*ny + m02*nz;
                    float pny	= m10*nx + m11*ny + m12*nz;
                    float pnz	= m20*nx + m21*ny + m22*nz;

                    float pb = rgbdata[3*ind+0];
                    float pg = rgbdata[3*ind+1];
                    float pr = rgbdata[3*ind+2];

                    Vector3f	pxyz	(px	,py	,pz );
                    Vector3f	pnxyz	(pnx,pny,pnz);
                    Vector3f	prgb	(pr	,pg	,pb );
                    float		weight	= 1.0/(z*z);
                    points.push_back(superpoint(pxyz,pnxyz,prgb, weight, weight, frameid));
                }
            }
        }
    }
}

bool Model::testFrame(int ind){
	printf("testing frame %i\n",ind);
	Eigen::Matrix3f covMat = Eigen::Matrix3f::Identity();

	ModelMask * modelmask = modelmasks[ind];
	RGBDFrame * frame = frames[ind];

	bool * maskvec = modelmask->maskvec;
	unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
	unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
	float		   * normalsdata	= (float			*)(frame->normals.data);

	Camera * camera				= frame->camera;
	const unsigned int width	= camera->width;
	const unsigned int height	= camera->height;

	double tot_w = 0;
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height;h++){
			int ind = h*width+w;
			if(maskvec[ind]){
				float nx = normalsdata[3*ind+0];
				if(nx != 2){
					float ny = normalsdata[3*ind+1];
					float nz = normalsdata[3*ind+2];
					covMat(0,0) += nx*nx;
					covMat(0,1) += nx*ny;
					covMat(0,2) += nx*nz;

					covMat(1,0) += ny*nx;
					covMat(1,1) += ny*ny;
					covMat(1,2) += ny*nz;


					covMat(2,0) += nz*nx;
					covMat(2,1) += nz*ny;
					covMat(2,2) += nz*nz;

					tot_w++;
				}
			}
		}
	}




	printf("totw: %f\n",tot_w);
	double threshold = 500;
	if(tot_w < threshold){return false;}

//	for(int i = 0; i < 3; i++){
//		for(int j = 0; j < 3; j++){
//			covMat(i,j) /= tot_w;
//		}
//	}

	Eigen::EigenSolver<Eigen::Matrix3f> es(covMat, false);
	auto e = es.eigenvalues();

	double e1 = e(0).real();
	double e2 = e(1).real();
	double e3 = e(2).real();

	printf("%f %f %f\n",e1,e2,e3);

	if(e1 > threshold || e2 > threshold || e3 > threshold){return false;}
	return true;
}

void Model::print(){
    printf("id: %i ",int(id));
    printf("last_changed: %i ",int(last_changed));
    printf("score: %f ",score);
    printf("total_scores: %f ",total_scores);
    printf("frames: %i ",int(frames.size()));
    printf("modelmasks: %i ",int(modelmasks.size()));
    printf("relativeposes: %i\n",int(relativeposes.size()));
}

void Model::addFrameToModel(RGBDFrame * frame,  ModelMask * modelmask, Eigen::Matrix4d p){
    addPointsToModel(frame, modelmask, p);

    relativeposes.push_back(p);
    frames.push_back(frame);
    modelmasks.push_back(modelmask);
}

void Model::merge(Model * model, Eigen::Matrix4d p){
    for(unsigned int i = 0; i < model->frames.size(); i++){
        relativeposes.push_back(p * model->relativeposes[i]);
        frames.push_back(model->frames[i]);
        modelmasks.push_back(model->modelmasks[i]);
    }
    recomputeModelPoints();
}

CloudData * Model::getCD(unsigned int target_points){
    std::vector<unsigned int> ro;
    unsigned int nc = points.size();
    ro.resize(nc);
    for(unsigned int i = 0; i < nc; i++){ro[i] = i;}
    for(unsigned int i = 0; i < nc; i++){
        unsigned int randval = rand();
        unsigned int rind = randval%nc;
        int tmp = ro[i];
        ro[i] = ro[rind];
        ro[rind] = tmp;
    }
    //Build registration input
    unsigned int nr_points = std::min(unsigned(points.size()),target_points);
    MatrixXd data			(6,nr_points);
    MatrixXd data_normals	(3,nr_points);
    MatrixXd information	(6,nr_points);

    for(unsigned int k = 0; k < nr_points; k++){
        superpoint & p		= points[ro[k]];
        data(0,k)			= p.point(0);
        data(1,k)			= p.point(1);
        data(2,k)			= p.point(2);
        data(3,k)			= p.feature(0);
        data(4,k)			= p.feature(1);
        data(5,k)			= p.feature(2);
        data_normals(0,k)	= p.normal(0);
        data_normals(1,k)	= p.normal(1);
        data_normals(2,k)	= p.normal(2);
        information(0,k)	= p.point_information;
        information(1,k)	= p.point_information;
        information(2,k)	= p.point_information;
        information(3,k)	= p.feature_information;
        information(4,k)	= p.feature_information;
        information(5,k)	= p.feature_information;
    }

    CloudData * cd			= new CloudData();
    cd->data				= data;
    cd->information			= information;
    cd->normals				= data_normals;
    return cd;
}

Model::~Model(){}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Model::getPCLnormalcloud(int step, bool color){
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for(unsigned int i = 0; i < points.size(); i+=step){
        superpoint & sp = points[i];
        pcl::PointXYZRGBNormal p;
        p.x = sp.point(0);
        p.y = sp.point(1);
        p.z = sp.point(2);

        p.normal_x = sp.normal(0);
        p.normal_y = sp.normal(1);
        p.normal_z = sp.normal(2);
        if(color){
            p.b =   0;
            p.g = 255;
            p.r =   0;
        }else{
            p.r = sp.feature(0);
            p.g = sp.feature(1);
            p.b = sp.feature(2);
        }
        cloud_ptr->points.push_back(p);
    }
    return cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model::getPCLcloud(int step, bool color){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    //	if(color){
    //		for(unsigned int i = 0; i < points.size(); i+=step){
    //			superpoint & sp = points[i];
    //			pcl::PointXYZRGB p;
    //			p.x = sp.point(0);
    //			p.y = sp.point(1);
    //			p.z = sp.point(2);
    //			if(color){
    //				p.b =   0;
    //				p.g = 255;
    //				p.r =   0;
    //			}else{
    //				p.r = sp.feature(0);
    //				p.g = sp.feature(1);
    //				p.b = sp.feature(2);
    //			}
    //			cloud_ptr->points.push_back(p);
    //		}
    //	}else{
    std::map<int,int> mymapR;
    std::map<int,int> mymapG;
    std::map<int,int> mymapB;
    for(unsigned int f = 0; f < frames.size(); f++){
        //unsigned char  * maskdata		= (unsigned char	*)(masks[f].data);
        bool * maskvec = modelmasks[f]->maskvec;
        unsigned char  * rgbdata		= (unsigned char	*)(frames[f]->rgb.data);
        unsigned short * depthdata		= (unsigned short	*)(frames[f]->depth.data);
        float		   * normalsdata	= (float			*)(frames[f]->normals.data);

        Eigen::Matrix4d p = relativeposes[f];

        int sweepid = modelmasks[f]->sweepid;

        int pr,pg,pb;
        if(sweepid == -1){
            pr = rand()%256;
            pg = rand()%256;
            pb = rand()%256;
        }else{
            if (mymapR.count(sweepid)==0){
                mymapR[sweepid] = rand()%256;
                mymapG[sweepid] = rand()%256;
                mymapB[sweepid] = rand()%256;
            }
            pr = mymapR[sweepid];
            pg = mymapG[sweepid];
            pb = mymapB[sweepid];
        }

        float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
        float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
        float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

        Camera * camera				= frames[f]->camera;
        const unsigned int width	= camera->width;
        const unsigned int height	= camera->height;
        const float idepth			= camera->idepth_scale;
        const float cx				= camera->cx;
        const float cy				= camera->cy;
        const float ifx				= 1.0/camera->fx;
        const float ify				= 1.0/camera->fy;


        for(unsigned int w = 0; w < width; w++){
            for(unsigned int h = 0; h < height;h++){
                int ind = h*width+w;
                //if(maskdata[ind] == 255){
                if(maskvec[ind]){
                    float z = idepth*float(depthdata[ind]);
                    if(z > 0){
                        float x = (w - cx) * z * ifx;
                        float y = (h - cy) * z * ify;

                        float px	= m00*x + m01*y + m02*z + m03;
                        float py	= m10*x + m11*y + m12*z + m13;
                        float pz	= m20*x + m21*y + m22*z + m23;

                        pcl::PointXYZRGB p;
                        p.x = px;
                        p.y = py;
                        p.z = pz;
                        if(color){
                            p.b = rgbdata[3*ind+0];
                            p.g = rgbdata[3*ind+1];
                            p.r = rgbdata[3*ind+2];
                        }else{
                            p.b = pb;
                            p.g = pg;
                            p.r = pr;
                        }
                        cloud_ptr->points.push_back(p);
                    }
                }
            }
        }
    }
    //}
    return cloud_ptr;
}

void Model::save(std::string path){
    unsigned long buffersize = (1+1+16*frames.size()+1+frames.size()*frames.size()+1*frames.size())*sizeof(double);
    char* buffer = new char[buffersize];
    double * buffer_double = (double *)buffer;
    unsigned long * buffer_long = (unsigned long *)buffer;

    int counter = 0;
    buffer_long[counter++] = frames.size();
    buffer_double[counter++] = score;
    for(unsigned int f = 0; f < frames.size(); f++){
        Eigen::Matrix4d pose = relativeposes[f];
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                buffer_double[counter++] = pose(i,j);
            }
        }
    }

    buffer_double[counter++] = total_scores;

    for(unsigned int f1 = 0; f1 < frames.size(); f1++){
        for(unsigned int f2 = 0; f2 < frames.size(); f2++){
            buffer_double[counter++] = scores[f1][f2];
        }
    }


    for(unsigned int f1 = 0; f1 < frames.size(); f1++){
        buffer_long[counter++] = modelmasks[f1]->sweepid;
    }

    std::ofstream outfile (path+"/data.txt",std::ofstream::binary);
    outfile.write (buffer,buffersize);
    outfile.close();
    delete[] buffer;


    ofstream posesfile;
    posesfile.open (path+"/poses.txt");

    ofstream raresfile;
    raresfile.open (path+"/raresposes.txt");
    Eigen::Matrix4f eigen_tr(Eigen::Matrix4f::Identity() );
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
    raresfile << eigen_tr.format(CommaInitFmt)<<endl;


    //printf("saving model %i to %s\n",id,path.c_str());
    for(unsigned int f = 0; f < frames.size(); f++){
        char buf [1024];

        sprintf(buf,"%s/frame_%i",path.c_str(),f);
        frames[f]->save(std::string(buf));

        sprintf(buf,"%s/modelmask_%i.png",path.c_str(),f);
        cv::imwrite( buf, modelmasks[f]->getMask() );

        Eigen::Matrix4f eigen_tr(relativeposes[f].cast<float>());
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
        raresfile << eigen_tr.format(CommaInitFmt)<<endl;
    }

    raresfile.close();


    for(unsigned int f = 0; f < frames.size(); f++){
        char buf [1024];

        sprintf(buf,"%s/views/cloud_%08i.pcd",path.c_str(),f);
        frames[f]->savePCD(std::string(buf));

        sprintf(buf,"%s/views/pose_%08i.txt",path.c_str(),f);
        ofstream posefile;
        posefile.open (buf);

        Eigen::Matrix4f eigen_tr(relativeposes[f].cast<float>());
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
        posefile << eigen_tr.format(CommaInitFmt)<<endl;
        posefile.close();

        bool * maskvec = modelmasks[f]->maskvec;
        sprintf(buf,"%s/views/object_indices_%08i.txt",path.c_str(),f);
        ofstream indfile;
        indfile.open (buf);
        for(int i = 0; i < 640*480; i++){
            if(maskvec[i]){indfile << i << std::endl;}
        }
        indfile.close();
        //object_indices_00000030.txt

    }
}

Model * Model::load(Camera * cam, std::string path){
    printf("Model * Model::load(Camera * cam, std::string path)\n");
    std::streampos size;
    char * buffer;
    char buf [1024];
    std::string datapath = path+"/data.txt";
    std::ifstream file (datapath, std::ios::in | std::ios::binary | std::ios::ate);
    if (file.is_open()){
        size = file.tellg();
        buffer = new char [size];
        file.seekg (0, std::ios::beg);
        file.read (buffer, size);
        file.close();

        Model * mod = new Model();
        double *		buffer_double	= (double *)buffer;
        unsigned long * buffer_long		= (unsigned long *)buffer;
        //		std::vector<ModelMask*> modelmasks;

        int counter = 0;
        unsigned int nr_frames = buffer_long[counter++];
        mod->score = buffer_double[counter++];
        for(unsigned int f = 0; f < nr_frames; f++){
            Eigen::Matrix4d pose;
            for(int i = 0; i < 4; i++){
                for(int j = 0; j < 4; j++){
                    pose(i,j) = buffer_double[counter++];
                }
            }

            sprintf(buf,"%s/frame_%i",path.c_str(),int(f));
            RGBDFrame * frame = RGBDFrame::load(cam, std::string(buf));

            sprintf(buf,"%s/modelmask_%i.png",path.c_str(),int(f));
            cv::Mat mask = cv::imread(buf, -1);   // Read the file

            mod->relativeposes.push_back(pose);
            mod->frames.push_back(frame);
            mod->modelmasks.push_back(new ModelMask(mask));
        }

        mod->total_scores = buffer_double[counter++];
        mod->scores.resize(nr_frames);
        for(unsigned int f1 = 0; f1 < nr_frames; f1++){
            mod->scores[f1].resize(nr_frames);
            for(unsigned int f2 = 0; f2 < nr_frames; f2++){
                mod->scores[f1][f2] = buffer_double[counter++];
            }
        }

        for(unsigned int f = 0; f < nr_frames; f++){
            mod->modelmasks[f]->sweepid = buffer_long[counter++];
            printf("modelmask sweepid: %i\n",int(mod->modelmasks[f]->sweepid));
        }

        mod->recomputeModelPoints();
        delete[] buffer;
        return mod;
    }else{std::cout << "Unable to open model file " << datapath << std::endl; exit(0);}
    return 0;
}

}

