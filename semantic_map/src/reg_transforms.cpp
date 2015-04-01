#include "semantic_map/reg_transforms.h"
#include <boost/filesystem.hpp>
#include <fstream>

using namespace std;

std::string semantic_map_registration_transforms::saveRegistrationTransforms(std::vector<tf::StampedTransform> transforms, bool verbose, std::string fn)
{
    // get home folder
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    path+="/.ros/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    path+="semanticMap/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    string fileName = path+fn;
    if (verbose)
    {
        cout<<"Saving registration transforms at  "<<fileName<<endl;
    }

    ofstream out;
    out.open(fileName);

    // save transforms
    for (tf::StampedTransform transform : transforms)
    {
        out<<transform.frame_id_<<" "<<transform.child_frame_id_<<" ";
        out<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<" ";
        out<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w();
        out<<endl;
    }

    out.close();
    return fileName;
}

std::vector<tf::StampedTransform> semantic_map_registration_transforms::loadRegistrationTransforms(std::string file, bool verbose)
{
    std::vector<tf::StampedTransform> toRet;
    if (file == "default") // load data from the default path
    {
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.ros/semanticMap/registration_transforms.txt";
        file = path;
    }

    if (verbose)
    {
        cout<<"Loading transforms from "<<file<<endl;
    }

    ifstream fin;
    fin.open(file);

    while(fin.good())
    {
        string frame, child_frame;
        double x,y,z,w;
        tf::StampedTransform transform;
        fin>>frame>>child_frame;
        transform.frame_id_ = frame;
        transform.child_frame_id_ = child_frame;
        fin>>x>>y>>z;
        transform.setOrigin(tf::Vector3(x,y,z));
        fin>>x>>y>>z>>w;
        transform.setRotation(tf::Quaternion(x,y,z,w));
        if (!fin.good()) break;
        toRet.push_back(transform);

    }

    if (verbose)
    {
        cout<<"Loaded "<<toRet.size()<<" transforms."<<endl;
    }
    return toRet;
}

std::string semantic_map_registration_transforms::saveRegistrationTransforms(double*** poses, unsigned int x, unsigned int y, bool verbose, std::string fn)
{

    // get home folder
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    path+="/.ros/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    path+="semanticMap/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    string fileName = path+fn;
    if (verbose)
    {
        cout<<"Saving raw registration data at  "<<fn<<endl;
    }

    ofstream out;
    out.open(fileName);

    out<<x<<" "<<y<<endl;

    for (size_t i=0; i<x; i++){
        for (size_t j=0; j<y;j++){
            for (size_t k=0; k<6;k++)
            {
                out<<poses[i][j][k]<<" ";
            }
        }
    }

    out.close();
    return fileName;

}

double*** semantic_map_registration_transforms::loadRegistrationTransforms(unsigned int& x, unsigned int& y, std::string file, bool verbose)
{
    double*** toRet;

    if (file == "default") // load data from the default path
    {
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.ros/semanticMap/registration_transforms_raw.txt";
        file = path;
    }

    if (verbose)
    {
        cout<<"Loading raw registration data from "<<file<<endl;
    }

    ifstream fin;
    fin.open(file);

    if (!fin.good())
    {
        return toRet;
    }

    fin>>x>>y;

    toRet = new double**[x];

    for(unsigned int i = 0; i < x; i++){
        toRet[i] = new double*[y];
        for(unsigned int j = 0; j < y; j++){
            toRet[i][j] = new double[6];
            for(unsigned int k = 0; k < 6; k++){toRet[i][j][k] = 0;}
        }
    }

    for (size_t i=0; i<x; i++){
        for (size_t j=0; j<y;j++){
            for (size_t k=0; k<6;k++)
            {
                fin>>toRet[i][j][k];
            }
        }
    }

    return toRet;
}

std::string semantic_map_registration_transforms::saveCameraParameters(image_geometry::PinholeCameraModel camParams, bool verbose, std::string file)
{
    // get home folder
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    path+="/.ros/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    path+="semanticMap/";
    if ( ! boost::filesystem::exists( path ) )
    {
        if (!boost::filesystem::create_directory(path))
        {
            cerr<<"Cannot create folder "<<path<<endl;
            return "";
        }
    }

    string fileName = path+file;
    if (verbose)
    {
        cout<<"Saving camera parameters at  "<<fileName<<endl;
    }

    ofstream out;
    out.open(fileName);

    out<<camParams.fx()<<" "<<camParams.fy()<<" "<<camParams.cx()<<" "<<camParams.cy();
    out.close();

    return fileName;
}

image_geometry::PinholeCameraModel semantic_map_registration_transforms::loadCameraParameters(std::string file, bool verbose)
{


    sensor_msgs::CameraInfo camInfo;
    camInfo.P = {-1, 0.0, -1, 0.0, 0.0, -1, -1, 0.0,0.0, 0.0, 1.0,0.0};
    camInfo.D = {0,0,0,0,0};
    image_geometry::PinholeCameraModel toRet;
    toRet.fromCameraInfo(camInfo);

    if (file == "default") // load data from the default path
    {
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.ros/semanticMap/camera_params.txt";
        file = path;
    }

    if (verbose)
    {
        cout<<"Loading camera parameters from "<<file<<endl;
    }

    ifstream fin;
    fin.open(file);

    if (!fin.good())
    {
        return toRet;
    }

    double fx, fy, cx, cy;
    fin>>fx>>fy>>cx>>cy;
    camInfo.P = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0,0.0, 0.0, 1.0,0.0};
    toRet.fromCameraInfo(camInfo);

    return toRet;
}

void semantic_map_registration_transforms::getPtuAnglesForIntPosition(int pan_start, int pan_step, int pan_end, int tilt_start, int tilt_step, int tilt_end,
                                int int_position, int& pan_angle, int& tilt_angle, bool verbose)
{
    int pan_steps = ((abs(pan_start)+abs(pan_end))/pan_step) + 1;
    int tilt_steps = ((abs(tilt_start)+abs(tilt_end))/tilt_step) + 1;

    tilt_angle = (int_position/pan_steps) * tilt_step + tilt_start;
    pan_angle = (int_position%pan_steps) * pan_step + pan_start;

    if (verbose)
    {
        cout<<"For intermediate position "<<int_position<<" the PTU angles are pan: "<<pan_angle<<"  tilt: "<<tilt_angle<<endl;
    }
}
