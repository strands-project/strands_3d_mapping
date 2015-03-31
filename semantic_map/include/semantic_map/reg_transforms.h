#ifndef __SEMANTIC_MAP_REGISTRATION_TRANSFORMS
#define __SEMANTIC_MAP_REGISTRATION_TRANSFORMS

#include <tf/tf.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <image_geometry/pinhole_camera_model.h>

namespace semantic_map_registration_transforms
{
    std::string saveRegistrationTransforms(std::vector<tf::StampedTransform> transforms, bool verbose=false, std::string file="registration_transforms.txt");
    std::vector<tf::StampedTransform> loadRegistrationTransforms(std::string file="default", bool verbose=false);

    std::string saveRegistrationTransforms(double*** poses, unsigned int x, unsigned int y, bool verbose=false, std::string filename="registration_transforms_raw.txt");
    double*** loadRegistrationTransforms(unsigned int& x, unsigned int& y, std::string file="default", bool verbose=false);

    std::string saveCameraParameters(image_geometry::PinholeCameraModel camParams, bool verbose=false, std::string file="camera_params.txt");
    image_geometry::PinholeCameraModel loadCameraParameters(std::string file="default", bool verbose=false);

    void getPtuAnglesForIntPosition(int pan_start, int pan_step, int pan_end, int tilt_start, int tilt_step, int tilt_end,
                                    int int_position, int& pan_angle, int& tilt_angle, bool verbose = false);
}


#endif // __SEMANTIC_MAP_REGISTRATION_TRANSFORMS
