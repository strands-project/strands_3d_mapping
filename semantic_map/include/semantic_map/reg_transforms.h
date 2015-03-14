#ifndef __SEMANTIC_MAP_REGISTRATION_TRANSFORMS
#define __SEMANTIC_MAP_REGISTRATION_TRANSFORMS

#include <tf/tf.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace semantic_map_registration_transforms
{
    std::string saveRegistrationTransforms(std::vector<tf::StampedTransform> transforms, bool verbose=false, std::string file="registration_transforms.txt");
    std::vector<tf::StampedTransform> loadRegistrationTransforms(std::string file="default", bool verbose=false);

    std::string saveRegistrationTransforms(double*** poses, unsigned int x, unsigned int y, bool verbose=false, std::string filename="registration_transforms_raw.txt");
    double*** loadRegistrationTransforms(unsigned int& x, unsigned int& y, std::string file="default", bool verbose=false);
}


#endif // __SEMANTIC_MAP_REGISTRATION_TRANSFORMS
