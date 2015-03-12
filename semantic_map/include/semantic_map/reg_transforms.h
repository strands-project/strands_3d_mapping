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
    std::string saveRegistrationTransforms(std::vector<tf::StampedTransform> transforms, bool verbose=false);
    std::vector<tf::StampedTransform> loadRegistrationTransforms(std::string file="default", bool verbose=false);
}


#endif // __SEMANTIC_MAP_REGISTRATION_TRANSFORMS
