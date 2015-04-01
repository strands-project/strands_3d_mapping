#ifndef __DYNAMIC_OBJECT_UTILITIES__
#define __DYNAMIC_OBJECT_UTILITIES__

#include <tf/tf.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <dynamic_object.h>
#include <dynamic_object_xml_parser.h>

namespace dynamic_object_utilities
{
    std::vector<DynamicObject::Ptr> loadDynamicObjects(std::string folder, bool verbose = false);
}


#endif // __DYNAMIC_OBJECT_UTILITIES__
