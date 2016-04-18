#ifndef __SURFEL_TYPE__H
#define __SURFEL_TYPE__H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct SurfelType
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
  union
  {
    struct
    {
      uint32_t rgba;
      float confidence;
    };
    float data_c[4];

  };
  union
  {
    struct
    {
      float radius;
      float initTime;
      float timestamp;
      float unused;
    };
    float data_t[4];
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (SurfelType,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (uint32_t, rgba, rgba)
                                   (float, confidence, confidence)
                                   (float, radius, radius)
                                   (float, initTime, initTime)
                                   (float, timestamp, timestamp)
                                   (float, unused, unused)
)


#endif
