Additional view registration
===========================

This package provides services for the registration of additional views of an object. The services are defined [here](../observation_registration_services). 

* ObjectAdditionalViewRegistrationService

```
string observation_xml # can be blank
string object_xml
---
geometry_msgs/Transform[] additional_view_transforms
int32[] additional_view_correspondences
geometry_msgs/Transform observation_transform
int32 observation_correspondences
```

* AdditionalViewRegistrationService

```
string observation_xml # can be blank
sensor_msgs/PointCloud2[] additional_views
geometry_msgs/Transform[] additional_views_odometry_transforms
---
geometry_msgs/Transform[] additional_view_transforms
int32[] additional_view_correspondences
geometry_msgs/Transform observation_transform
int32 observation_correspondences
```

Two interfaces are provides: the first takes as input an `object_xml` file which points to the additional views acquired by the robot (loaded with the [`metaroom_xml_parser`](https://github.com/strands-project/strands_3d_mapping/blob/hydro-devel/metaroom_xml_parser/include/metaroom_xml_parser/load_utilities.h#L99)), while the second one takes as input directly the additional views and the initial odometry poses. **Note** that the system will attempt registration even when the odometry transforms are not available, however in general the results are exepcted to be worse. Both interfaces take as input a second parameter `observation_xml` which points to an observation that the additional views should be registered to. 

The result is stored in `additional_view_transforms` - corresponding to the transforms which align the views with each other, along with `additional_view_correspondences` denoting the number of [SIFT](../siftgpu) correspondences used to compute the transforms. Similary, `observation_transform` stores the transform which aligns the additional views to the observation, and `observation_correspondences` denotes the number of correspondences used to compute it. 
