Observation registration services
=================================


This package defines a number of service messages, which should be included by the packages using observation/additional view registration (thus avoiding a dependency on the registration packages, which in turn depend on CUDA). 

#### ObservationRegistrationService

```
string source_observation_xml
string target_observation_xml
---
geometry_msgs/Transform transform
int32 total_correspondences
```

This service returns the 3D transformation aligning the source observation to the target observation (i.e. `source * transform = target`). The number of correspondences in `total_correspondences` gives an indication of the quality of the registration (specifically, if `total_correspondences = 0` the registration has failed). 

#### ObjectAdditionalViewRegistrationService

```
string observation_xml # can be blank
string object_xml
---
geometry_msgs/Transform[] additional_view_transforms
int32[] additional_view_correspondences
geometry_msgs/Transform observation_transform
int32 observation_correspondences
```

This service registers the additional views acquired for a specific object a) with respect to each other and b) with an observation. The inputs to this service are the `object_xml` file, pointing to the object containing the additional views to be registered. The object is loaded from the disk using the [metaroom_xml_parser](https://github.com/strands-project/strands_3d_mapping/blob/hydro-devel/metaroom_xml_parser/include/metaroom_xml_parser/load_utilities.h#L99). The second parameter is the `observation_xml` previously acquired using the [strands_3d_mapping](https://github.com/strands-project/strands_3d_mapping) pipeline. If this parameters is not blank, the additional views of the object will also be registered to the observation intermediate clouds. The underlying registration is done using [siftgpu](../siftgpu) and the CERES optimization engine. 

The registered poses of the object additional views are recorded in the vector `additional_view_transforms`, while the number of correspondences used per view are stored in `additional_view_correspondences`. Similarly, the transform that aligns the views to the observation is stored in `observation_transform` and the number of correspondences used to computed it in `observation_correspondences`.

####  	AdditionalViewRegistrationService

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

This service can be used for the same purpose as the previous one. The difference is that, instead of passing an `object_xml` file, one can pass a number of `additional_view` (in the form of point clouds), and, optionally (though strongly advised), odometry poses for the additional views, which the optimizer will use as initial guesses for the additional view poses. 
