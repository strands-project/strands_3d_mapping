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
