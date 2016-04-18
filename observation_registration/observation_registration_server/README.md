Observation registration server
================================

This node provides a service which registers two observations. [The service used](../observation_registration_services/srv/ObservationRegistrationService.srv) is defined [here](../observation_registration_services):

```

string source_observation_xml
string target_observation_xml
---
geometry_msgs/Transform transform
int32 total_correspondences
```

The service returns the transform which aligns the source observation to the target observation. The underlying registration computes correspondences between pairs of images from the source and target observations from which the registration transformation is computed. 
