# object_manager

This package allows interaction with the dynamic clusters segmented by the `semantic_map` package. 

## Parameters

* `log_objects_to_db` - whether the clusters segmented should be logged to mongodb. The default value is `True`
*  `object_folder` - the folder where to look for dynamic clusters. The default path is `~/.semanticMap/`
*  `min_object_size` - clusters with fewer points than this threshold are discarded. The default value is `500`.
*  `additional_views_topic` - the topic on which the additional views are published. The default is `/object_learning/object_view` 
*  `additional_views_status_topic` - the topic on which status messages when collecting additional views are published. The default is `/object_learning/status`. The topic messages supported are `start_viewing` (only accepted if a cluster has been previously selected) and `stop_viewing`

## DynamicObjectsService

Message tpe:
```
string waypoint_id
---
string[] object_id
sensor_msgs/PointCloud2[] objects
geometry_msgs/Point[] centroids
```

Given a waypoint id, this service returns all the dynamic clusters segmented at that waypoint, with their ids, point clouds and centroid. 

Service topic: `ObjectManager/DynamicObjectsService`

The point cloud corresponding to all the dynamic clusters is also published on the topic `"/object_manager/objects` 
##  GetDynamicObjectService

Message type:
```
string waypoint_id
string object_id
---
sensor_msgs/PointCloud2 object_cloud
int32[] object_mask
geometry_msgs/Transform transform_to_map
int32 pan_angle
int32 tilt_angle
```

Given a waypoint id and a cluster id (should correspond to the ids received after calling the `DynamicObjectsService`), this service returns the point cloud corresponding to that dynamic cluster in the camera frame of reference and a transform to get the point cloud in the map frame of refence. In addition, a set of angles (`pan_angle`, and `tilt_angle`) to which to turn the PTU, and a set of indices representing image pixels corresponding to the dynamic cluster in the image obtained after turning the PTU to the specified angles. 
After calling this service, the requested dynamic cluster is "selected", and after receiving the `start_viewing` mesasge on the `object_learning/status` topic, additional views received on the `/object_learning/object_view` topic will be added and logged together with this cluster.

Service topic: `ObjectManager/GetDynamicObjectService`

The point cloud corresponding to the requested dynamic cluster is also published on the topic `/object_manager/requested_object`.

The cluster mask is also published as an image on the topic: `/object_manager/requested_object_mask`

Note that the clusters are logged to the database when calling the `DynamicObjectsService` or  the `GetDynamicObjectService` (if the `log_to_db` argument is set to `True`). Calling these services multiple times does not affect (negatively) the logging. 

## Export logged dynamic clusters from mongodb

```rosrun object_manager load_objects_from_mongo /path/where/to/export/data/```

The data exported is saved according to the sweeps where the clusters were extracted (i.e. `YYYYMMDD/patrol_run_#/room_#/...`)
