# semantic_map_publisher

This package provides an interface to observation data previously recorded and stored on the disk. The data can be queried using the services described below.


## WaypointInfoService

Message type:

```
---
string[] waypoint_id
int32[] observation_count
```

Returns a list of waypoints along with the number of observations collected at those waypoints.

Service name: `SemanticMapPublisher/WaypointInfoService`

## SensorOriginService

Message type:

```
string waypoint_id
---
geometry_msgs/Vector3 origin
```

Given a waypoint this service returns the origin from where the latest observation was acquired at that waypoint.

Service name: `SemanticMapPublisher/SensorOriginService`

## ObservationService
 
Message type:

```
string waypoint_id
float64 resolution
---
sensor_msgs/PointCloud2 cloud
```

Given a waypoint, and a resolution, this service returns the latest observation collected at that waypoint as a PointCloud with the specified resolution. 

Service name: `SemanticMapPublisher/ObservationService`

## ObservationOctomapService

Message type:
```
string waypoint_id
float64 resolution
---
octomap_msgs/Octomap octomap
```

Same as `ObservationService` but returns the latest observation as an Octomap.

Service name: `SemanticMapPublisher/ObservationOctomapService`

## WaypointTimestampService

Message type:

```
string waypoint_id
---
string[] waypoint_timestamps
```

Given a waypoint, this service returns the timestamps of all the observations collected at that waypoint, as a list. 

Service name: `SemanticMapPublisher/WaypointTimestampService`

## ObservationInstanceService

Message type:
```
string waypoint_id
int64 instance_number # convention 0 - oldest available
float64 resolution
---
sensor_msgs/PointCloud2 cloud
string observation_timestamp
```

Given a waypoint id, an instance number and a resolution, this service returns a particular instance from the observations collected at that particular waypoint, with the desired resolution, along with the timestamp of the observation (as opposed to `ObservationService` which returns the latest observation at that particular waypoint). 
Service name: `SemanticMapPublisher/ObservationInstanceService`

## ObservationOctomapInstanceService

Message type:

```
string waypoint_id
int64 instance_number # convention 0 - oldest available
float64 resolution
---
octomap_msgs/Octomap octomap
string observation_timestamp
```

Same as `ObservationInstanceService`, but returns the observation instance as an `Octomap`. 

Service name: `SemanticMapPublisher/ObservationOctomapInstanceService`
