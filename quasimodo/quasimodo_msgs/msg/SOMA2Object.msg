Header header

string id			#### id of the object that is inserted. Useful for identifying objects. SOMA2 delete service use this id to delete objects from DB.

string map_name 	#### the global map that the object belongs. Automatically filled by SOMA2 insert service 

string map_unique_id 	#### the unique db id of the map. Automatically filled by SOMA2 insert service

string config 		#### the corresponding world configuration. This could be incremented as config1, config2 at each meta-room observation

string mesh 		#### mesh model of the object. Could be left blank

string type 		#### type of the object. For higher level objects, this could chair1, chair2. For middle or lower level segments it could be segment1101, segment1102, etc.

string waypoint 	#### the waypoint id. Could be left blank

int32 timestep 		#### this is the discrete observation instance. This could be incremented at each meta-room observation as 1,2,3,etc...

uint32 logtimestamp 	#### this is the unix timestamp information that holds the time when the object is logged. SOMA2 uses UTC time values.

int8 logday 		#### the day of week. Automatically filled by SOMA2 insert service when logtimestamp info is provided
int8 loghour 		#### log hour.  Automatically filled by SOMA2 insert service when logtimestamp info is provided
int8 logminute 		#### log minute.  Automatically filled by SOMA2 insert service when logtimestamp info is provided
int32 logtimeminutes 	#### Automatically filled by SOMA2 insert service. Simplifies queries

geometry_msgs/Pose pose 	#### Object pose in 3D			(with respect to /map frame)
geometry_msgs/Pose sweepCenter 	#### The pose of the robot during sweep (with respect to /map frame)
sensor_msgs/PointCloud2 cloud 	#### The 3D cloud of the object		(with respect to /map frame)


string geotype 				#### geotype: Point,Polygon,etc. For SOMA2 objects, it is automatically filled as 'Point'

geometry_msgs/PoseArray geoposearray 	#### the poses in geospatial format lat,long size= posearray+1 for forming the polygon

