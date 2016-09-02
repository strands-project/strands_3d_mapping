uint64 ALL_QUERY = 0
uint64 MONGODB_QUERY = 1
uint64 METAROOM_QUERY = 2
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image image
sensor_msgs/Image depth
sensor_msgs/Image mask
geometry_msgs/Pose global_pose
sensor_msgs/CameraInfo camera
int32 number_query
geometry_msgs/Transform room_transform
uint64 query_kind
