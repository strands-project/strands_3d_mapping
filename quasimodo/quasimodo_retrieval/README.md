# Quasimodo Retrieval

This package contains nodes to interface with the `object_3d_retrieval` and
`object_3d_benchmark` packages. They are meant to be used together with
with the scripts in `object_3d_benchmark` to build a feature representation
than can be used by these nodes for querying. Please look at the package
`quasimodo_test` (specifically the node `test_msgs`) for an example of this in action.

## Retrieval Server

The retrieval server basically takes a point cloud and queries for
similar objects observed previously within our feature representation.
The definition of the service looks like:

```
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image image
sensor_msgs/CameraInfo camera
---
retrieval_result result
```

Please check the message `quasimodo_msgs/retrieval_result` for the exact format
of the return type. Invoke a server instance that can be called with the
above service definition by running

`rosrun quasimodo_retrieval quasimodo_retrieval_server _vocabulary_path:=/path/to/vocabulary`.

## Visualization Server

The idea is that the output from the retrieval server can be sent on to
another server for visualization. The original query data, together with the
result from the retrieval server is passed on to the visualization server.
It offers the following service:

```
sensor_msgs/Image image
sensor_msgs/CameraInfo camera
geometry_msgs/Transform room_transform
retrieval_result result
---
sensor_msgs/Image image
```

Run the node by simply typing

`rosrun quasimodo_retrieval quasimodo_visualization_server`.
