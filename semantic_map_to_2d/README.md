This package contains a single map server node that publishes a downprojected metric map. 
A service is provided for chaning which waypoint is currently being published.

# Usage

Start it with

`rosrun semantic_map_to_2d semantic_map_2d_server`

This then provides the service `/set_waypoint` with the following format [TODO: This should be moved to the nodes namespace...]

Request:
- `string waypoint` : The name of the waypoint to switch the map to.

Response:
- `bool is_ok` : If the map was switched or not
- `string response` : Some textual description of what when wrong if not ok.

The map server publishes the map as a `nav_msgs::OccupancyGrid` on the topic `/waypoint_map`.
