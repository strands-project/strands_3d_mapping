## Quasimodo Test

This package contains a simple test node that demonstrates how to put
together the `metaroom_xml_parser` together with the `quasimodo_retrieval_server`
for retrieving and `quasimodo_visualization_server` for visualizing a query.
Run the node with the command

`rosrun quasimodo_test test_msgs _data_path:=/path/to/labeled/data`.

Note that you will also need to run the two servers above, both in the
package `quasimodo_retrieval`. You should get output on the topic
`visualization_image` and you can use e.g. Rviz to visualize the image.
