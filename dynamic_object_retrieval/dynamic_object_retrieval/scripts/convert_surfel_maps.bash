for f in $(find . -name '*surfel_map.pcd'); do
    rosrun surfel_publisher surfel_exporter _cloud_name:=$f _threshold:=0.3
done
