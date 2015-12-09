for f in $(find /home/nbore/Data/KTH_longterm_dataset_labels -name '*surfel_map.pcd'); do
    rosrun surfel_publisher surfel_exporter _cloud_name:=$f _threshold:=0.3
done
