for f in $(find /home/nbore/Data/KTH_longterm_surfels -wholename '*surfel_map.pcd'); do
    DIR=$(dirname "${f}")
    mv $f $DIR/backup_complete_cloud.pcd
done
