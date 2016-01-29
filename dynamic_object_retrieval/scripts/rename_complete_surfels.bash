for f in $(find . -wholename '*backup_complete_cloud.pcd'); do
    DIR=$(dirname "${f}")
    mv $f $DIR/complete_cloud.pcd
done
