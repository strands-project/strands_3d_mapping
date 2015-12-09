for f in $(find /home/nbore/Data/KTH_longterm_dataset_labels -name '*room.xml'); do
    rosrun surfelize_it surfelize_it -l $f
done
