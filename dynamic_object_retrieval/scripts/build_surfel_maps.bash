for f in $(find 'KTH_Objects' -name '*room.xml'); do
    rosrun surfelize_it surfelize_it -l "$f"
done
