for f in $(find . -name '*room.xml'); do
    p=$(dirname "${f}")
    c=$(find "${p}" -name 'intermediate*.pcd' | wc -l)
    if [ $c -lt 17 ]; then
        echo $p
    fi
done
