for f in $(find . -name '*rgb_*_label_0.jpg'); do
    p=$(dirname "${f}")
    c=$(find "${p}" -name 'intermediate*.pcd' | wc -l)
    if [ $c -lt 17 ]; then
        echo $p
    fi
done
