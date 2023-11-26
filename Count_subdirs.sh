for dir in */; do
    subdirectories=$(find "$dir" -type d -maxdepth 1 | wc -l)
    echo "Number of subdirectories in $dir: $subdirectories"
done
