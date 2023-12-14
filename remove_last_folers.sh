for dir in ./tmp_train*/; do
    innermost_dir=$(ls -d "$dir"*/ | tail -n 1)
    rm -r "$innermost_dir"
done