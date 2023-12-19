
# python3 construct_dataset.py
 
# python3 move_files.py --root_dir .
 
python3 get_scenes_json.py --dataset_dir train --split train --out_path scenes-depth-train.json

# python3 get_instructions_json.py --dataset_dir train --split train --out_path instructions-train.json
 
python3 get_scenes_json.py --dataset_dir test --split test --out_path scenes-depth-test.json
# python3 get_instructions_json.py --dataset_dir test --split test --out_path instructions-test.json
 
python3 get_scenes_json.py --dataset_dir val --split val --out_path scenes-depth-val.json

# python3 get_instructions_json.py --dataset_dir val --split val --out_path instructions-val.json

# Note: in case of errors, manually debug and delete example please
# Disclaimer: We don't gaurantee sanity of generated examples

# python get_scenes_json.py --dataset_dir /home/vishal/projects/nsrmp/recons/test --split test --out_path /home/vishal/projects/nsrmp/recons/scenes-test.json
