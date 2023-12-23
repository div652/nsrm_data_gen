import os
import shutil
import tqdm
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--root_dir', required = True, type = str)
args = parser.parse_args()

root_dir = args.root_dir

datasets = ['train', 'test', 'val']
max_ex = [0, 0, 0]


for i, dset in enumerate(datasets):
    if not os.path.exists(dset):
        # make train, test, val directories, if not alreadt presenet. 
        os.mkdir(dset)
        for dir in next(os.walk(dset))[1]:
            max_ex[i] = max(max_ex[i], int(os.path.join(dset, dir)))
# after this we store the current max_example number of each directory. 

# dir will iterate over all the dubdirectories of root directory. 
for dir in tqdm.tqdm(next(os.walk(root_dir))[1]):
    # check if the directory name is tmp_train* or tmp_val* or tmp_test*
    for i, dset in enumerate(datasets):
        if "tmp_"+dset in dir:
            # example now iterates over all the examples present in the tmp_ directory. 
            for example in next(os.walk(os.path.join(root_dir, dir)))[1]:
                # check if the example contains S00, S01 and demo.json, else skip it. 
                example_path = os.path.join(root_dir, dir, example)
                valid = True
                for f in ['S00', 'S01', 'demo.json']:
                    if not os.path.exists(os.path.join(example_path, f)):
                        valid = False
                
                if valid:
                    shutil.copytree(example_path, os.path.join(dset, str(max_ex[i]).zfill(5)))
                    max_ex[i] += 1
                    # this will remove the tmp_train/example_num directory thus freeing up space. 
                    shutil.rmtree(example_path)

        