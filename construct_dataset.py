from construct import construct_main
import json
import configs
import random
import os
import subprocess
import tempfile
import json

# CHUNK_SIZE is the max number of examples per tmp_train folder
# eg. if 2k examples of a template are generated, 40 different tmp_train folders will be instantated. 
# This is to leverage the speed up due to the parallel generation of examples. 
CHUNK_SIZE= 50

template_file = {
    (1, False): "SingleStep.json",
    (1, True): "RelationalSingleStep.json",
    (2, False): "DoubleStep.json",
    (2, True): "RelationalDoubleStep.json",
    (6, False) :  "SixStep.json",
    (0,False): "TwinTowers.json",
    (-1, False): "TwinTowersV2.json",
    (8, False): "TwinTowers_2.json",
    (7, False): "TwinTowersV2_2.json",
    (9, False): "TwinTowers_3.json",
    (10, False): "TwinTowers_4.json",
    (11,False) : "GapLeft_0_3.json",
    (12,False) : "GapLeft_0_4.json",
    (13,False) : "GapLeft_0_5.json",
    (14,False) : "GapLeft_1_3.json",
    (15,False) : "GapLeft_1_4.json",
    (16,False) : "GapLeft_1_5.json",
    (17,False) : "GapLeft_2_3.json",
    (18,False) : "GapLeft_2_4.json",
    (19,False) : "GapLeft_2_5.json",
    (20,False) : "GapLeft_3_3.json",
    (21,False) : "GapLeft_3_4.json",
    (22,False) : "GapLeft_3_5.json",
    (23,False) : "GapRight_0_3.json",
    (24,False) : "GapRight_0_4.json",
    (25,False) : "GapRight_0_5.json",
    (26,False) : "GapRight_1_3.json",
    (27,False) : "GapRight_1_4.json",
    (28,False) : "GapRight_1_5.json",
    (29,False) : "GapRight_2_3.json",
    (30,False) : "GapRight_2_4.json",
    (31,False) : "GapRight_2_5.json",
    (32,False) : "GapRight_3_3.json",
    (33,False) : "GapRight_3_4.json",
    (34,False) : "GapRight_3_5.json",
    (35,False) : "GapFront_0_3.json",
    (36,False) : "GapFront_0_4.json",
    (37,False) : "GapFront_0_5.json",
    (38,False) : "GapFront_1_3.json",
    (39,False) : "GapFront_1_4.json",
    (40,False) : "GapFront_1_5.json",
    (41,False) : "GapFront_2_3.json",
    (42,False) : "GapFront_2_4.json",
    (43,False) : "GapFront_2_5.json",
    (44,False) : "GapFront_3_3.json",
    (45,False) : "GapFront_3_4.json",
    (46,False) : "GapFront_3_5.json",
    (47,False) : "GapBack_0_3.json",
    (48,False) : "GapBack_0_4.json",
    (49,False) : "GapBack_0_5.json",
    (50,False) : "GapBack_1_3.json",
    (51,False) : "GapBack_1_4.json",
    (52,False) : "GapBack_1_5.json",
    (53,False) : "GapBack_2_3.json",
    (54,False) : "GapBack_2_4.json",
    (55,False) : "GapBack_2_5.json",
    (56,False) : "GapBack_3_3.json",
    (57,False) : "GapBack_3_4.json",
    (58,False) : "GapBack_3_5.json",
    (59,False) : "TowerCopyRight3.json"
}
template_file_prefix = './panda/construct/templates/'
metadata_file = './panda/construct/metadata.json'

with open('curriculum.json', 'r') as f:
    data = json.load(f)
processes = []


for dataset in ['train','val','test']:
    count_downscale = data[dataset + '_count_downscale']
    dirnum = 0  
    for c, category in enumerate(data['categories']):
        total_num_examples = category['count']
        batches = ((total_num_examples-1)//CHUNK_SIZE)+1
        for i in range(batches):
            num_examples = 0 
            if(i<batches-1):
                num_examples = CHUNK_SIZE
            else : 
                num_examples = ((category['count']-1)%CHUNK_SIZE)+1
            num_examples = num_examples // count_downscale
            f = tempfile.TemporaryFile()
            dir_name = 'tmp_' + dataset + "-" + str(dirnum)
            dirnum+=1
            command = f'python3 construct.py --template_file {os.path.join(template_file_prefix, template_file[(category["steps"], category["relational"])])} --metadata_file {metadata_file} --dataset_dir {dir_name} --type {category["type"]} --max_objects {category["num_objects"]} --language {category["language"]} --num_examples {num_examples}'
            print(command)
            p = subprocess.Popen(command.split(), stdout=f)
            processes.append((p,f))

logfile = open('log.txt', 'wb')

for p, f in processes:
    p.wait()
    f.seek(0)
    logfile.write(f.read())
    f.close()
