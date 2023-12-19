#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : get_scenes_json.py
# Author :Vishal Bindal


import json
import argparse
import os
from datetime import date
import cv2
import numpy as np
from tqdm import tqdm
from panda.settings import directions 
from collections import defaultdict


def get_bbox(image_mask_path, image_depth_path, bbox_mode = 'yxhw'):
    masks = [cv2.imread(mask_path, 0) for mask_path in image_mask_path]
    depths = [cv2.imread(depth_path, 0) for depth_path in image_depth_path]
    
    bboxes_info = defaultdict(lambda: defaultdict(list))
    
    for scene_id, (mask, depth) in enumerate(zip(masks,depths)):
    
        obj_ids = np.unique(mask)

        #previously it was h,w,_ = mask.shape. Namas changed it to h,w
        h,w = mask.shape
        
        for id in obj_ids:
            if id==0:
                continue
            if bbox_mode == 'yxhw':
                py, px = np.where(mask == id)
                bbox = [np.min(px), np.min(py), np.max(px), np.max(py)]
                center_x = (bbox[0] + bbox[2])//2
                center_y = (bbox[1] + bbox[3])//2
                # Note: the opencv representation of image is (rows,columns)
                depth_center = depth[center_y, center_x] / 255

                #Normalize bbox
                bbox = [np.min(py)/h, np.min(px)/w, (np.max(py) - np.min(py))/h, (np.max(px) - np.min(px))/w]
                this_depths = [depth[y,x] for (x,y) in zip(px,py)]
                depth_min = min(this_depths)/255
                depth_max = max(this_depths)/255
                
            else:
                raise NotImplementedError      
            bboxes_info[id.item()][scene_id] = list(map(float, bbox + [depth_center,depth_min,depth_max]))

    total_objs = sorted(list(bboxes_info.keys()))
    total_scenes = len(masks)

    bboxes = []
    for obj_id in total_objs:
        this_obj_bboxes = {}
        for scene_id in range(total_scenes):
            #if a particlar object is detected, then append the corresponding bbox or append 5 zeros for corners and depth
            if scene_id in bboxes_info[obj_id]:
                this_obj_bboxes[image_mask_path[scene_id].split('/')[-2]] = (bboxes_info[obj_id][scene_id])
            else:
                this_obj_bboxes[image_mask_path[scene_id].split('/')[-2]] = ([0]*5) 
        bboxes.append(this_obj_bboxes)

    return bboxes


def get_scene_info(images_dir, demo_json_path, demonstration_id, scene_no, split, eps):
    subdirs = [f.path for f in os.scandir(images_dir) if f.is_dir()]
    subdirs.sort()
    
    rgb_path, mask_path, depth_path = [], [], []
    for i, dir in enumerate(subdirs):
        rgb_path.append(os.path.join(dir, 'rgba.png'))
        mask_path.append(os.path.join(dir, 'mask.png'))
        depth_path.append(os.path.join(dir, 'depth.png'))
    

    with open(demo_json_path, 'r') as f:
        demo_json = json.load(f)
    scene = {
        'image_filename': {path.split('/')[-2]: '/'.join(path.split('/')[-4:]) for path in rgb_path}, #TODO: @Namas currently the rgb_path is ../data/0000/S00 etc. We are handcodedly extracting 0000/S00 from that. Need to change this in future 
        'demonstration_index': demonstration_id,
        'scene_no':scene_no,
        'objects': [],
        'directions': directions,
        'relationships': {},
        'split': split
    }
    
    bboxes = get_bbox(mask_path, depth_path, bbox_mode= 'yxhw')
    
    # Occlusion
    if len(bboxes) != len(demo_json['objects']):
        return None

    for i, obj in enumerate(demo_json['objects']):
        object = {}
        object['id'] = i
        object['3d_coords'] = demo_json['object_positions'][scene_no][i]
        object['color'] = obj['color'][1]
        object['type'] = obj['type']
        object['bbox'] = {key: value[:-2] for key, value in bboxes[i].items()}
        object['depths'] = {key: value[-3:] for key, value in bboxes[i].items()}
        # object['pixel_coords'] = [(bboxes[i][0]+bboxes[i][2])/2, (bboxes[i][1]+bboxes[i][3])/2, bboxes[i][4]]
        object['rotation'] = obj['rotation']
        scene['objects'].append(object)

    # Compute relationships
    # num_obj = len(bboxes)
    # for dir, vec in directions.items():
    #     scene['relationships'][dir] = []
    #     for i in range(num_obj):
    #         coords_i = scene['objects'][i]['3d_coords']
    #         related_i = []
    #         for j in range(num_obj):
    #             if j==i:
    #                 continue
    #             coords_j = scene['objects'][j]['3d_coords']
    #             diff = np.array(coords_j) - np.array(coords_i)
    #             dot = np.dot(np.array(vec), diff)/np.linalg.norm(np.array(vec),2)
    #             if dot > eps:
    #                 related_i.append(j)
    #         scene['relationships'][dir].append(related_i)

    return scene

def main(dataset_dir, split, out_path, eps=0.075, out_dir="."):
    dump = {
        'info':{
            'date': str(date.today()),
            'split': split
        },
        'scenes': []
    }

    f = open(os.path.join(out_dir, f'occlusion-cases-{split}.txt'), 'w')

    all_dirs = [os.path.join(dataset_dir, f) for f in os.listdir(dataset_dir) if os.path.isdir(os.path.join(dataset_dir, f))]
    all_dirs.sort()
    for id, dir in tqdm(enumerate(all_dirs)):
        scenes = []
        subdirs = [f.path for f in os.scandir(dir) if f.is_dir()]
        subdirs.sort()
        for i,subdir in enumerate(subdirs[:]):
            # print(subdir)
            scenes.append(get_scene_info(subdir, os.path.join(dir, 'demo.json'), id, i, split, eps))
        if None in scenes:
            f.writelines(dir + '\n')
            continue
        dump['scenes'].append(scenes)


    with open(os.path.join(out_dir, out_path), 'w') as f:
        json.dump(dump, f, indent=4)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', help='Relative path to the Dataset Directory')
    parser.add_argument('--split', required = True, help='Split (train/val/test)')
    parser.add_argument('--out_path', default='scenes.json', help='Output path')
    parser.add_argument('--eps', default=0.075, help='Epsilon for computing relationships')
    args = parser.parse_args()

    main(args.dataset_dir, args.split, args.out_path, args.eps)
    
