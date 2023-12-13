#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : base.py
# Author : Rahul Jain
# Email  : rahuljain13101999@gmail.com
# Date   : 19/06/2021

import os
import cv2
import json
import math
import sys
import numpy as np

from PIL import Image
from panda.world import PandaWorld
from panda.objects import Cube, Dice, Lego
from panda.world import PandaState
from panda.settings import camera_settings,CameraViews
import time

class ConstructBase(PandaWorld):
	def __init__(self, bullet_client, offset, config: dict, height, width, instance_dir, set_hide_panda_body=True):
		super().__init__(bullet_client, offset, config, height, width)
		self.instance_dir = instance_dir
		self.mapping = list()
		if set_hide_panda_body:
			self.hide_panda_body()
		self.position_list = list()

		while self.state == PandaState.INIT:
			self.update_state()
			self.step()
			bullet_client.stepSimulation()

	def hide_panda_body(self):
		for link_idx in range(-1, 11):
			self.bullet_client.changeVisualShape(self.panda, link_idx, rgbaColor=[0, 0, 0, 0])
		self.panda_visible = False
	
	def show_panda_body(self):
		for link_idx in range(-1, 11):
			self.bullet_client.changeVisualShape(self.panda, link_idx, rgbaColor=[1., 1., 1., 1.])
		self.panda_visible = True

	## -------------------------------------------------------------------------
	## Saving Instances and Demonstration Info
	## -------------------------------------------------------------------------
	# def depth_color_map(self, depth):
	# 	depth = (depth - 0.90)*10*255
	# 	depth_colormap = cv2.convertScaleAbs(depth)
	# 	return depth_colormap

	def save_instance(self,use_panda=False):
		w = self.width
		h = self.height
		instance_dir = self.instance_dir
		# instance_dir = '/home/vishal/projects/nsrmp/new_saved'
		demo_states = [os.path.join(instance_dir, f) for f in os.listdir(instance_dir)]
		demo_states = [s for s in demo_states if os.path.isdir(s)]
		instance_folder = "S" + "{0:0=2d}".format(len(demo_states))
		projmatrix = self.bullet_client.computeProjectionMatrixFOV(fov=90,aspect=1,nearVal=0.1,farVal=1.40)
		
		# filepaths 
		os.mkdir(os.path.join(instance_dir, instance_folder))
  
		for viewName, viewInfo in CameraViews.items():
			viewMatrix = self.bullet_client.computeViewMatrixFromYawPitchRoll(viewInfo['cameraTargetPosition'],viewInfo['cameraDistance'],viewInfo['cameraYaw'],viewInfo['cameraPitch'],0.0,2)
			# viewMatrix = self.bullet_client.computeViewMatrix(cameraEyePosition = [-10,0,5],cameraTargetPosition=[0,0,0],cameraUpVector=[1,0,0])
			# self.bullet_client.resetDebugVisualizerCamera(**viewInfo)

			for panda_visible in [True, False]:
				if (not use_panda) and panda_visible:
					continue
				if not panda_visible:
					self.hide_panda_body()
				else:
					self.show_panda_body()
     
				_, _, rgba, depth, mask = self.bullet_client.getCameraImage(width = w, height = h, viewMatrix = viewMatrix,projectionMatrix=projmatrix,renderer=self.bullet_client.ER_TINY_RENDERER)
    
				# _, _, rgba, depth, mask = self.bullet_client.getCameraImage(width = w, height = h)
				# Get Instance Folder
	
				type_folder = viewName + '-panda' if panda_visible else viewName
				folder = os.path.join(instance_dir, instance_folder, type_folder)
				os.mkdir(folder)
				img_filepath = os.path.join(folder,f"rgba.png")
				depth_filepath = os.path.join(folder,  f"depth.png")
				mask_filepath = os.path.join(folder, f"mask.png")

				def depth_color_map(depth):
					# print(depth[400:420,400:420])
					depth = ((0.140)/(1.40-(1.30)*depth))
					depth = ((depth-0.1)/1.30)*255
					return cv2.convertScaleAbs(depth)
					
				mask[mask < 3] = 0
				mask = cv2.convertScaleAbs(mask)
				depth = depth_color_map(depth)
    
				# print(np.unique(mask))
				# Save Images
				Image.fromarray(rgba, 'RGBA').save(img_filepath)
				Image.fromarray(depth, 'L').save(depth_filepath)
				Image.fromarray(mask, 'L').save(mask_filepath)
    
		if use_panda:
			self.show_panda_body()
		#self.bullet_client.resetDebugVisualizerCamera(**camera_settings['small_table_view'])

	
	def get_scene_info(self):
		info = {}
		info['objects'] = []
		for idx,obj in enumerate(self.objects):
			current_object = {}
			current_object["type"] = obj.type
			current_object["object_idx"] = idx 
			current_object["color"] = obj.color
			current_object['position'] = self.position_list[0][idx]
			current_object["rotation"] = obj.rotation # here rotation is measured in Quaternions
			info['objects'].append(current_object)
		info['object_color'] = [o.color[0] for o in self.objects]
		info['object_type'] = [o.type for o in self.objects]
		info['object_positions'] = self.position_list
		return info
	
	def save_position_info(self):
		l = [o.position for o in self.objects]
		self.position_list.append(l)

	## -------------------------------------------------------------------------
	## Applying Programs :
	## -------------------------------------------------------------------------
	def apply_program(self):
		"""
			Moves the Objects to their target positions.
			The target positions are determined at the time of action_compatability tests. 
		"""
		program = self.get_program()
		print("Trying to apply progam ",program)
		for idx, action in enumerate(program):
			# if only here to save scene, called by idle nodes. 
			if(action[0]=="SaveScene"):
				self.save_instance()
				continue
			move_object = action[1]
			self.move_object(move_object, self.target_position[idx])
			print("Saving position info for " , idx,action)
			self.save_position_info()
			# action[4] stores whether the scenes must be stored or not. By default is True, can turn it off by setting "show" : false in nodes in templates 
			if(action[4]):
				self.save_instance()

	def is_clear(self, object_positions, target_position, skip):
		for i in range(0, len(object_positions)):
			if i == skip: continue
			assert type(object_positions) == list and type(target_position) == list
			x, y, z = list(np.array(object_positions[i]) - np.array(target_position))
			if max(abs(x),abs(y)) < 0.075 and abs(z) < 0.05: # 0.05 is the dim of cube. 0.75 = 0.5+0.25 is taken for safety margin. 
				print("Clear issue at ",i)
				print("The positions are , ",object_positions)
				return False
		return True

	def move_object(self, move_obj_idx, target_pos, use_panda = False, timeStep = None, initial_pos = None, adjust_horizontal=False):
		""" Moves Object (By Index) To The Target Position
			Inputs: 
				move_obj_id(int): The index of the object being moved
				target_pos(3-tuple): The target position of the moved object 
				use_panda(bool): If False, the object will be moved by deleting and creating a new object in the target position. 
									If True, the panda robot will be simulated to move the object to the target. 	
		"""
		if use_panda == True:
			# print(move_obj_idx, target_pos, use_panda, timeStep, initial_pos, adjust_horizontal, gripper_error)
			m_object = self.objects[move_obj_idx]
			obj_id = m_object.object_idx
			pos_init, _ = self.bullet_client.getBasePositionAndOrientation(obj_id)
			m_object.position = list(pos_init)
			self.executeCommand(m_object.position, target_pos, move_obj_idx)
			# m_object.position = target_pos
			self.adjust_horizontal = adjust_horizontal
			stuck = False
			reachable = False
			while self.state != PandaState.IDLE:
				# print(self.state)
				self.update_state()
				self.step()
				if self.state == PandaState.POST_GRASP:
					pos, _ = self.bullet_client.getBasePositionAndOrientation(obj_id)
					if pos[2] > 0.8:
						reachable = True
				if self.state == PandaState.POST_RELEASE:
					pos, _ = self.bullet_client.getBasePositionAndOrientation(obj_id)
					if pos[2] > 0.8:
						stuck = True
				self.bullet_client.stepSimulation()
				time.sleep(self.control_dt)
			
			pos, _ = self.bullet_client.getBasePositionAndOrientation(obj_id)

			for i in range(len(pos)):
				if abs(m_object.position[i] - pos[i]) > 1e-2:
					reachable = True
					break
			m_object.position = list(pos) 
			return reachable, stuck
		else:
			m_object = self.objects[move_obj_idx]
			self.bullet_client.removeBody(m_object.object_idx)
			color_idx = m_object.color[0]
			if m_object.type == 'Cube':
				self.objects[move_obj_idx] = Cube(self.bullet_client, self.flags, target_pos, color_idx,orn = self.obj_orn)
				# print("move_obj   ",self.objects[move_obj_idx].object_idx)
			elif m_object.type == 'Dice':
				self.objects[move_obj_idx] = Dice(self.bullet_client, self.flags, target_pos, color_idx, orn = self.obj_orn)
			elif m_object.type == 'Lego':
				self.objects[move_obj_idx] = Lego(self.bullet_client, self.flags, target_pos, color_idx, orn = self.obj_orn)
			else:
				print(f"No Implementation for Object Type:{m_object.type} in move_object")
				raise NotImplementedError()


	def check_action_compatibility(self, program, block_positions: list):
		print("checking action compatibabilty")
		import copy
		target_positions = list()
		block_positions_cur = copy.deepcopy(block_positions)
		
		#Here program is a sequence of subtasks and each subtask is a 3-tuple
		for subtasks in program:
			action, move_object_idx, base_object_idx,distance,_ = subtasks
			if(action=='SaveScene'):
				target_positions.append([])
				continue
			if move_object_idx == base_object_idx: return None

			# check if move object doesn't have anything on top
			up_pos = self.top_target_pos(move_object_idx, move_object_idx, block_positions_cur)
			if not self.is_clear(block_positions_cur, up_pos, move_object_idx):
				print("Not clear from top ")
				return None

			# check if target position valid
			if action == 'TOP':
				tr_pos = self.top_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				if self.is_clear(block_positions_cur, tr_pos, base_object_idx):
					target_positions.append(tr_pos)
					block_positions_cur[move_object_idx] = tr_pos
				else:
					print("top positon not empty")
					return None
			elif action == 'LEFT':
				tr_pos = self.left_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				if self.is_clear(block_positions_cur, tr_pos, base_object_idx):
					target_positions.append(tr_pos)
					block_positions_cur[move_object_idx] = tr_pos
				else:
					print("Shift position not empty")
					print("Target position is ",tr_pos)
					print("block_positions right now are ",block_positions) 
					# self.save_instance()
					return None
			elif action == 'RIGHT':
				tr_pos = self.right_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				if self.is_clear(block_positions_cur, tr_pos, base_object_idx):
					target_positions.append(tr_pos)
					block_positions_cur[move_object_idx] = tr_pos
				else:
					print("right position not empty")
					return None
			elif action == 'FRONT':
				tr_pos = self.front_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				if self.is_clear(block_positions_cur, tr_pos, base_object_idx):
					target_positions.append(tr_pos)
					block_positions_cur[move_object_idx] = tr_pos
				else:
					return None
			elif action == 'BACK':
				tr_pos = self.back_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				if self.is_clear(block_positions_cur, tr_pos, base_object_idx):
					target_positions.append(tr_pos)
					block_positions_cur[move_object_idx] = tr_pos
				else:
					return None
			elif action == "INSIDE":
				tr_pos = self.inside_target_pos(base_object_idx, move_object_idx, block_positions_cur,distance)
				#Assumption: No need to check is_clear(). It's okay to collide when keeping inside a tray/box
				target_positions.append(tr_pos)
				block_positions_cur[move_object_idx] = tr_pos
			
			else:
				print(f"No Implementation for action_compatible:{action[0]} in N_SameAction")
				raise NotImplementedError()
		print("returning True Compatibbality" )
		return target_positions

