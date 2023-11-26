#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : base.py
# Author : Rahul Jain
# Email  : rahuljain13101999@gmail.com
# Date   : 19/06/2021

import math
import random
import numpy as np
import json
from copy import deepcopy, copy

from .settings import *
from .objects import Cube, Dice, Tray, Lego


# Panda Arm Position/Orientaion
PANDA_NUM_DOF = 7
END_EFFECTOR_INDEX = 8 
PANDA_POSITION = np.array([0, 0.5, 0.5])

# Inverse Kinematics
lower_limit = [-7]*PANDA_NUM_DOF
upper_limit = [7]*PANDA_NUM_DOF
joint_range = [7]*PANDA_NUM_DOF
rest_position = [0, 0, 0, -2.24, -0.30, 2.66, 2.32, 0, 0]

# Gripping
# Already defined in settings.py
# TABLE_OFFSET = 0.64
MOVE_HEIGHT = TABLE_OFFSET + 0.40
GRASP_WIDTH = 0.01
RELEASE_WIDTH = 0.05
GRASP_HEIGHT = 0.10
ALPHA = 0.99
DROP_MARGIN = 0.001

TIME_STEP = [1.25, 2.0, 1.5, 0.5, 1.5, 2.0, 1.5, 0.5, 1.5]


class PandaState(Enum):
	INIT = 0
	MOVE = 1
	PRE_GRASP = 2
	GRASP = 3
	POST_GRASP = 4
	MOVE_BW = 5
	PRE_RELEASE = 6
	RELEASE = 7
	POST_RELEASE = 8
	IDLE = 9


class PandaWorld(object):
	def __init__(self, bullet_client, offset, config: dict, height, width):
		# Client
		self.bullet_client = bullet_client
		self.offset = np.array(offset)
		self.flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
		flags = self.flags

		self.height = height
		self.width = width
		self.table_offset = TABLE_OFFSET

		# Panda ARM
		panda_orn = self.bullet_client.getQuaternionFromEuler([0, 0, -math.pi/2])
		self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", self.offset + PANDA_POSITION, panda_orn, useFixedBase=True, flags=flags)
		finger_constraint = self.bullet_client.createConstraint(
			self.panda, 9, self.panda, 10,
			jointType=self.bullet_client.JOINT_GEAR,
			jointAxis=[1, 0, 0],
			parentFramePosition=[0, 0, 0],
			childFramePosition=[0, 0, 0])
		self.bullet_client.changeConstraint(finger_constraint, gearRatio=-1, erp=0.1, maxForce=50)
		self.panda_visible = True

		# Joint Panda Frame
		index = 0
		num_panda_joints = self.bullet_client.getNumJoints(self.panda)
		for joint_idx in range(num_panda_joints):
			self.bullet_client.changeDynamics(self.panda, joint_idx, linearDamping=0, angularDamping=0)
			info = self.bullet_client.getJointInfo(self.panda, joint_idx)
			jointType = info[2]
			if (jointType == self.bullet_client.JOINT_PRISMATIC) or (jointType == self.bullet_client.JOINT_REVOLUTE):
				self.bullet_client.resetJointState(self.panda, joint_idx, rest_position[index])
				index = index+1

		# Static Objects
		self.bullet_client.loadURDF("plane.urdf", offset, flags=flags)
		table = self.bullet_client.loadURDF("table/table.urdf", offset, flags=flags)
		self.bullet_client.changeVisualShape(table, -1, rgbaColor=[0.48, 0.435, 0.2, 1])

		# Maintain a list of all Objects
		self.objects = list()
		self.object_type = []

		self.PANDA_FINGER_HORIZONTAL = np.array([0, math.pi, math.pi/2])
		self.PANDA_FINGER_VERTICAL = np.array([0, math.pi, 0])

		# Initialize Variables
		self.t = 0
		self.control_dt = 1./240.
		self.state = PandaState.INIT
		self.finger_width = RELEASE_WIDTH
		self.gripper_height = MOVE_HEIGHT
		self.finder_orientation = self.PANDA_FINGER_VERTICAL


	def reset_world(self):
		self.bullet_client.resetSimulation()
		panda_orn = self.bullet_client.getQuaternionFromEuler([0, 0, -math.pi/2])
		self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", self.offset + PANDA_POSITION, panda_orn, useFixedBase=True, flags=self.flags)
		finger_constraint = self.bullet_client.createConstraint(
			self.panda, 9, self.panda, 10,
			jointType=self.bullet_client.JOINT_GEAR,
			jointAxis=[1, 0, 0],
			parentFramePosition=[0, 0, 0],
			childFramePosition=[0, 0, 0])
		self.bullet_client.changeConstraint(finger_constraint, gearRatio=-1, erp=0.1, maxForce=50)
		self.panda_visible = True

		# Joint Panda Frame
		index = 0
		num_panda_joints = self.bullet_client.getNumJoints(self.panda)
		for joint_idx in range(num_panda_joints):
			self.bullet_client.changeDynamics(self.panda, joint_idx, linearDamping=0, angularDamping=0)
			info = self.bullet_client.getJointInfo(self.panda, joint_idx)
			jointType = info[2]
			if (jointType == self.bullet_client.JOINT_PRISMATIC) or (jointType == self.bullet_client.JOINT_REVOLUTE):
				self.bullet_client.resetJointState(self.panda, joint_idx, rest_position[index])
				index = index+1

		# Static Objects
		self.bullet_client.loadURDF("plane.urdf", self.offset, flags=self.flags)
		table = self.bullet_client.loadURDF("table/table.urdf", self.offset, flags=self.flags)
		self.bullet_client.changeVisualShape(table, -1, rgbaColor=[0.48, 0.435, 0.2, 1])

		# Maintain a list of all Objects
		self.objects = list()
		self.object_type = []
		self.state = PandaState.INIT
		self.finger_width = RELEASE_WIDTH
		self.gripper_height = MOVE_HEIGHT
		self.finder_orientation = self.PANDA_FINGER_VERTICAL
  
		
	def relative_target_pos(self, relation, base_obj_id, move_obj_id, all_block_positions,distance=1):
		handlers = {
			'TOP': self.top_target_pos,
			'LEFT': self.left_target_pos,
			'RIGHT': self.right_target_pos,
			'FRONT': self.front_target_pos,
			'BACK': self.back_target_pos,
			# 'DIAG_45' : self.diag_45_target_pos,
			# 'DIAG_135' : self.diag_135_target_pos,
			# 'DIAG_225' : self.diag_225_target_pos,
			# 'DIAG_315' : self.diag_315_target_pos,
		}
		handler = handlers[relation]
		print(base_obj_id, move_obj_id, all_block_positions,distance)
		print(len(self.object_type))
		return handler(base_obj_id, move_obj_id, all_block_positions,distance)

	## -------------------------------------------------------------------------
	## Allocate Positions 
	## -------------------------------------------------------------------------

	# to check whether a target position is on the table.   
	def on_table(self,position,object_id):
		# position is the coordinate of the center of the object, if there are other 
		# comments that say otherwise, then they are incorrect.
		obj_dim = self.get_object_dim(object_id)
		center_x=position[0]
		center_y=position[1]
		return( ((center_x + obj_dim/2)<TABLE_BREADTH/2) and ((center_x - obj_dim/2)>(-TABLE_BREADTH/2)) and ((center_y + obj_dim/2)<TABLE_LENGTH/2) and ((center_y - obj_dim/2)>(-TABLE_LENGTH/2)))

	# note this function is dependant on the camera view , and the parameters need to be set accordingly
	# for now these are just hardcoded for the view that was being used while generation. 
	# this is for the front view . 
	#this also changes with changes in alpha and up_limit in get_random_table_poistion 

	def in_camera_view(self,position,object_id):
		center_x=position[0]
		center_y=position[1]
		up_limit = -0.4*WORK_AREA_LENGTH
		alpha = 1.1
		if(center_y>0):
			return True 
		else : 
			return abs(center_x)< 0.5*WORK_AREA_BREADTH*(1 - alpha*center_y/(up_limit))

	## Ealier the range was -0.25 to 0.25 .
	def get_random_table_position(self):
		up_limit = -0.4*WORK_AREA_LENGTH
		down_limit = 0.5*WORK_AREA_LENGTH
		y = np.random.uniform(down_limit,up_limit)
		alpha = 1.1
		if(y>=0.0):
			breadth = WORK_AREA_BREADTH 
		else: 
			breadth = WORK_AREA_BREADTH*(1 - alpha*y/(up_limit))
		left_limit = -0.5* breadth
		right_limit = 0.5* breadth
		x = np.random.uniform(left_limit, right_limit)
		var_xy = [x,y]
		# var_xy = WORK_AREA_SIDE*(np.random.rand(2) + np.array([-0.5, -0.5]))
		return self.offset + np.array([var_xy[0], var_xy[1], TABLE_OFFSET])
		
	def check_blocks_not_overlapping(self, block_positions, margin = 0.075,numSkips=0):
		print("checking non overlapping for ",block_positions)
		num_blocks = len(block_positions)
		
		for i in range(numSkips):
			for j in range(numSkips,num_blocks):
				x, y, _ = block_positions[i]-block_positions[j]
				max_d = max(abs(x), abs(y))
				if max_d < margin: print("got false, ",i,j)
				if max_d < margin: return False
       
		for i in range(numSkips,num_blocks):
			for j in range(i + 1, num_blocks):
				x, y, _ = block_positions[i]-block_positions[j]
				max_d = max(abs(x), abs(y))
				if max_d < margin: print("got false, ",i,j)
				if max_d < margin: return False
		return True


	# def get_block_positions(self, num_blocks,direction,nums):
	# 	'''
	# 	Return a list of random non-overlaping positions on the table. The positions are 3-D co-ordinates with reference to the bullet client. 
	# 	'''
	# 	valid, block_positions = False, None
	# 	while not valid:
	# 		block_positions = [self.get_random_table_position() for _ in range(num_blocks)]
	# 		valid = self.check_blocks_not_overlapping(block_positions) 
	# 	return block_positions
	def get_block_positions(self, num_blocks):
		'''
		Return a list of random non-overlaping positions on the table. The positions are 3-D co-ordinates with reference to the bullet client. 
		'''
		
		valid, block_positions = False, None
		while(not(valid)):
			print("Trying a valid position again")
			block_positions=[]
			notVisible=False
			firstBlockPosition = self.get_random_table_position()
			block_positions.append(firstBlockPosition)
			pseduoBlocks=0
			if 'checkOnTable' in self.template:
				# # for now we are assuming that check on table would only be utilised when there are no relations present. 
				# if('Relations' in self.template):
				# 	assert(len(self.template['Relations'])==0)
	
				pseduoBlocks=1
				keepPos = self.relative_target_pos(self.template['checkOnTable'][0],0,0,block_positions,self.template['checkOnTable'][1])
    
				print("Added block")
				if(not(self.on_table(keepPos,object_id=0)) or not(self.in_camera_view(keepPos,object_id=0))):
					continue
				else:
					block_positions.append(keepPos)
			relations = self.template["Relations"] if 'Relations' in self.template else []
			# print(len(relations),nums)
			nums=len(relations)
			for i in range(nums):
				if(i==0):
					relPos = np.array(self.relative_target_pos(relations[i],0,0,block_positions))
				else : 
					relPos = np.array(self.relative_target_pos(relations[i],pseduoBlocks+i,0,block_positions))
				block_positions.append(relPos)
				if(not(self.on_table(block_positions[i+1+pseduoBlocks],object_id =0)) or not(self.in_camera_view(block_positions[i+1+pseduoBlocks],object_id=0))):
					notVisible=True
					break
			
			# using object_id =0 as for now the ghost blocks are assumed to be identical to the first block in shape and color 
			if(notVisible):
				print("notVisible")
				continue
			print(num_blocks-nums-1)

			otherBlocks = [self.get_random_table_position() for _ in range(num_blocks-nums-1)]
			block_positions.extend(otherBlocks)
			valid = self.check_blocks_not_overlapping(block_positions,numSkips=nums+1+pseduoBlocks) 
			print("found ",valid)
		# we don't send pseduo blocks in outout
		block_positions = block_positions[:1]+block_positions[pseduoBlocks+1:]
		print("The positions that are being returned are ",block_positions)
		return block_positions
	
	# -------------------------------------------------------------------------
	## Get Positions W.R.T. Objects
	## -------------------------------------------------------------------------

	def get_object_dim(self, object_target_id):
		'''
		Inputs:
			object_target_id: The id of the object in the pandaworld. Note that this not the object id w.r.t bullet client
		Return:(float) The object size. Currently, we are having cubes only. So a single number is enough.  
		'''
		print("In obect dimensions",self.object_type,object_target_id)
		obj_type = self.object_type[object_target_id]
		if obj_type in object_dimensions.keys():
			return object_dimensions[obj_type]
		else:
			print(f"Unrecognized Object Type in ConstructWorld:get_object_dim {obj_type}")
			raise TypeError()


	def noise(self, sigma_p= 0.05, sigma_s=0.005):
		p, s = abs(np.random.normal(0, sigma_p, 1)[0]), np.random.normal(0, sigma_s, 1)[0]
		return 0, 0 #uncomment to remove randomness 
		return (p, s) if p > abs(s) else self.noise(sigma_p, sigma_s)
		

	def inside_target_pos(self, base_obj_id:int, move_obj_id:int, all_block_positions:list):
		'''
			Inputs: (Note that the indices are not the object id w.r.t bullet client. The bullet may have additional objects like table and plane.)
				base_obj_id(int): Index of the object(tray or container etc) w.r.t the action is performed
				move_obj_id(int): Dummy input(not required as of now). The index of the object being moved 
				all_block_positions:(list of 3-tuples) The list of positions of all objects in the PandaWorld
			Description: The target co-ordinates = (left-bottom corner of base_obj + the height of the base object + some_offset).
		'''
		base_pos = all_block_positions[base_obj_id]
		base_dim = self.get_object_dim(base_obj_id)
		blocks_positions = [np.array(b) for idx, b in enumerate(all_block_positions) if idx != base_obj_id ]
		for _ in range(1000):
			variation = np.random.uniform(low=0,high=base_dim[0]), np.random.uniform(low=0,high=base_dim[1])
			target_pos = [base_pos[0]+variation[0], base_pos[1] + variation[1], base_pos[2]+base_dim[2]/2]
			blocks_positions[move_obj_id] = np.array(target_pos)
			if self.check_blocks_not_overlapping(blocks_positions, margin = 0.001):	
				return target_pos
		return None
	def top_target_pos(self, base_obj_id:int, move_obj_id:int, all_block_positions:list,distance=1):
		'''
			Inputs: (Note that the indices are not the object id w.r.t bullet client. The bullet may have additional objects like table and plane.)
				base_obj_id(int): Index of the id w.r.t the action is performed
				move_obj_id(int): The index of the object being moved
				all_block_positions:(list of 3-tuples) The list of positions of all objects in the PandaWorld
			Description: The target z co-ordinate = The left-bottom corner of base_obj + the height of the base object.
		'''
		base_pos = all_block_positions[base_obj_id]
		base_dim = self.get_object_dim(move_obj_id)
		move_z_pos = base_pos[2] + distance*base_dim
		# print(base_pos[2],base_dim)
		target_pos = [base_pos[0], base_pos[1], move_z_pos]
		return target_pos

	def left_target_pos(self, base_obj_id, move_obj_id, all_block_positions,distance=1):
		'''
		Refer top_target_pos for input/output specs
		Description: The target x-cordiante = base_obj x co-ordinate - the length of the object being moved - MARGIN(to avoid overlapping). 
					 Here the length is same as that of the shape since the objects are cube. Refer panda.setting for more details
		'''
		base_pos = all_block_positions[base_obj_id]
		move_dim = self.get_object_dim(move_obj_id)
		variation = self.noise()
		# print("MARGIN:", MARGIN, "Noise:", variation)
		move_left_pos = base_pos[0] - distance*(move_dim + MARGIN + variation[0])
		target_pos = [move_left_pos, base_pos[1] + variation[1], base_pos[2]]
		return target_pos

	def back_target_pos(self, base_obj_id, move_obj_id, all_block_positions,distance=1):
		'''
		Refer top_target_pos for input/output specs
		Description: The target x-cordiante = base_obj y co-ordinate - the length of the object being moved - MARGIN(to avoid overlapping). 
					 Here the length is same as that of the shape since the objects are cube. Refer panda.setting for more details
		'''
		base_pos = all_block_positions[base_obj_id]
		move_dim = self.get_object_dim(move_obj_id)
		variation = self.noise()
		# print("MARGIN:", MARGIN, "Noise:", variation)
		move_back_pos = base_pos[1] -distance* (move_dim + MARGIN + variation[0])
		target_pos = [base_pos[0] + variation[0], move_back_pos,  base_pos[2]]
		return target_pos


	def right_target_pos(self, base_obj_id, move_obj_id, all_block_positions,distance=1):
		'''
		Refer top_target_pos for input/output specs
		Description: The target x-cordiante = base_obj x co-ordinate + the length of the base object - MARGIN(to avoid overlapping). 
					 Here the length is same as that of the shape since the objects are cube. Refer panda.setting for more details
		'''
		base_pos = all_block_positions[base_obj_id]
		base_dim = self.get_object_dim(move_obj_id)
		variation = self.noise()
		# print("MARGIN:", MARGIN, "Noise:", variation)
		move_right_pos = base_pos[0] + distance*(base_dim  + MARGIN + variation[0])
		target_pos = [move_right_pos, base_pos[1], base_pos[2]]
		return target_pos

	def front_target_pos(self, base_obj_id, move_obj_id, all_block_positions,distance=1):
		'''
		Refer top_target_pos for input/output specs
		Description: The target x-cordiante = base_obj y co-ordinate + the length of the object being moved - MARGIN(to avoid overlapping). 
					 Here the length is same as that of the shape since the objects are cube. Refer panda.setting for more details
		'''
		base_pos = all_block_positions[base_obj_id]
		base_dim = self.get_object_dim(move_obj_id)
		variation = self.noise()
		# print("MARGIN:", MARGIN, "Noise:", variation)
		move_front_pos = base_pos[1] + distance*(base_dim  + MARGIN + variation[1])
		target_pos = [base_pos[0] , move_front_pos , base_pos[2]]
		return target_pos

	def staircase_target_pos(self, base_obj_id, move_obj_id, all_block_positions, height):
		self.height = height
		count = 0
		for h in range(1,height+1):
			target_pos = self.right_target_pos(base_obj_id, move_obj_id[count], all_block_positions)
			target_pos = self.top_target_pos(base_obj_id, move_obj_id, all_block_positions)
			target_pos[2] += h*self.get_object_dim(move_obj_id)[2]
			if self.check_blocks_not_overlapping(all_block_positions, margin = 0.001):
				return target_pos

        

	def update_state(self):
		""" 
			The Control Time of 1 Second For Each State of Panda Execution
			INIT -> Initialization State
			MOVE -> Source State, IDLE -> Terminal State
		"""
		if (self.state != PandaState.IDLE):
			self.t += self.control_dt
			if (self.t > TIME_STEP[self.state.value]):
				self.t = 0
				if self.state == PandaState.INIT:
					self.state = PandaState.IDLE
				else:
					self.state = PandaState(self.state.value+1)

	def pre_execute_command(self):
		pass

	def executeCommand(self, block_pos, target_pos, move_obj_idx):
		""" Execute Command / Initiliaze State, dt """
		if self.state == PandaState.IDLE:
			self.pre_execute_command()
			self.block_position = block_pos
			self.target_position = target_pos
			self.obj_dim = self.get_object_dim(move_obj_idx)
			self.state = PandaState.MOVE
			self.t = 0

	def isExecuting(self):
		""" If the Panda is in the middle of command Execution """
		return self.state != PandaState.IDLE

	def movePanda(self, pos):
		"""
			Given The Position of the Panda (End_Effector_Part)
			Move The Panda/Joints to reach the location 
		"""
		# Move Panda Body
		self.bullet_client.submitProfileTiming("IK")
		orn = self.bullet_client.getQuaternionFromEuler(self.finder_orientation)
		jointPoses = self.bullet_client.calculateInverseKinematics(
			self.panda, END_EFFECTOR_INDEX, pos, orn, lower_limit, upper_limit, joint_range, rest_position)
		self.bullet_client.submitProfileTiming()
		control_mode = self.bullet_client.POSITION_CONTROL
		for idx in range(PANDA_NUM_DOF):
			self.bullet_client.setJointMotorControl2(self.panda, idx, control_mode, jointPoses[idx], force=1200., maxVelocity=1.0)
		# Move Gripper 
		control_mode = self.bullet_client.POSITION_CONTROL
		self.bullet_client.setJointMotorControl2(self.panda, 9, control_mode, self.finger_width, force=100)
		self.bullet_client.setJointMotorControl2(self.panda, 10, control_mode, self.finger_width, force=100)

	def get_nearest_below(self):
		nearest_obj_id, least_dist = None, float('inf')
		for id, obj in enumerate(self.objects):
			# check if below
			obj_dim = self.get_object_dim(id)
			below = True
			for i in range(2):
				below = below and obj.position[i] <= self.pos[i] + self.obj_dim and obj.position[i] >= self.pos[i] - obj_dim
			if below:
				dist = sum([(self.pos[i] - obj.position[i])**2 for i in range(3)])
				if dist < least_dist:
					least_dist = dist
					nearest_obj_id = id
		return nearest_obj_id

	def fine_correct_release(self):
		# find nearest object
		nearest_obj_id = self.get_nearest_below()
		if nearest_obj_id is not None:
			dim = self.get_object_dim(nearest_obj_id)
			nearest_obj_pos = self.objects[nearest_obj_id].position
			# print(nearest_obj_id, nearest_obj_pos[2], self.pos[2])
			self.pos[2] = max(self.pos[2], nearest_obj_pos[2] + dim + GRASP_HEIGHT + DROP_MARGIN)
			if self.adjust_horizontal:
				self.pos[0] = nearest_obj_pos[0]
				self.pos[1] = nearest_obj_pos[1]

	def fine_correct_grip(self):
		# find nearest object
		nearest_obj_id = self.get_nearest_below()
		if nearest_obj_id is not None:
			nearest_obj_pos = self.objects[nearest_obj_id].position
			# print(nearest_obj_id, nearest_obj_pos[2], self.pos[2])
			self.pos[2] = max(self.pos[2], nearest_obj_pos[2] + GRASP_HEIGHT)
			if self.adjust_horizontal:
				self.pos[0] = nearest_obj_pos[0]
				self.pos[1] = nearest_obj_pos[1]

	def step(self):

		alpha_change_gripper_height = lambda x: ALPHA * self.gripper_height + (1.0-ALPHA)* x
		# alpha_change_gripper_height = lambda x: x
		
		if self.state == PandaState.INIT:
			self.pos = np.zeros(3)
			self.gripper_height = alpha_change_gripper_height(MOVE_HEIGHT)
			self.pos[2] = self.gripper_height
			self.movePanda(self.pos)

		if self.state == PandaState.MOVE:
			self.pos = self.block_position.copy()
			self.gripper_height = alpha_change_gripper_height(MOVE_HEIGHT)
			self.pos[2] = self.gripper_height
			self.movePanda(self.pos)

		elif (self.state == PandaState.PRE_GRASP):
			self.pos = self.block_position.copy()
			self.gripper_height = alpha_change_gripper_height(GRASP_HEIGHT+self.pos[2])
			self.pos[2] = self.gripper_height
			self.fine_correct_grip()
			self.movePanda(self.pos)

		elif self.state == PandaState.GRASP:
			self.finger_width = GRASP_WIDTH
			self.movePanda(self.pos)

		elif self.state == PandaState.POST_GRASP:
			self.pos = self.block_position.copy()
			self.gripper_height = alpha_change_gripper_height(MOVE_HEIGHT)
			self.pos[2] = self.gripper_height
			self.movePanda(self.pos)

		elif self.state == PandaState.MOVE_BW:
			self.gripper_height = alpha_change_gripper_height(MOVE_HEIGHT)
			for i in range(2):
				self.pos[i] = ALPHA * self.pos[i] + (1.0-ALPHA)*self.target_position[i]
			# self.pos = self.target_position.copy()
			self.pos[2] = self.gripper_height
			self.movePanda(self.pos)

		elif self.state == PandaState.PRE_RELEASE:
			self.pos = self.target_position.copy()
			self.gripper_height = alpha_change_gripper_height(GRASP_HEIGHT + self.pos[2])
			self.pos[2] = self.gripper_height
			self.fine_correct_release()
			self.movePanda(self.pos)
			return self.pos[2] - GRASP_HEIGHT - DROP_MARGIN

		elif self.state == PandaState.RELEASE:
			self.finger_width = RELEASE_WIDTH
			self.movePanda(self.pos)

		elif self.state == PandaState.POST_RELEASE:
			self.pos = self.target_position.copy()
			self.gripper_height = alpha_change_gripper_height(MOVE_HEIGHT)
			self.pos[2] = self.gripper_height
			self.fine_correct_release()
			self.movePanda(self.pos)

		elif self.state == PandaState.IDLE:
			pass

		return None


