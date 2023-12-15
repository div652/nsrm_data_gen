from enum import Enum

#4-vec specification of the colour (R,G,B,A): R = Red, G = Green, B = Blue, A = Alpha (0 = invisible, 1 = visible)
ColorList = [
	(0, 0, 1, 1), #BLUE
	(0, 1, 0, 1), #GREEN
	(1, 0, 0, 1), #RED
	(0, 1, 1, 1), #CYAN
	(1, 1, 0, 1), #YELLOW
	(1, 0, 1, 1), #MAGENTA
	(1, 1, 1, 1), #WHITE
]
CubeOrientations = [
	#(1, 0, 0, 0),
    #(0, 1, 0, 0),
   #(0, 0, 1, 0),
    (0, 0, 0, 1), # ONLY THIS ORIENTATION WAS USED IN ALL THE NSRM TRAINING . 
  # (0.5, 0.5, 0.5, 0.5),
  #  (0.5, -0.5, -0.5, 0.5),
  #  (0.5, -0.5, 0.5, -0.5),
  #  (0.5, 0.5, -0.5, -0.5),
  #  (0.5, 0.5, -0.5, 0.5),
  #  (0.5, -0.5, 0.5, 0.5),
  #  (0.5, 0.5, 0.5, -0.5),
  #  (0.5, -0.5, -0.5, -0.5),
  #  (0.5, 0.5, 0.5, 0.5),
  #  (0.5, 0.5, -0.5, -0.5),
  #  (0.5, -0.5, -0.5, 0.5),
  #  (0.5, -0.5, 0.5, -0.5),
  #  (0.5, -0.5, -0.5, -0.5),
  #  (-0.5, 0.5, 0.5, -0.5),
  #  (-0.5, -0.5, -0.5, -0.5),
  #  (-0.5, 0.5, -0.5, 0.5),
  #  (-0.5, -0.5, 0.5, 0.5),
  #  (-0.5, 0.5, 0.5, 0.5),
  #  (-0.5, -0.5, 0.5, -0.5),
  #  (-0.5, 0.5, -0.5, -0.5)
]
class Color(Enum):
	Blue = 0
	Green = 1
	Red = 2
	Cyan = 3
	Yellow = 4
	Magenta = 5
	White = 6

class Objects(Enum):
	Block = "cube_small.urdf"
	Tray = "tray/tray.urdf"
	Lego = "lego/lego.urdf"
	BlueDice = "./urdf/dice/blue/Dice.urdf"
	GreenDice = "./urdf/dice/green/Dice.urdf"
	RedDice = "./urdf/dice/red/Dice.urdf"
	CyanDice = "./urdf/dice/cyan/Dice.urdf"
	YellowDice = "./urdf/dice/yellow/Dice.urdf"
	MagentaDice = "./urdf/dice/magenta/Dice.urdf"
	WhiteDice = "./urdf/dice/white/Dice.urdf"

DiceUrdf =  [
	Objects.BlueDice,
	Objects.GreenDice,
	Objects.RedDice,
	Objects.CyanDice,
	Objects.YellowDice,
	Objects.MagentaDice,
	Objects.WhiteDice,
]
#The amount of area that we use to place the objects
WORK_AREA_SIDE = 0.5
WORK_AREA_LENGTH = 0.80
WORK_AREA_BREADTH = 1.3

# These are values used in table.urdf 
# length is the dimension of table along y axis. 
TABLE_LENGTH = 1 
# breadth is the dimenstion of the table along x-axis.
TABLE_BREADTH = 1.5
PANDA_POSITION = [0, 0.5, 0.5]

# The thickness of the table edge is 0.05 , and the hieght of the bottom of table edge is 0.60
# hence the height of the table top is 0.60+0.05 = 0.65 , this is set as the table_offset
TABLE_OFFSET = 0.65

MARGIN = 0.03
TrayPositions = [
	[-0.5,  0.15, TABLE_OFFSET], # LEFT
	[0.5,  0.15, TABLE_OFFSET],  # RIGHT
]

# the size of the objects. Currently the size of cube, dice and lego objects loaded from urdf files are <0.05,0.05,0.05>. 
# If you want to increase the size of the object, please change the values here. The default will be 0.05
object_dimensions = {
	'Cube': 0.0500,
	'Dice': 0.0500,
	'Lego': 0.0625,
}

directions = {
    'above': [0., 0., 1.],
    'behind': [0., 1., 0.],
    'below': [0., 0., -1.],
    'front': [0., -1., 0.],
    'left': [-1., 0., 0.],
    'right': [1., 0., 0.] 
}

camera_settings = {
	'small_table_view' : {
		'cameraDistance': 12.4,
		'cameraYaw': 0,
		'cameraPitch': -47,
		'cameraTargetPosition':[0, 8.0, -8.0],
	},
	'small_table_view_2' : {
		'cameraDistance': 12.3,
		'cameraYaw': 0,
		'cameraPitch': -47,
		'cameraTargetPosition':[0, 8.0, -8.0],
	},
	'small_table_view_3' : {
		'cameraDistance': 1.20,
		'cameraYaw': 0,
		'cameraPitch': -47,
		'cameraTargetPosition':[0, 0, 0.64],
	},
	'big_table_view' : {
		'cameraDistance': 12.9,
		'cameraYaw': 0,
		'cameraPitch': -60,
		'cameraTargetPosition': [0, 6.0, -10.0],
	},
	'big_table_view_2' : {
		'cameraDistance': 12.3,
		'cameraYaw': 0,
		'cameraPitch': -46,
		'cameraTargetPosition':[0, 8.0, -7.5],
	},
	'big_table_view_3' : {
		'cameraDistance': 13.0,
		'cameraYaw': 0,
		'cameraPitch': -49,
		'cameraTargetPosition':[0, 8.0, -8.2],
	},
	'top_view' : {
		'cameraDistance': 0.7,
		'cameraYaw': 0,
		'cameraPitch': -85,
		'cameraTargetPosition':[0, 0, 0.64],
	},
	'right_view' : {
		'cameraDistance': 0.8,
		'cameraYaw': 90,
		'cameraPitch': -5,
		'cameraTargetPosition':[0, 0, 0.64],
	},
	'robot_view' : {
		'cameraDistance': 1.,
		'cameraYaw': 180,
		'cameraPitch': -60,
		'cameraTargetPosition':[0, 0, 0.64],
	},
}

## Camera Views is also a dictionary like CameraSeettings, made by Div, can be integrated later on 
CameraViews=  {
		# 'top' : {
		# 		'cameraDistance': 0.6,
		# 		'cameraYaw': 0,
		# 		'cameraPitch': -85,
		# 		'cameraTargetPosition':[0, 0, 0.64],
		# },
    
    # 'bottom' : {
	# 		'cameraDistance': 1.2,
	# 		'cameraYaw': 0,
	# 		'cameraPitch': 90,
	# 		'cameraTargetPosition':[0, 8.0, -7.5],
	# },
    
    'front_0' : {
			'cameraDistance': 0.56,
			'cameraYaw': 0,
			'cameraPitch': -5,
			'cameraTargetPosition':[0, 0, 0.69],
	},
    # this is the original one that has been used trhoughotut sem7
    'diag1_45' : {
			'cameraDistance': 0.565,
			'cameraYaw': 48,
			'cameraPitch': -43,
			'cameraTargetPosition':[0.12, -0.12, 0.66],
	},

	# 'diag1_45' : {
	# 		'cameraDistance': 0.580,
	# 		'cameraYaw': 41,
	# 		'cameraPitch': -46,
	# 		'cameraTargetPosition':[0.14, -0.02, 0.66],
	# },

    
	'front_45' : {
			'cameraDistance': 0.630,
			'cameraYaw': 0,
			'cameraPitch': -43,
			'cameraTargetPosition':[-0.05, 0, 0.66],
	},
    
    
    
    
    # 'right_0' : {
	# 		'cameraDistance': 0.50,
	# 		'cameraYaw': -90,
	# 		'cameraPitch': 0,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
    
    
	# 'right_45' : {
	# 		'cameraDistance': 0.45,
	# 		'cameraYaw': -90,
	# 		'cameraPitch': -40,
	# 		'cameraTargetPosition':[-0.04, 0, 0.64],
	# },
    # 'diag2_45' : {
	# 		'cameraDistance': 0.50,
	# 		'cameraYaw': -45,
	# 		'cameraPitch': -47,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
    
    # 'back_0' : {
	# 		'cameraDistance': 0.52,
	# 		'cameraYaw': -180,
	# 		'cameraPitch': 0,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
    
	# 'back_45' : {
	# 		'cameraDistance': 0.58,
	# 		'cameraYaw': -180,
	# 		'cameraPitch': -40,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
					
    # 'diag3_45' : {
	# 		'cameraDistance': 0.50,
	# 		'cameraYaw': -135,
	# 		'cameraPitch': -47,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
    
    # 'left_0' : {
	# 		'cameraDistance': 0.66,
	# 		'cameraYaw': 90,
	# 		'cameraPitch': 0,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# },
    

	# 'left_45' : {
	# 		'cameraDistance': 0.45,
	# 		'cameraYaw': 90,
	# 		'cameraPitch': -40,
	# 		'cameraTargetPosition':[0.1, 0, 0.64],
	# },
    
    # 'diag4_45' : {
	# 		'cameraDistance': 0.50,
	# 		'cameraYaw': 135,
	# 		'cameraPitch': -47,
	# 		'cameraTargetPosition':[0, 0, 0.64],
	# }
         }

