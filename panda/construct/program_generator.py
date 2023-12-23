import json
import random
import re
import pybullet as p
from copy import deepcopy
import numpy as np
from .scene_graph import SceneGraph
from .program_engine import ProgramExecutor
from panda.settings import *
from panda.objects import Cube, Dice, Tray, Lego



class ProgramGenerator(ProgramExecutor):
    def __init__(self, bullet_client, offset, config, height, width, instance_dir,template_file,metadata_file= './metadata.json'):
        self.template_file = template_file
        self.template = self.load_template(template_file)
        self.metadata = self.load_metadata(metadata_file)
        self.config = config
        super().__init__(bullet_client, offset, config, height, width, instance_dir)
        self.generate_random_scene()
        self.save_position_info()

    def generate_scene_from_template(self):
        pass

    def generate_random_scene(self):
        TABLE_OFFSET = self.table_offset
        TrayPositions = [
                        [-0.5,  0.15, TABLE_OFFSET],
                        [-0.5, -0.15, TABLE_OFFSET],
                        [0.5,  0.15, TABLE_OFFSET],
                        [0.5, -0.15, TABLE_OFFSET],
                    ]
        config = self.config
        flags = self.flags
        self.block_position = None
        self.target_position = None
        self.pos = None
        demo_data = None

        if 'scene_data' in config:
            demo_data = config['scene_data']
        elif 'demo_json_path' in config:
            with open(config['demo_json_path'], 'r') as f:
                demo_data = json.load(f)
        if demo_data is not None:
            
            for objidx, obj in enumerate(demo_data['objects']):
                self.obj_orn = obj['rotation'] # assuming same rotation for all objects
                object_color_idx = obj['color'][0]
                obj_position = obj['position'] if 'position' in obj else demo_data['object_positions'][0][objidx]
                self.object_type.append(obj['type'])
                if obj['type'] == 'Cube':
                    self.objects.append(Cube(self.bullet_client, flags, obj_position, object_color_idx, orn = obj['rotation']))
                elif obj['type'] == 'Dice':
                    # assuming that the Dice can have any random orientation 
                    random_orn = random.choice(CubeOrientations)
                    self.objects.append(Dice(self.bullet_client, flags, obj_position, object_color_idx, orn = random_orn))
                elif obj['type'] == 'Lego':
                    self.objects.append(Lego(self.bullet_client, flags, obj_position, object_color_idx, orn = obj['rotation']))
                elif obj['type'] == 'Tray':
                    self.objects.append(Tray(self.bullet_client, flags, obj_position , object_color_idx))
        else:
            # Restrict Depth of Program
            # self.single_depth = config.get('single_depth', False)
            self.obj_orn = [0,0,0,1] if config['rotation'] == False else config.get('orientation')
            object_counts = config['object_counts']
            # Configuration
            num_cubes = object_counts.get('num_cubes', 0)
            num_trays = object_counts.get('num_trays', 0)
            num_dices = object_counts.get('num_dices', 0)
            num_legos = object_counts.get('num_legos', 0)
            
            # Unique Colors for All Objects. The indices are used to represent the colors
            unique_tray_colors = np.random.permutation(len(ColorList))[: num_trays]
            unique_block_colors = np.random.permutation(len(ColorList))[: num_cubes]
            unique_dice_colors = np.random.permutation(len(ColorList))[: num_dices]
            unique_lego_colors = np.random.permutation(len(ColorList))[: num_legos]

            # Initialize Movable Objects
            self.object_type = ['Cube'] * num_cubes + ['Dice'] * num_dices + ['Lego'] * num_legos
            cube_idx, dice_idx, lego_idx = 0, 0, 0
            random.shuffle(self.object_type)
            
            # this is the number of objects that are supposed to be identical to the first object
            self.numIdenticalToFirst = self.template['numIdenticalToFirst'] if 'numIdenticalToFirst' in self.template else 0
            
            num_movable_objects = num_cubes + num_dices + num_legos + self.numIdenticalToFirst
            # total_objects = num_cubes + num_trays + num_dices + num_legos
            assert num_movable_objects > 0
            
            block_positions = self.get_block_positions(num_movable_objects)

            for idx in range(self.numIdenticalToFirst+1):
                position = block_positions[idx]
                if self.object_type[0] == 'Cube':
                    object_color_idx = int(unique_block_colors[0])
                    self.objects.append(Cube(self.bullet_client, flags,position, object_color_idx,orn = self.obj_orn))
                    
                elif self.object_type[0] == 'Dice':
                    object_color_idx = int(unique_dice_colors[0])
                    self.objects.append(Dice(self.bullet_client, flags, position, object_color_idx,orn = self.obj_orn))

                elif self.object_type[0] == 'Lego':
                    object_color_idx = int(unique_lego_colors[0])
                    self.objects.append(Lego(self.bullet_client, flags, position, object_color_idx,orn = self.obj_orn))

            if self.object_type[0] == 'Cube' : 
                cube_idx+=1
            elif self.object_type[0] == 'Dice' :
                dice_idx+=1
            elif self.object_type[0] == 'Lego':
                lego_idx+=1
 
 
            for idx in range(1,num_movable_objects-self.numIdenticalToFirst):
                position = block_positions[self.numIdenticalToFirst +  idx]
                if self.object_type[idx] == 'Cube':
                    object_color_idx = int(unique_block_colors[cube_idx])
                    self.objects.append(Cube(self.bullet_client, flags,position, object_color_idx,orn = self.obj_orn))
                    cube_idx += 1
                    
                elif self.object_type[idx] == 'Dice':
                    object_color_idx = int(unique_dice_colors[dice_idx])
                    self.objects.append(Dice(self.bullet_client, flags, position, object_color_idx,orn = self.obj_orn))
                    dice_idx += 1

                elif self.object_type[idx] == 'Lego':
                    object_color_idx = int(unique_lego_colors[lego_idx])
                    self.objects.append(Lego(self.bullet_client, flags, position, object_color_idx,orn = self.obj_orn))
                    lego_idx += 1
            

            # Initialize Trays
            tray_positions = np.random.permutation(TrayPositions)
            for idx in range(num_trays):
                position = tray_positions[idx]
                object_color_idx = int(unique_tray_colors[idx])
                self.objects.append(Tray(self.bullet_client, flags, position, object_color_idx))

        
    def load_template(self,template_file):
        with open(template_file,'r') as f:
            all_templates = json.load(f)
        return random.choice(all_templates)
    
    def load_metadata(self, metadata_file):
        with open(metadata_file,'r') as f:
            metadata = json.load(f)
        return metadata
    
    def get_substitute(self,char_token, index):
        # print("subsititue asked for",index,self.unique_objects)
        #Note: The index starts from 1 in the template. So reduce the indices by -1 in the subsequent part.
        if char_token == 'T':
            return self.objects[self.unique_objects[index -1 ]].type
        elif char_token == 'C':
            return self.objects[self.unique_objects[index -1 ]].color[1]
        elif char_token == 'A':
            return self.unique_actions[index-1]
        elif char_token == 'R':
            return self.relations[index -1]
        else:
            raise ValueError("Unknown char token {}".format(char_token))



    def generate_instruction(self,complexity=None):
        '''
        Description: The instructions are read from the template and the <A>, <C>, <O> tokens are replaced with the appropriate concept words.
        Inputs: complexity(str): options = ['simple', 'complex', 'compound']. Please pass this in the config dict. If None, one of the options is randomly choosen.

        Output: program:(list of 3-tuples)
                instruction:(string)
        '''
        complexity = random.choice(['simple','complex','compound']) if complexity is None else complexity
        sent_lexed = random.choice(self.template['text'][complexity])
        words = sent_lexed.split()
        
        for idx,w in enumerate(words):
            match = re.search("<(\w)(\d+)>",w)
            if match:
                char_token = match.group(1)
                if char_token == "T":
                    substitute = self.get_substitute('T',int(match.group(2)))
                    substitute = substitute.lower()
                    if substitute == 'lego':
                        substitute += ' block'
                    words[idx] = substitute 
                elif char_token == 'C':
                    substitute = self.get_substitute('C', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute 
                elif char_token == 'A':
                    substitute = self.get_substitute('A', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute+'_a'
                elif char_token == 'R':
                    substitute = self.get_substitute('R', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute
                else:
                    raise ValueError("Unknown char token {}".format(char_token))
        #At this point, the instruction will be created. But we replace some of the words with thier synonyms from the metadata. 
        sent = ' '.join(words)
        words = sent.split()
        synon = self.metadata['synonyms']
        for idx,w in enumerate(words):
            w = w.lower()
            if w in synon.keys():
                replace_text = random.choice(synon[w])
                words[idx] = replace_text
        
        return self.get_program(), sent_lexed, ' '.join(words), complexity


    def generate_grounded_functional_program(self, object_choice = 'default', MAX_ATEMPTS = 10000):
        '''
         Inputs:  object_choice(string): options= ['default','random']. By default, the first n objects are used. 
        '''
        numIdenticalToFirst = self.template['numIdenticalToFirst'] if 'numIdenticalToFirst' in self.template else 0
        template = self.template
        is_relational = template.get("relational", False)
        num_objects = sum(list(self.config['object_counts'].values()))+numIdenticalToFirst
        try:
            assert num_objects >= template['num_unique_objects']
        except AssertionError:
            print( " The number of  unique objects to be intialized is less than the no of objects in the world")
            return False
        for _ in range(MAX_ATEMPTS):
            #choose actions and objects instances
            possible_actions = self.metadata['actions']['move'] if len(template.get('constraints')["actions"]) == 0 else template.get('constraints')["actions"]
            self.unique_actions = random.sample(possible_actions,template['num_unique_actions']) if "action_preference" not in template.get("constraints") else template.get("constraints")["action_preference"]
            
            # all identical to first are unconstrained as they are just used to build base structure . 
            num_uncons_objs, num_cons_objs = template['num_unique_objects']-len(template['constraints'].get("type",[]))+numIdenticalToFirst, len(template['constraints'].get("type",[]))
            #select unconstrained objects
            self.unique_objects = [i for i in range(num_uncons_objs)] if object_choice == 'default' else random.sample([i for i in range(num_objects) if self.objects[i].type != 'Tray'], num_uncons_objs)
            for constraint in template['constraints'].get("type", []):
                poss_cons_objs = [i for i in range(num_objects) if self.objects[i].type == constraint[1]]
                if len(poss_cons_objs) ==0:
                    return False
                self.unique_objects.insert(constraint[0], random.choice(poss_cons_objs))
            if is_relational:
                all_relations = self.config.get('relations', self.metadata['relations']['spatial_relations'])
                self.relations = [random.choice(all_relations) for i in range(template['num_relations'])]
            self.symbolic_program = deepcopy(self.template['nodes'])
            self.scene_graph = SceneGraph(self.objects,self.position_list[0],self.config)
            self.program = list()
            for node in self.symbolic_program:
                # Replace the tokens by the respective concept words
                for idx,str in enumerate(node['value_inputs']):
                    if(type(str)==int):
                        # we allow ints as an argument for choose nodes. 
                        continue
                    match = re.search("<(\w)(\d+)>", str)
                    if match:
                        node['value_inputs'][idx] = self.get_substitute(match.group(1),int(match.group(2)))
                    else:
                        print("Rasinng unimplmenetd error")
                        raise NotImplementedError

            # Try Executing the program on the scene
            status = self.execute_symbolic_program()
            print("obtained status ",status)
            if status == True:
                # print("##### Found one program ###### ")
                # print("\n...........Requesting PyBullet to perform the Simulation...............")
                self.target_position = self.check_action_compatibility(self.get_program(),self.position_list[-1])
                # assert self.target_position is not None , "There is a bug in program generation"
                if self.target_position is not None:
                    self.apply_program()
                    return True
            else:
                self.program = []

        if len(self.program) == 0:
            print("*******NO compatible program found *******")
            return False
 
    def get_program(self):
        return self.program

    

    def save_demonstration_info(self, command_lexed, command, complexity, program,  configs):
        info = dict()
        info['instruction'] = command
        info['instruction_lexed'] = command_lexed
        scene_info = self.get_scene_info()
        info.update(scene_info)
        info['template_json_filename'] = self.template_file
        info['template_id'] = self.template['template_id']
        info['language_complexity'] = complexity
        info['is_question'] = False
        info['program'] = self.symbolic_program
        # Grounded program must not have save scene actions as they were for saving the initial scene only
        info['grounded_program'] =  [subtask for subtask in program if subtask[0] != "SaveScene"]
        with open(f"{self.instance_dir}/demo.json", 'w') as write_file:
            json.dump(info, write_file)

    def save_question_info(self, question):
        info = dict()
        info['question'] = question
        scene_info = self.get_scene_info()
        info.update(scene_info)
        info['template_filename'] = self.template_file
        info['is_question'] = True
        info['program'] = self.symbolic_program
        with open(f"{self.instance_dir}/question.json", 'w') as write_file:
            json.dump(info, write_file)


 