#Ackowledgement: few parts of the code is taken from clevr-data generation code

from copy import deepcopy
from re import S
from .base import ConstructBase
import random 
def scene_handler(scene_graph, inputs, value_inputs):
  # Just return all objects in the scene
  return list(range(len(scene_graph.objects)))


def make_filter_handler(attribute):
  def filter_handler(scene_graph, inputs, value_inputs):
    assert len(inputs) == 1
    assert len(value_inputs) == 1
    value = value_inputs[0]
    output = []
    for idx in inputs[0]:
      atr = scene_graph.objects[idx][attribute]
      if value in atr:
        output.append(idx)
    return output
  return filter_handler


def unique_handler(scene_graph, inputs, value_inputs):
  assert len(inputs) == 1
  if len(inputs[0]) != 1:
    return '__INVALID__'
  return inputs[0][0]

def choose_index_handler(scene_graph, inputs, value_inputs):
  assert len(inputs) == 1
  if len(value_inputs) != 1:
    return '__INVALID__'
  return inputs[0][value_inputs[0]]


def relate_handler(scene_graph, inputs, value_inputs):
  assert len(inputs) == 1
  assert len(value_inputs) == 1
  relation = value_inputs[0]
  if len(inputs[0]) != 1:
        return '__INVALID__'
  return scene_graph.relationships[relation][inputs[0][0]]
    

def union_handler(scene_graph, inputs, value_inputs):
  assert len(inputs) == 2
  assert len(value_inputs) == 0
  return sorted(list(set(inputs[0]) | set(inputs[1])))


def intersect_handler(scene_graph, inputs, value_inputs):
  assert len(inputs) == 2
  assert len(value_inputs) == 0
  return sorted(list(set(inputs[0]) & set(inputs[1])))

def complement_handler(scene_graph, inputs, value_inputs):
      assert len(inputs) == 1, "expected len 1 in complement handler, got {}".format(len(inputs))
      assert len(value_inputs) == 0
      output = []
      for objs in scene_handler(scene_graph, inputs, value_inputs):
          if objs not in inputs[0]:
              output.append(objs)
      return output

execute_handlers = {
  'scene': scene_handler,
  'filter_color': make_filter_handler('color'),
  'filter_type': make_filter_handler('type'),
  'unique': unique_handler,
  'relate': relate_handler,
  'union': union_handler,
  'intersect': intersect_handler,
  'complement': complement_handler,
  'choose_index' : choose_index_handler
}

class ProgramExecutor(ConstructBase):
    def _init__(self,bullet_client, offset, config, height, width, instance_dir):
        super().__init__(bullet_client, offset, config, height, width, instance_dir)

# only updated the cur_subtask for "move" nodes, not for forall and and program nodes. 
    def execute_symbolic_program(self):
      """
      Use scene graph information to answer a structured question. Most of the
      heavy lifting is done by the execute handlers defined above.

      Note: The current implementation of the executor is not standalone. The ProgramGenerator class has to be intialized and executor can be called
              from the ProgramGenerator only. Rewrite this to make it standalone in the future.
      """
      assert hasattr(self,'symbolic_program')
      assert hasattr(self,'scene_graph')
      assert hasattr(self,'metadata')
      node_outputs = []
      # find outputs at each nodes using SceneGraph. When an action is encountered, the action will be simulated and the SceneGraph will be regenerated.
      pos_list = deepcopy(self.position_list)
      for node in self.symbolic_program:
          node_type = node['type'] 
          if node_type in self.metadata['actions']:
              node['output'] = None   
              if node_type == 'move':          
                action, m_object, b_object = node['value_inputs'] + [node_outputs[idx] for idx in node['inputs'][0:2]]
                distance = node['distance'] if 'distance' in node else 1
                show = node['visible'] if 'visible' in node else True
                cur_subtask = (action, m_object, b_object,distance,show)
                target_position = self.check_action_compatibility([cur_subtask], pos_list[-1])
                if target_position is None:
                    print("Returning false due to non compatible action")
                    return False
                else:
                    next_obj_positions = deepcopy(pos_list[-1])
                    next_obj_positions[m_object] = target_position[0]
                    pos_list.append(next_obj_positions)

                self.program.append(cur_subtask)
                self.scene_graph.update(pos_list[-1])
                node_outputs.append([])
              
              elif node_type == 'program':
                action, m_object, b_object = node['value_inputs'] + [node_outputs[idx] for idx in node['inputs'][0:2]]
                cur_subtask = (action, m_object, b_object)
                target_position = self.check_action_compatibility([cur_subtask], pos_list[-1])
                if target_position is None:
                    return False
                else:
                    next_obj_positions = deepcopy(pos_list[-1])
                    next_obj_positions[m_object] = target_position[0]
                    pos_list.append(next_obj_positions)

                self.program.append(cur_subtask)
                self.scene_graph.update(pos_list[-1])
                node_outputs.append([])
                
              elif node_type == "forall":
                action,m_object_set,b_object =  (node['value_inputs'] + [node_outputs[idx] if idx != 'None' else None for idx in node['inputs'][0:2] ])
                for m in m_object_set:
                    if self.objects[m].type == 'Tray':
                        breakpoint()
                if b_object is None:
                  if len(m_object_set) > 1:
                    b_object = m_object_set[0]
                  else:
                    return False 
                  
                if b_object in m_object_set:
                  m_object_set.remove(b_object)
                
                random.shuffle(m_object_set)
                for m_obj in m_object_set:
                  cur_subtask = (action,m_obj,b_object)
                  target_position = self.check_action_compatibility([cur_subtask],pos_list[-1])
                  if target_position is None:
                    return False
                  else:
                    next_obj_positions = deepcopy(pos_list[-1])
                    next_obj_positions[m_obj] = target_position[0]
                    pos_list.append(next_obj_positions)

                  self.program.append(cur_subtask)
                  self.scene_graph.update(pos_list[-1])

                  if action != "INSIDE":
                    b_object = m_obj #using commutativity of the action to make data. not present in the model bias 
                   
                node_outputs.append([])

              elif node_type == 'idle':
                node_outputs.append([])
              else:
                raise NotImplementedError
          
          else :
              msg = 'Could not find handler for "%s"' % node_type
              assert node_type in execute_handlers, msg
              handler = execute_handlers[node_type]
              node_inputs = [node_outputs[idx] for idx in node['inputs']]
              value_inputs = node.get('value_inputs', [])
              node_output = handler(self.scene_graph, node_inputs, value_inputs)
              node['output'] = node_output
              node_outputs.append(node_output)
              if node_output == '__INVALID__':
                return False
      return True
  
        
