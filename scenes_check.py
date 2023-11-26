#!/usr/bin/env python
# coding: utf-8

# In[1]:


import json
import numpy as np
import sys

# In[ ]:


_scene_data = json.load(open('./scenes-train.json'))


# In[ ]:


scene_data = _scene_data["scenes"]


# # T5 data

# In[ ]:


# scene_data[0][0]['objects']


# In[ ]:


# instr_json[0].keys()


# In[ ]:


print("Number of scenes present : ",len(scene_data))
sceneList=[]
for i in range(len(scene_data)):
    sceneList.append(scene_data[i][0]['demonstration_index'])


if(len(sys.argv)>1):
    print("Missing scenes are :")
    for i in range(int(sys.argv[1])):
        if i not in sceneList:
            print(i)

# In[ ]:





# # DataRemove

# In[4]:


