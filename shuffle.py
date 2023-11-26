import json
import random

# Load data from instructions.json
with open('instructions-train.json', 'r') as file:
    data_instructions = json.load(file)

# Load data from scenes.json
with open('scenes-train.json', 'r') as file:
    data_scenes = json.load(file)

# Assuming the arrays you want to shuffle are at index 1 in the dictionaries
array_instructions = data_instructions["instructions"]
array_scenes = data_scenes["scenes"]

# Get the size of the arrays
array_size = len(array_instructions)

print(array_size)
assert(len(array_instructions)==(len(array_scenes)))
# Generate a random permutation
permutation = random.sample(range(array_size), array_size)
print(permutation[0:10])

# Apply the permutation to both arrays

shuffled_array_instructions = [array_instructions[i] for i in permutation]
shuffled_array_scenes = [array_scenes[i] for i in permutation]

# Update the original dictionaries with the shuffled arrays
data_instructions["instructions"] = shuffled_array_instructions
data_scenes["scenes"] = shuffled_array_scenes

# Save the updated data back to instructions.json
with open('instructions-train-new.json', 'w') as file:
    json.dump(data_instructions, file,indent=2)

# Save the updated data back to scenes.json
with open('scenes-train-new.json', 'w') as file:
    json.dump(data_scenes, file,indent=2)

