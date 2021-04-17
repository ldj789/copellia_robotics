#%%
import sim
import sys

#%%
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')
    
#%%
#simx.GetObjects to return an array of object handles
_, scene_objects, _, _, object_names = sim.simxGetObjectGroupData(clientID, sim.sim_object_shape_type, 0, sim.simx_opmode_oneshot_wait)
print(scene_objects)
print(object_names)

#%%
#select only the obstacle object handles
scene_obstacles = scene_objects[object_names.index('80cmHighWall200cm'):]
print(scene_obstacles)

#%%
#loop through object handles to get each objects position
obstacle_positions = []

for obstacle in scene_obstacles:
    _, position = sim.simxGetObjectPosition(clientID, obstacle, -1, sim.simx_opmode_oneshot_wait)
    obstacle_positions.append(position)

print(obstacle_positions)

#TODO: draw 2d grid of map
# %%
