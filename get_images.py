#%%
import sim

from sensors.vision import VisionSensorP3DX

#%%
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    print('Vision Sensor object handling')
    print('Getting images')
    vision_sensor = VisionSensorP3DX(clientID)
    vision_sensor.save_images('python_vrep_maze_1')
else:
    print('Connection not successful')
    sys.exit('Could not connect')