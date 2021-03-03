#%%
import sys
import time  # used to keep track of time
import numpy as np  # array library
import cv2
import sim

#%%
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    print('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_vision', sim.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)
    while (clientID != -1):
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)
        if err == sim.simx_return_ok:
            print('Image ok!!!')
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == sim.simx_return_novalue_flag:
            print('No Image Yet')
            pass
        else:
            print(err)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

cv2.destroyAllWindows()
