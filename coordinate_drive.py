import sys
import time  # used to keep track of time
import numpy as np  # array library

import sim
from sensors.position import RobotGPS
# from sensors.proximity import ProximitySensorP3DX

# Pre-Allocation
saving = False
PI = np.pi  # constant

# Coordinates to drive along
coords = {
    1: (-3.9, -0.5),
    2: (-3.9, 3.9),
    3: (3.5, 3.9),
}

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# retrieve motor  handles
_, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
_, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)


gps = RobotGPS(clientID)
gps_start = gps.get_position(actual=True)
print(
    f"Staring Position\n"
    f"{gps_start}\n"
    f"{gps.get_orientation()}"
)
# proximity = ProximitySensorP3DX(clientID)

# start time
t = time.time()

while (time.time() - t) < 1:
    # Driving goes here

    # loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
