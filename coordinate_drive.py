import sys
import time  # used to keep track of time
import numpy as np  # array library

import sim
from drive.navigate import turn_to_point, check_destination
from sensors.position import RobotGPS
# from sensors.proximity import ProximitySensorP3DX

# Initial Variables
loop_duration = 60  # in seconds
speed_setting = 1.25
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

# proximity = ProximitySensorP3DX(clientID)

destination_queue = [
    coords[2],
    coords[3],
    coords[2],
    coords[1]
]

current_destination = destination_queue.pop(0)
current_pose = gps.get_pose()

print(
    f"Staring Position\n"
    f"pose: {current_pose}\n"
    f"target: {current_destination}\n"
    f"required turn: {turn_to_point(current_pose, current_destination)}"
)

# start time
t = time.time()

while (time.time() - t) < loop_duration:
    gps.update_position()
    current_pose = gps.get_pose()

    current_destination = check_destination(current_pose, current_destination, destination_queue)

    if current_destination is None:
        v, steer = 0, 0
    else:
        v, steer = speed_setting, turn_to_point(current_pose, current_destination) / np.pi

    kp = 2  # steering gain
    vl = v - kp * steer
    vr = v + kp * steer
    # print("V_l =", vl)
    # print("V_r =", vr)

    _ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, vl, sim.simx_opmode_streaming)
    _ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, vr, sim.simx_opmode_streaming)

    # loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
