import sys
import time  # used to keep track of time
import numpy as np  # array library
import json

import sim
from drive.navigate import turn_to_point, check_destination
from sensors.position import RobotGPS
from sensors.odometery import Odometer
# from sensors.proximity import ProximitySensorP3DX

# Initial Variables
loop_duration = 60  # in seconds
speed_setting = 1.25
PI = np.pi  # constant
saving_data = True

export_data = []

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
odometer = Odometer(clientID, 0.098, pose=[gps_start[0], gps_start[1], gps.get_orientation()[2]])
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
    odometer.update_motors()
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
    
    export_data.append({
        'actual_x': gps.get_position(actual=True)[0],
        'actual_y': gps.get_position(actual=True)[1],
        'odometer_x': odometer.pose[0],
        'odometer_y': odometer.pose[1],
        'gps_x': gps.get_position()[0],
        'gps_y': gps.get_position()[1],
        
    })
    # loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)

# save data
if saving_data:
    with open('output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))
