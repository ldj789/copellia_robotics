"""Basic obstacle avoidance with Braitenberg algorithm

"""
import sys
import time  # used to keep track of time
import numpy as np  # array library

import sim

from drive.navigate import turn_to_point, check_destination
from sensors.odometery import Odometer
from sensors.position import RobotGPS
from sensors.proximity import ProximitySensorP3DX
from sensors.vision import VisionSensorP3DX
from localization.landmarks import RobotLandmarks
from localization.kalman import LandmarkOdometerKf
import matplotlib.pyplot as plt  # used for image plotting
import json

# Initial Variables
LOOP_DURATION = 3  # in seconds
speed_setting = 0.75
kp = 2  # steering gain
PI = np.pi  # constant
PLOTTING_FLAG = False
SAVING_DATA = False

LANDMARK_FRAMES = ["LM_Blue", "LM_Green", "LM_Red", "LM_Yellow"]

# Coordinates to drive along in maze
coords = {
    1: (-3.8, -0.5),
    2: (-3.8, 3.8),
    3: (3.5, 3.8),
}


sim.simxFinish(-1)  # just in case, close all opened connections
clientId = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientId != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# retrieve motor  handles
_, left_motor_handle = sim.simxGetObjectHandle(clientId, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
_, right_motor_handle = sim.simxGetObjectHandle(clientId, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

gps = RobotGPS(clientId)
landmarks = RobotLandmarks(clientId, gps, landmark_frames=LANDMARK_FRAMES)
gps_start = gps.get_position(actual=True)

odometer = Odometer(clientId, 0.098, pose=[gps_start[0], gps_start[1], gps.get_orientation()])
proximity = ProximitySensorP3DX(clientId)
vision = VisionSensorP3DX(clientId)
lmkf = LandmarkOdometerKf(odometer, landmarks, init_pose=gps.get_pose(actual=True))

plotting_flag = False
random_positions = []
accurate_positions = []
odometer_positions = []
export_data = []

destination_queue = [
    coords[2],
    coords[3],
    coords[2],
    coords[1]
]
current_destination = destination_queue.pop(0)

# start time
t = time.time()

while (time.time() - t) < LOOP_DURATION:
    odometer.update_motors()
    gps.update_position()
    proximity.update_distances()
    lmkf.update()

    random_positions.append(gps.get_position())
    accurate_positions.append(gps.get_position(actual=True))
    odometer_positions.append(odometer.get_position())
    # print(f"{odometer} {gps}")

    min_dist, min_sensor_angle, min_ind = proximity.braitenberg_min()
    # print(f"{min_dist} {min_ind}")

    print(
        f"Landmark Ranges\n"
        f"lmkf pose: {lmkf.pose}\n"
        f"odometer pose: {odometer.pose}\n"
        # verify relative distances between odometer and kf across run
        # f"{(lmkf.pose[0] - odometer.pose[0], lmkf.pose[1] - odometer.pose[1])}\n"
        # f"lm ranges: {landmarks.landmark_ranges()}\n"
    )

    export_data.append({
        'gps_x': gps.get_position()[0],
        'gps_y': gps.get_position()[1],
        'odometer_x': odometer.pose[0],
        'odometer_y': odometer.pose[1]
    })

    current_pose = gps.get_pose()
    current_destination = check_destination(current_pose, current_destination, destination_queue, d=0.5)

    if current_destination is None:
        v, steer = 0, 0
    else:
        v, steer = speed_setting, turn_to_point(current_pose, current_destination) / np.pi

    vl = v - kp * steer
    vr = v + kp * steer
    # print("V_l =", vl)
    # print("V_r =", vr)

    _ = sim.simxSetJointTargetVelocity(clientId, left_motor_handle, vl, sim.simx_opmode_streaming)
    _ = sim.simxSetJointTargetVelocity(clientId, right_motor_handle, vr, sim.simx_opmode_streaming)

    time.sleep(0.1)  # loop executes once every 0.1 seconds (= 10 Hz)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientId, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientId, right_motor_handle, 0, sim.simx_opmode_streaming)
# print(vision.raw_image())

if SAVING_DATA:
    with open('output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))

if PLOTTING_FLAG:
    rxs = list(map(lambda x: x[0], random_positions))
    rys = list(map(lambda x: x[1], random_positions))
    xs = list(map(lambda x: x[0], accurate_positions))
    ys = list(map(lambda x: x[1], accurate_positions))
    oxs = list(map(lambda x: x[0], odometer_positions))
    oys = list(map(lambda x: x[1], odometer_positions))

    print(oxs, oys)

    # plt.plot(rxs, rys, color='r')
    plt.plot(xs, ys, color='b')
    plt.plot(oxs, oys, color='g')
    plt.show()
