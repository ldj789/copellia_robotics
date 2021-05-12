"""Basic obstacle avoidance with Braitenberg algorithm

"""
import os
import sys
import time  # used to keep track of time
import numpy as np  # array library
import json

import sim
import matplotlib.pyplot as plt  # used for image plotting

from drive.navigate import turn_to_point, check_destination, Navigation
from sensors.odometery import Odometer
from sensors.position import RobotGPS
from sensors.proximity import ProximitySensorP3DX
from sensors.vision import VisionSensorP3DX
from localization.landmarks import RobotLandmarks
from localization.kalman import LandmarkOdometerKf

# Initial Variables
LOOP_DURATION = 35  # in seconds
speed_setting = 1
kp = 1  # steering gain
PI = np.pi  # constant
PLOTTING_FLAG = False
SAVING_DATA = False
SAVE_FILE = os.path.join("data", "lmkf-output.json")

LANDMARK_FRAMES = ["LM_Blue", "LM_Green", "LM_Red", "LM_Yellow"]

# Coordinates to drive along in maze
coords = {
    1: (-3.8, -0.5),
    2: (-3.8, 3.8),
    3: (4.25, 3.8),
    4: (4.25, 1.5),
    5: (1, 1),
    6: (-2, -2.5),
}

sim.simxFinish(-1)  # just in case, close all opened connections
clientId = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientId != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

navigation = Navigation(clientId, speed=speed_setting, steering_gain=kp)

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
lmkf_positions = []
export_data = []

destination_queue = [
    coords[1],
    coords[2],
    coords[3],
    coords[4],
    coords[5],
    coords[6],
]
navigation.set_queue(destination_queue)

# start time
t = time.time()
iter = 0

while (time.time() - t) < LOOP_DURATION:
    iter += 1
    odometer.update_motors()
    gps.update_position()
    proximity.update_distances()
    # if iter % 10 == 0:
    #     lmkf.update()
    lmkf.update()

    random_positions.append(gps.get_position())
    accurate_positions.append(gps.get_position(actual=True))
    odometer_positions.append(odometer.get_position())

    lmkf_positions.append(lmkf.pose)
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
        'odometer_y': odometer.pose[1],
        'kf_x': lmkf.pose[0],
        'kf_y': lmkf.pose[1],
    })

    navigation.update(gps.get_pose())
    time.sleep(0.25)  # loop executes once every 0.1 seconds (= 10 Hz)

# Post Allocation - Stop
navigation.stop()

if SAVING_DATA:
    with open(SAVE_FILE, 'w') as data_out:
        data_out.write(json.dumps(export_data))

if PLOTTING_FLAG:
    rxs = list(map(lambda x: x[0], random_positions))
    rys = list(map(lambda x: x[1], random_positions))
    xs = list(map(lambda x: x[0], accurate_positions))
    ys = list(map(lambda x: x[1], accurate_positions))
    oxs = list(map(lambda x: x[0], odometer_positions))
    oys = list(map(lambda x: x[1], odometer_positions))
    kxs = list(map(lambda x: x[0], lmkf_positions))
    kys = list(map(lambda x: x[1], lmkf_positions))

    plt.plot(xs, ys, color='b')
    plt.plot(oxs, oys, 'r--')
    plt.plot(kxs, kys, color='g')
    plt.show()

