"""Basic obstacle avoidance with Braitenberg algorithm

"""
import sys
import time  # used to keep track of time
import numpy as np  # array library

import sim

from sensors.odometery import Odometer
from sensors.position import RobotGPS
from sensors.proximity import ProximitySensorP3DX
from sensors.vision import VisionSensorP3DX
from localization.landmarks import RobotLandmarks
from localization.kalman import LandmarkOdometerKf
import matplotlib.pyplot as plt  # used for image plotting
import json

# Initial Variables
LOOP_DURATION = 15  # in seconds
PLOTTING_FLAG = False
SAVING_DATA = False

LANDMARK_FRAMES = ["LM_Blue", "LM_Green", "LM_Red", "LM_Yellow"]

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

landmarks = RobotLandmarks(clientId, landmark_frames=LANDMARK_FRAMES)
gps = RobotGPS(clientId)
gps_start = gps.get_position(actual=True)

print(gps.get_orientation())
odometer = Odometer(clientId, 0.098, pose=[gps_start[0], gps_start[1], gps.get_orientation()[2]])
proximity = ProximitySensorP3DX(clientId)
vision = VisionSensorP3DX(clientId)
lmkf = LandmarkOdometerKf(odometer, landmarks)

plotting_flag = False
random_positions = []
accurate_positions = []
odometer_positions = []
export_data = []

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
        f"{landmarks.landmark_ranges()}\n"
        f"{lmkf.pose}"
    )

    export_data.append({
        'gps_x': gps.get_position()[0],
        'gps_y': gps.get_position()[1],
        'odometer_x': odometer.pose[0],
        'odometer_y': odometer.pose[1]
    })

    time.sleep(0.2)  # loop executes once every 0.2 seconds (= 5 Hz)

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
