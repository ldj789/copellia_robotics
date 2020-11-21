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
import matplotlib.pyplot as plt  # used for image plotting
import json

# Initial Variables
loop_duration = 15  # in seconds
plotting_flag = False
saving_data = False

# Pre-Allocation
PI = np.pi  # constant

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
print(gps.get_orientation())
odometer = Odometer(clientID, 0.098, pose=[gps_start[0], gps_start[1], gps.get_orientation()[2]])
proximity = ProximitySensorP3DX(clientID)
vision = VisionSensorP3DX(clientID)


random_positions = []
accurate_positions = []
odometer_positions = []
export_data = []

# start time
t = time.time()

while (time.time() - t) < loop_duration:
    odometer.update_motors()
    gps.update_position()
    proximity.update_distances()

    random_positions.append(gps.get_position())
    accurate_positions.append(gps.get_position(actual=True))
    odometer_positions.append(odometer.get_position())
    # print(f"{odometer} {gps}")

    min_dist, min_sensor_angle, min_ind = proximity.braitenberg_min()
    print(f"{min_dist} {min_ind}")
    export_data.append({
        'gps_x': gps.get_position()[0],
        'gps_y': gps.get_position()[1],
        'odometer_x': odometer.pose[0],
        'odometer_y': odometer.pose[1]
    })
    
    # Braitenberg steering
    if min_dist < 0.5:
        steer = 1 / min_sensor_angle
    else:
        steer = 0

    # gain_map = {
    #     0: .25,
    #     1: .50,
    #     2: .75
    # }
    #
    # kp = gain_map.get(int(time.time() - t) // 60)

    v = 1  # forward velocity
    kp = 0.6  # steering gain
    vl = v - kp * steer
    vr = v + kp * steer
    # print("V_l =", vl)
    # print("V_r =", vr)

    _ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, vl, sim.simx_opmode_streaming)
    _ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, vr, sim.simx_opmode_streaming)

    time.sleep(0.2)  # loop executes once every 0.2 seconds (= 5 Hz)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
print(vision.raw_image())

if saving_data:
    with open('output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))

if plotting_flag:
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
