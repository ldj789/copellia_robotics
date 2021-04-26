import sys
import time  # used to keep track of time
import numpy as np  # array library
import json

import sim
import matplotlib.pyplot as plt
from drive.navigate import turn_to_point, check_destination
from sensors.proximity import ProximitySensorP3DX
from sensors.position import RobotGPS
from sensors.odometery import Odometer
from localization.kalman import GpsOdometerKf

# from sensors.proximity import ProximitySensorP3DX

# Initial Variables
loop_duration = 25  # in seconds
speed_setting = 1.25
PI = np.pi  # constant
saving_data = False
plotting_flag = True

export_data = []

# Coordinates to drive along
coords = {
    1: (-3.9, -0.5),
    2: (-3.9, 3.9),
    3: (3.5, 3.9),
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
proximity = ProximitySensorP3DX(clientId)
gps_start = gps.get_position(actual=True)
odometer = Odometer(clientId, 0.097, pose=[gps_start[0], gps_start[1], gps.get_orientation()[2]])

# proximity = ProximitySensorP3DX(clientID)

destination_queue = [
    coords[2],
    coords[3],
    coords[2],
    coords[1]
]

current_destination = destination_queue.pop(0)
current_pose = gps.get_pose()
kf = GpsOdometerKf(gps, odometer, pose=current_pose)

print(
    f"Staring Position\n"
    f"pose: {current_pose}\n"
    f"target: {current_destination}\n"
    f"required turn: {turn_to_point(current_pose, current_destination)}"
)

# start time
t = time.time()

obstacle_positions = []
robot_positions = []

while (time.time() - t) < loop_duration:
    gps.update_position()
    odometer.update_motors()
    kf.update()
    proximity.update_distances()
    current_pose = gps.get_pose()

    obstacle_distances = proximity.get_distances()
    print(obstacle_distances)

    # write obstacle position
    # robot_x + distance * np.cos(obs_theta + robot_theta),
    # robot_y + distance * np.sin(obs_theta + robot_theta)
    for obstacle in obstacle_distances:
        if obstacle[0] < 10:
            obstacle_position = (
                current_pose[0] + obstacle[0] * np.sin(obstacle[1] + current_pose[2]),
                current_pose[1] + obstacle[0] * np.cos(obstacle[1] + current_pose[2])
            )
            obstacle_positions.append(obstacle_position)
            print(obstacle_position)
    robot_positions.append(current_pose)
    print(current_pose)

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

    _ = sim.simxSetJointTargetVelocity(clientId, left_motor_handle, vl, sim.simx_opmode_streaming)
    _ = sim.simxSetJointTargetVelocity(clientId, right_motor_handle, vr, sim.simx_opmode_streaming)
    
    # loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
_ = sim.simxSetJointTargetVelocity(clientId, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientId, right_motor_handle, 0, sim.simx_opmode_streaming)

# save data
if saving_data:
    with open('output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))

if plotting_flag:
    rxs = list(map(lambda x: x[0], obstacle_positions))
    rys = list(map(lambda x: x[1], obstacle_positions))
    xs = list(map(lambda x: x[0], robot_positions))
    ys = list(map(lambda x: x[1], robot_positions))
    # oxs = list(map(lambda x: x[0], odometer_positions))
    # oys = list(map(lambda x: x[1], odometer_positions))

    plt.plot(rxs, rys, color='r')
    plt.plot(xs, ys, color='b')
    # plt.plot(oxs, oys, color='g')
    plt.show()