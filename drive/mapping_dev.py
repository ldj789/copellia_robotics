import sys
import time  # used to keep track of time
import numpy as np  # array library
import json
from pprint import pprint

import sim
import matplotlib.pyplot as plt
from drive.navigate import turn_to_point, check_destination, Navigation
from sensors.proximity import ProximitySensorP3DX
from sensors.position import RobotGPS
# from sensors.proximity import ProximitySensorP3DX

# Initial Variables
loop_duration = 15  # in seconds
speed_setting = 0.5
steering_gain = 1.5
# loop_duration = 0  # in seconds
# speed_setting = 0
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

gps = RobotGPS(clientId)
proximity = ProximitySensorP3DX(clientId)
gps_start = gps.get_position(actual=True)
navigation = Navigation(clientId, speed=speed_setting, steering_gain=steering_gain)

destination_queue = [
    coords[2],
    coords[3],
    coords[2],
    coords[1]
]
navigation.set_queue(destination_queue)

print(
    f"Staring Position\n"
    f"pose: {gps.get_pose(actual=True)}\n"
    f"target: {destination_queue[0]}\n"
    f"required turn: {turn_to_point(gps.get_pose(actual=True), destination_queue[0])}"
)
time.sleep(0.25)

# start time
t = time.time()
obstacle_positions = []
robot_positions = []
iteration_count = 0

while (time.time() - t) < loop_duration:
    iteration_count += 1
    # Robot and Sensor Updates
    gps.update_position()
    proximity.update_distances()
    current_pose = gps.get_pose(actual=True)
    navigation.update(current_pose)
    v, turning = navigation.report()

    print(f"speed: {v}, turning: {turning}")

    # Mapping
    if np.abs(turning) < 0.033:
        obstacle_distances = proximity.get_distances()
        # print(obstacle_distances)
        # print(iteration_count)
        proximate_objects = proximity.get_proximate_objects(current_pose=current_pose)
        for i, obj in enumerate(proximate_objects):
            proximate_objects[i] = obj + (iteration_count,)
            # print(proximate_objects[i])

        # Tracking of elements
        obstacle_positions.extend(proximate_objects)

    robot_positions.append(current_pose)

    # Short Sleep loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
navigation.stop()

# save data
if saving_data:
    with open('output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))

if plotting_flag:
    rxs = list(map(lambda x: x[0], obstacle_positions))
    rys = list(map(lambda x: x[1], obstacle_positions))
    iterations = list(map(lambda x: x[2], obstacle_positions))
    # sensors = list(map(lambda x: x[2], obstacle_positions))
    xs = list(map(lambda x: x[0], robot_positions))
    ys = list(map(lambda x: x[1], robot_positions))
    # oxs = list(map(lambda x: x[0], odometer_positions))
    # oys = list(map(lambda x: x[1], odometer_positions))

    plt.scatter(rxs, rys, color='r')
    plt.plot(xs, ys, color='b')
    # plt.text(rxs, rys, iterations)
    # [plt.annotate(iterations[i], (rxs[i] + .2, rys[i] + .2)) for i in range(len(iterations))]
    # plt.plot(oxs, oys, color='g')
    # for i, sensor in enumerate(obstacle_positions):
    #     plt.annotate(sensor, (rxs[i], rys[i]))
    plt.xlim([-6, 6])
    plt.ylim([-6, 6])
    plt.show()
