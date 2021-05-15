#%%
import sys
import time  # used to keep track of time
import numpy as np  # array library
import re
import json
import matplotlib.pyplot as plt

import sim
from drive.navigate import turn_to_point, check_destination, Navigation
from drive.astar import AStarPlanner
from sensors.position import RobotGPS
from sensors.odometery import Odometer
from localization.kalman import GpsOdometerKf


#%%
def get_wall_endpoints(x, theta, l):
    """Get endpoints for wall segment
    Add half of length to center point in direction of wall (Euler gamma)
    substract half of length like above
    TODO: create grid
    
    :param x: center point  as (x,y,z)
    :param theta: object euler gamma for rotation
    :param length: length of wall segment in cm
    :return: (endpoint, endpoint) as ((x, y), (x, y))
    """

    # theta for wall segments is perpendicluar to the wall, first it is rotated 90
    theta += np.pi/2
    start_point = (x[0] + (l/2/100) * np.cos(theta), x[1] + (l/2/100) * np.sin(theta))
    end_point = (x[0] - (l/2/100) * np.cos(theta), x[1] - (l/2/100) * np.sin(theta))
    return start_point, end_point


def get_wall_mesh_points(start, end, marker_size=0.10):
    """
    
    :param start: x y of wall start
    :param end: x y of wall end
    :return: 
    """
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    l = np.sqrt(dx**2 + dy**2)
    n = int(np.floor_divide(l, marker_size))
    if n == 0:
        return []
    return [(start[0] + (0.5 + i) * dx/n , start[1] + (0.5 + i) * dy/n) for i in range(n)]


#%%
# Connect to simulator
sim.simxFinish(-1)  # just in case, close all opened connections
clientId = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientId != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

#%%
# Initial Variables
speed_setting = 1.25
steering_gain = 2
PI = np.pi  # constant
saving_data = False
robot_radius = .38
grid_size = .25
gx = -4.0
gy = -4.0
navigation = Navigation(client_id=clientId, speed=speed_setting, steering_gain=steering_gain)
export_data = []

#%%
# Retrieve map obstacles
_, scene_indices, _, _, object_names = sim.simxGetObjectGroupData(clientId, sim.sim_object_shape_type, 0, sim.simx_opmode_oneshot_wait)
scene_objects = list(zip(object_names, scene_indices))
# Retrieve wall objects
wall_objects = list(filter(lambda x: x[0].startswith("240cmHighWall"), scene_objects))
rack_objects = list(filter(lambda x: x[0].startswith(('rack', 'X', 'x', 'Corner')), scene_objects))
rack_objects.extend(list(filter(lambda x: x[0].startswith("Rack_"), scene_objects)))
door_objects = list(filter(lambda x: x[0].startswith('door'), scene_objects))
sliding_door_objects = list(filter(lambda x: x[0].startswith('sliding'), scene_objects))
table = list(filter(lambda x: x[0].startswith('customizable'), scene_objects))[0]
chair = list(filter(lambda x: x[0].startswith('conference'), scene_objects))[0]

#%%
# Retrieve center point, length, and orientation of wall segment
segment_properties = []
for segment in wall_objects:
    length_match = re.match("240cmHighWall(\d+)cm", segment[0])
    if length_match:
        segment_length = int(length_match.group(1))
        _, center_point = sim.simxGetObjectPosition(clientId, segment[1], -1, sim.simx_opmode_oneshot_wait)
        _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientId, segment[1], -1, sim.simx_opmode_oneshot_wait)
        segment_properties.append((center_point, euler_gamma, segment_length))
    
rack_properties = []
for rack in rack_objects:
    # testing wall segment of 1.3 meters wide and .3 meters deep
    rack_length = 130
    rack_depth = 30
    _, center_point = sim.simxGetObjectPosition(clientId, rack[1], -1, sim.simx_opmode_oneshot_wait)
    _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientId, rack[1], -1, sim.simx_opmode_oneshot_wait)
    rack_properties.append((center_point, euler_gamma, rack_length))
    # rack_points.append((center_point[0], center_point[1]))

sliding_door_properties = []
for sliding_door in sliding_door_objects:
    sliding_door_length = 350
    _, center_point = sim.simxGetObjectPosition(clientId, sliding_door[1], -1, sim.simx_opmode_oneshot_wait)
    _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientId, sliding_door[1], -1, sim.simx_opmode_oneshot_wait)
    rack_properties.append((center_point, euler_gamma, sliding_door_length))

door_properties = []
for door in door_objects:
    door_length = 75
    _, center_point = sim.simxGetObjectPosition(clientId, door[1], -1, sim.simx_opmode_oneshot_wait)
    _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientId, door[1], -1, sim.simx_opmode_oneshot_wait)
    door_properties.append((center_point, euler_gamma, door_length))

_, table_point = sim.simxGetObjectPosition(clientId, table[1], -1, sim.simx_opmode_oneshot_wait)
_, (_, _, table_gamma) = sim.simxGetObjectOrientation(clientId, table[1], -1, sim.simx_opmode_oneshot_wait)
table_point = (table_point[0], table_point[1])
table_length = 140
table_properties = [
    ((table_point[0], table_point[1]+.3), table_gamma + np.pi/2, table_length),
    ((table_point[0], table_point[1]), table_gamma + np.pi/2, table_length),
    ((table_point[0], table_point[1]-.3), table_gamma + np.pi/2, table_length)
]

_, chair_point = sim.simxGetObjectPosition(clientId, chair[1], -1, sim.simx_opmode_oneshot_wait)
chair_point = (chair_point[0], chair_point[1])

#%%
# Fill in wall
mesh_points = []
for segment in segment_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*segment)))

for rack in rack_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*rack)))

for sliding_door in sliding_door_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*sliding_door)))

for door in door_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*door)))

# mesh_points.append(table_point)
for table in table_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*table)))
mesh_points.append(chair_point)

# Build obstacle coordinates
xs, ys = list(zip(*mesh_points))
ox = xs
oy = ys

#%%
# Get starting posotopm of robot
gps = RobotGPS(clientId)
gps_start = gps.get_position(actual=True)
odometer = Odometer(clientId, 0.097, pose=[gps_start[0], gps_start[1], gps.get_orientation()])
sx = gps_start[0]
sy = gps_start[1]

#%%
#Schow grid
show_animation = True
if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

#%%
# Build path
a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
rx, ry = a_star.planning(sx, sy, gx, gy)

#%%
# Show path
if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()

#%%
# Coordinates to drive along
destination_queue = [(rx[i], ry[i]) for i in range(len(rx))]
destination_queue.reverse()
navigation.set_queue(destination_queue)

while destination_queue:
    gps.update_position()
    navigation.update(gps.get_pose())
    odometer.update_motors()

    export_data.append({
        'actual_x': gps.get_position(actual=True)[0],
        'actual_y': gps.get_position(actual=True)[1],
        'odometer_x': odometer.pose[0],
        'odometer_y': odometer.pose[1],
        'gps_x': gps.get_position()[0],
        'gps_y': gps.get_position()[1],
        'v': odometer.get_velocity()
    })
    # loop executes once every 0.1 seconds (= 10 Hz)
    time.sleep(0.1)

# Post Allocation - Stop
navigation.stop()

# save data
if saving_data:
    with open('../output.json', 'w') as data_out:
        data_out.write(json.dumps(export_data))