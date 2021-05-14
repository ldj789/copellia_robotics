#%%
import sys
import time  # used to keep track of time
import numpy as np  # array library
import re
import json
import matplotlib.pyplot as plt

import sim
from drive.navigate import turn_to_point, check_destination
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

# from sensors.proximity import ProximitySensorP3DX

#%%
# Initial Variables
speed_setting = 1.25
PI = np.pi  # constant
saving_data = False
robot_radius = .38
grid_size = .25
gx = -4.0
gy = -4.0

export_data = []

#%%
# Connect to simulator
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

#%%
# Retrieve map obstacles
_, scene_indices, _, _, object_names = sim.simxGetObjectGroupData(clientID, sim.sim_object_shape_type, 0, sim.simx_opmode_oneshot_wait)
scene_objects = list(zip(object_names, scene_indices))
# Retrieve wall objects
wall_objects = list(filter(lambda x: x[0].startswith("80cmHighWall"), scene_objects))

# Retrieve center point, length, and orientation of wall segment
segment_properties = []
for segment in wall_objects:
    length_match = re.match("80cmHighWall(\d+)cm", segment[0])
    if length_match:
        segment_length = int(length_match.group(1))
        _, center_point = sim.simxGetObjectPosition(clientID, segment[1], -1, sim.simx_opmode_oneshot_wait)
        _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientID, segment[1], -1, sim.simx_opmode_oneshot_wait)
        segment_properties.append((center_point, euler_gamma, segment_length))

#%%
# Fill in wall
mesh_points = []
for segment in segment_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*segment)))

# Build obstacle coordinates
xs, ys = list(zip(*mesh_points))
ox = xs
oy = ys

#%%
# retrieve motor  handles
_, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
_, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

# Get starting poistion of robot
gps = RobotGPS(clientID)
gps_start = gps.get_position(actual=True)
odometer = Odometer(clientID, 0.097, pose=[gps_start[0], gps_start[1], gps.get_orientation()[2]])
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
destination_queue  = [(rx[i], ry[i]) for i in range(len(rx))]
destination_queue.reverse()
current_destination = destination_queue.pop(0)
current_pose = gps.get_pose()
kf = GpsOdometerKf(gps, odometer, pose=current_pose)

print(
    f"Staring Position\n" 
    f"pose: {current_pose}\n"
    f"target: {current_destination}\n"
    f"required turn: {turn_to_point(current_pose, current_destination)}"
)


while destination_queue:
    gps.update_position()
    odometer.update_motors()
    kf.update()
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
        'kf_x': kf.get_position()[0],
        'kf_y': kf.get_position()[1],
        'v': odometer.get_velocity()
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
# %%
