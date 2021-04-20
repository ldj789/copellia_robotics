#%%
import numpy as np
import re
import sys
import sim
import matplotlib.pyplot as plt
from drive.astar import AStarPlanner
from pprint import pprint
from sensors.position import RobotGPS

#%%
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')
    
#%%
#simx.GetObjects to return an array of object handles
_, scene_indices, _, _, object_names = sim.simxGetObjectGroupData(clientID, sim.sim_object_shape_type, 0, sim.simx_opmode_oneshot_wait)
scene_objects = list(zip(object_names, scene_indices))
wall_objects = list(filter(lambda x: x[0].startswith("80cmHighWall"), scene_objects))

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
segment_properties = []
for segment in wall_objects:
    length_match = re.match("80cmHighWall(\d+)cm", segment[0])
    if length_match:
        segment_length = int(length_match.group(1))
        _, center_point = sim.simxGetObjectPosition(clientID, segment[1], -1, sim.simx_opmode_oneshot_wait)
        _, (_, _, euler_gamma) = sim.simxGetObjectOrientation(clientID, segment[1], -1, sim.simx_opmode_oneshot_wait)
        segment_properties.append((center_point, euler_gamma, segment_length))

#%%
mesh_points = []
for segment in segment_properties:
    mesh_points.extend(get_wall_mesh_points(*get_wall_endpoints(*segment)))   

#%%
xs, ys = list(zip(*mesh_points))

#%%
robot_radius = .38
gps = RobotGPS(clientID)
gps_start = gps.get_position(actual=True)
ox = xs
oy = ys
grid_size = .25
sx = gps_start[0]
sy = gps_start[1]
gx = -4.0
gy = -4.0

#draw 2d grid of map
# %%
show_animation = True
if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
rx, ry = a_star.planning(sx, sy, gx, gy)

if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()
