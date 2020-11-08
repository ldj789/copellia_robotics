import numpy as np

current_pose = [-4.13474822,  0.09394728, 3.1188435554504395]
next_point = [-3.9, 3.9]

desired_theta = np.arctan2(next_point[1] - current_pose[1], next_point[0] - current_pose[0])
current_theta = current_pose[2]

rotation = -(np.pi + current_theta + desired_theta) % np.pi

desired_theta = np.arctan2(next_point[1] - current_pose[1], next_point[0] - current_pose[0])
current_theta = -1.57

rotation = -(np.pi + current_theta + desired_theta) % np.pi


# bearing = imu.read_euler()[0] * math.pi/180
# phi = np.arctan2(y_total, x_total)
# # math.pi, bearing, and phi in radians, \
# rotation = -((math.pi + bearing + phi)*180/math.pi)
# rotation = rotation % 360
# print("current yaw CW from north = %8.2f rotation = %8.2f" % (bearing, rotation))
