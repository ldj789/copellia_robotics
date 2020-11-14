import numpy as np

# sample 1
# [0, 0, 0], [0, 3] -> -1/2 pi

np.arctan2(3, 0) / np.pi - 0

# sample 2
# [0, 0, -1 or 1], [2, 2] -> -3/4
(np.arctan2(2, 2) / np.pi - -1)

# sample 3
# [0, 0, -1 or 1], [1, 0] -> -1 or 1
np.arctan2(0, 1) / np.pi - -1

# sample 4
# [0, 0, -0.5], [0, 3] -> -1 or 1
np.arctan2(3, 0) / np.pi + -0.5
(np.arctan2(3, 0) / np.pi - -0.5)

coords = {
    1: (-3.9, -0.5),
    2: (-3.9, 3.9),
    3: (3.5, 3.9),
}


def turn_to_point(current_pose, dest):
    """Turn to point from pose

    Find the radians to turn in order to face point

    :param current_pose: current pose in x, y, theta (rads)
    :param dest: destination x, y
    :return: degrees to turn (rads)
    """
    phi = np.arctan2(dest[1] - current_pose[1], dest[0] - current_pose[0]) / np.pi
    bearing = current_pose[2]
    rotation = phi - bearing
    if rotation > 1:
        rotation -= 2
    return rotation


"""
bearing = imu.read_euler()[0] * math.pi/180
phi = np.arctan2(y_total, x_total)
# math.pi, bearing, and phi in radians, rotation in deg
rotation = -((math.pi + bearing + phi)*180/math.pi)
rotation = rotation % 360
print(“current yaw CW from north = %8.2f rotation = %8.2f” % (bearing, rotation))
"""

if __name__ == "__main__":
    # sample 1
    # [0, 0, 0], [0, 3] -> 1/2 pi
    assert(np.abs(turn_to_point([0, 0, 0], [0, 3]) - 0.5) < 0.01)

    # sample 2
    # [0, 0, -1 or 1], [2, 2] -> -3/4
    assert(np.abs(turn_to_point([0, 0, -1], [2, 2]) - -0.75) < 0.01)
    assert(np.abs(turn_to_point([0, 0, 1], [2, 2]) - -0.75) < 0.01)

    # sample 3
    # [0, 0, -1 or 1], [1, 0] -> -1 or 1
    assert (np.abs(turn_to_point([0, 0, 1], [1, 0]) - -1) < 0.01)
    # ---- Failed Test ----
    assert (np.abs(turn_to_point([0, 0, -1], [1, 0]) - -1) < 0.01)
    turn_to_point([0, 0, 1], [1, 0]) - 1

    # sample 4
    # [0, 0, -0.5], [0, 3] -> -1 or 1
    assert(np.abs(turn_to_point([0, 0, -0.5], [0, 3]) - -1) < 0.01)
