import numpy as np


def turn_to_point(current_pose, dest):
    """Turn to point from pose

    Find the radians to turn in order to face point

    In robot currently positive for left and negative for right

    :param current_pose: current pose in x, y, theta (rads)
    :param dest: destination x, y
    :return: degrees to turn (rads)
    """
    phi = np.arctan2(dest[1] - current_pose[1], dest[0] - current_pose[0]) / np.pi
    bearing = current_pose[2] / np.pi
    rotation = phi - bearing
    if rotation > 1:
        rotation -= 2
    elif rotation < -1:
        rotation += 2
    return rotation * np.pi


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
    assert(np.abs(turn_to_point([0, 0, 0], [0, 3]) - 0.5 * np.pi) < 0.01)

    # sample 2
    # [0, 0, -1 or 1], [2, 2] -> -3/4
    assert(np.abs(turn_to_point([0, 0, -1 * np.pi], [2, 2]) - -0.75 * np.pi) < 0.01)
    assert(np.abs(turn_to_point([0, 0, 1 * np.pi], [2, 2]) - -0.75 * np.pi) < 0.01)

    # sample 3
    # [0, 0, -1 or 1], [1, 0] -> -1 or 1
    assert (np.abs(turn_to_point([0, 0, 1 * np.pi], [1, 0])) - 1 * np.pi < 0.01)
    # ---- Failed Test ----
    assert (np.abs(turn_to_point([0, 0, -1 * np.pi], [1, 0])) - 1 * np.pi < 0.01)

    # sample 4
    # [0, 0, -0.5], [0, 1] -> -1 or 1
    assert(np.abs(turn_to_point([0, 0, -0.5 * np.pi], [0, 1])) - 1  * np.pi < 0.01)
