import numpy as np


def turn_to_point(pose, dest):
    """Turn to point from pose

    Find the radians to turn in order to face point

    In robot currently positive for left and negative for right

    :param pose: current pose in x, y, theta (rads)
    :param dest: destination x, y
    :return: degrees to turn (rads)
    """
    # if dest is None:
    #     return 0

    phi = np.arctan2(dest[1] - pose[1], dest[0] - pose[0]) / np.pi
    bearing = pose[2] / np.pi
    rotation = phi - bearing
    if rotation > 1:
        rotation -= 2
    elif rotation < -1:
        rotation += 2
    return rotation * np.pi


def check_destination(pose, dest, dest_queue, d=.25):
    """Check proximity to destination and maybe update pathing"""
    if dest is None:
        return None

    dist = np.sqrt((pose[0] - dest[0])**2 + (pose[1] - dest[1])**2)
    if dist < d:
        dest = dest_queue.pop(0) if len(dest_queue) > 0 else None
    return dest


def pi_mod4q(theta):
    """Unit circle modulo

    This is in handy in case we make any turns, we want them to be minimal
    so 90 degree direction not 270 degree direction
    """
    return ((theta + np.pi) % (2 * np.pi)) - np.pi

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
    assert(np.abs(turn_to_point([0, 0, -0.5 * np.pi], [0, 1])) - 1 * np.pi < 0.01)
