import sim
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


class Navigation:
    def __init__(self,
                 client_id,
                 left_motor_frame='Pioneer_p3dx_leftMotor',
                 right_motor_frame='Pioneer_p3dx_rightMotor',
                 speed=1,
                 steering_gain=1,
                 **kwargs):

        self.client_id = client_id
        _, self.left_motor_handle = sim.simxGetObjectHandle(client_id, left_motor_frame, sim.simx_opmode_oneshot_wait)
        _, self.right_motor_handle = sim.simxGetObjectHandle(client_id, right_motor_frame, sim.simx_opmode_oneshot_wait)
        self.speed = speed
        self.steering_gain = steering_gain
        self.queue = kwargs.get('queue', [])
        self.destination = self.queue.pop(0) if len(self) > 0 else None
        print(self.queue)

    def __str__(self):
        return f"Speed {self.speed}, Gain {self.steering_gain} Destination {self.destination}"

    def __len__(self):
        return len(self.queue)

    def check_destination(self, pose, d=0.4):
        """Check proximity to destination and maybe update pathing"""
        if self.destination is None:
            return

        dist = np.sqrt((pose[0] - self.destination[0]) ** 2 + (pose[1] - self.destination[1]) ** 2)
        if dist < d:
            self.destination = self.queue.pop(0) if len(self) > 0 else None

    def steer(self, pose):
        if self.destination is None:
            v, steer = 0, 0
        else:
            v, steer = self.speed, self.turn_to_point(pose) / np.pi

        vl = v - self.steering_gain * steer
        vr = v + self.steering_gain * steer

        _ = sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, vl, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, vr, sim.simx_opmode_streaming)

    def stop(self):
        _ = sim.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, 0, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, 0, sim.simx_opmode_streaming)

    def turn_to_point(self, pose):
        """Turn to point from pose

        Find the radians to turn in order to face point

        In robot currently positive for left and negative for right

        :param pose: current pose in x, y, theta (rads)
        :param dest: destination x, y
        :return: degrees to turn (rads)
        """
        phi = np.arctan2(self.destination[1] - pose[1], self.destination[0] - pose[0]) / np.pi
        bearing = pose[2] / np.pi
        rotation = phi - bearing
        if rotation > 1:
            rotation -= 2
        elif rotation < -1:
            rotation += 2
        return rotation * np.pi

    def update(self, pose, d=0.4):
        self.check_destination(pose, d=d)
        self.steer(pose)

    def set_queue(self, way_points):
        self.queue = way_points
        self.destination = self.queue.pop(0) if len(self) > 0 else None

    def insert_queue(self, way_point):
        """Store current way point and use provided way point as destination"""
        self.queue.insert(0, self.destination)
        self.destination = way_point

    def extend_queue(self, way_points):
        if isinstance(way_points, list):
            self.queue.extend(way_points)
        else:  # Single way_point
            self.queue.append(way_points)

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
