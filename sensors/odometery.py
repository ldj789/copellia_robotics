"""Coppelia Odometery Class for creating wheel encoders

Inspiration:
Leo Armesto English V-REP tutorial, video 42
https://www.youtube.com/watch?v=e1qcfDuVN1w&list=PLjzuoBhdtaXOoqkJUqhYQletLLnJP8vjZ&index=42

Steps
1 Initialize Odometery system
2 Obtain angular displacement from the speed of each joint
3 Update value of odometer using
4 Update the position of the robot

functions:
__init__ will initialize the odometery system

Position/Orientation Estimation

Formulas for linear and angular displacement:
delS = (delSl + delr) / 2
delTheta = (delSl + delr) / 2b

Position and Orientation:
x_t+1 = x_t + delS * cos theta
y_t+1 = y_t + delS * sin theta
theta_t+1 = theta_t + delTheta
"""
from datetime import datetime
import sim
import numpy as np


class Odometer:
    """Track Odometers based upon wheel movements"""

    def __init__(
            self,
            client_id,
            r,
            left_motor_name='Pioneer_p3dx_leftMotor',
            right_motor_name='Pioneer_p3dx_rightMotor',
            **kwargs
    ):
        self.client_id = client_id
        self.r = r  # Wheel Radius

        self.lm_status, self.left_motor_handle = sim.simxGetObjectHandle(
            self.client_id,
            left_motor_name,
            sim.simx_opmode_oneshot_wait
        )
        self.rm_status, self.right_motor_handle = sim.simxGetObjectHandle(
            self.client_id,
            right_motor_name,
            sim.simx_opmode_oneshot_wait
        )
        self.b = self.get_b_parameter()
        # update motors will update this each time its called
        self.left_odometer = 0
        self.right_odometer = 0

        self.pose = kwargs['pose'] if 'pose' in kwargs else [0, 0, 0]
        self.last_time = datetime.now()

    def get_b_parameter(self):
        _, rr = sim.simxGetObjectPosition(self.client_id, self.right_motor_handle, -1, sim.simx_opmode_oneshot_wait)
        _, rl = sim.simxGetObjectPosition(self.client_id, self.left_motor_handle, -1, sim.simx_opmode_oneshot_wait)
        return np.sqrt((rr[0] - rl[0]) ** 2 + (rr[1] - rl[1]) ** 2 + (rr[2] - rl[2]) ** 2) / 2

    def update_motors(self):
        """Get new measurements for velocity and update displacement"""
        # Collect joint params
        _, wL = sim.simxGetObjectFloatParameter(self.client_id, self.left_motor_handle, 2012, sim.simx_opmode_oneshot_wait)
        # sim.sim_jointfloatparam_velocity(self.left_motor_handle)
        _, wR = sim.simxGetObjectFloatParameter(self.client_id, self.right_motor_handle, 2012, sim.simx_opmode_oneshot_wait)
        # _, wR = sim.sim_jointfloatparam_velocity(self.right_motor_handle)

        # Get time elapsed in seconds
        delta_time = (datetime.now() - self.last_time).total_seconds()
        # Set prior time to now
        self.last_time = datetime.now()

        # Calculate thetas
        theta_l = wL * delta_time * self.r
        theta_r = wR * delta_time * self.r

        # Update odometers
        self.left_odometer += theta_l
        self.right_odometer += theta_r

        # Distance Traveled
        delta_s = (theta_l + theta_r) / 2

        # Change in Rotation
        delta_theta = (theta_r - theta_l) / (2 * self.b)  # b = width of cart

        self.pose[2] = self.pose[2] + delta_theta
        self.pose[0] = self.pose[0] + delta_s * np.cos(self.pose[2])
        self.pose[1] = self.pose[1] + delta_s * np.sin(self.pose[2])

    def update_odometry(self):
        """Use new """
        pass

    def get_position(self):
        return self.pose[0], self.pose[1], self.pose[2]

    # def set_pose(self, new_pose):
    #     self.pose = [x, y, theta]

    def __str__(self):
        return f"Odometer {self.left_odometer} {self.right_odometer} {self.pose}"
