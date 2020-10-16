"""Coppelia positioning classes
"""
from datetime import datetime
import sim
import numpy as np
from random import uniform


class PositionSensor:
    def __init__(self, client_id, handle_name):
        self._client_id = client_id
        self._handle_name = handle_name
        self._def_op_mode = sim.simx_opmode_oneshot_wait
        _, self._handle = sim.simxGetObjectHandle(
            self._client_id,
            handle_name,
            self._def_op_mode
        )
        
    def get_position(self):
        _, pos = sim.simxGetObjectPosition(self._client_id, self._handle, -1, self._def_op_mode)
        return np.array(pos)


class RobotGPS:
    def __init__(self,
                 client_id,
                 left_motor_name='Pioneer_p3dx_leftMotor',
                 right_motor_name='Pioneer_p3dx_rightMotor',
                 noise=.1):
        self.left_motor_position_sensor = PositionSensor(client_id, left_motor_name)
        self.right_motor_position_sensor = PositionSensor(client_id, right_motor_name)
        self._position_vec = self.set_position()
        self.noise = noise

    def __str__(self):
        return f"<x: {self._position_vec[0]}, y: {self._position_vec[1]}, z: {self._position_vec[2]}>"

    def set_position(self):
        rp = self.right_motor_position_sensor.get_position()
        lp = self.left_motor_position_sensor.get_position()
        return np.array([(rp[0] + lp[0]) / 2, (rp[1] + lp[1]) / 2, (rp[2] + lp[2]) / 2])

    def update_position(self):
        self._position_vec = self.set_position()

    def get_position(self, **kwargs):
        if 'actual' in kwargs and kwargs['actual']:
            return self._position_vec
        return self._position_vec + np.array([
            uniform(-self.noise, self.noise), uniform(-self.noise, self.noise), uniform(-self.noise, self.noise)
        ])


# --------------
# TODO:
#   Implement get_orientation
#   Implement velocity
#
#     def get_orientation(self) -> EulerAngles:
#         """
#         Retrieves the linear and angular velocity.
#         @rtype EulerAngles
#         """
#         code, orient = sim.simxGetObjectOrientation(self._id, self._handle, -1, self._def_op_mode)
#         return EulerAngles(orient[0], orient[1], orient[2])
#
#     def get_velocity(self) -> (Vec3, EulerAngles):
#         """
#         Retrieves the linear and angular velocity.
#         @rtype (Vec3, EulerAngles)
#         """
#         code, lin_vel, ang_vel = sim.simxGetObjectVelocity(self._id, self._handle, self._def_op_mode)
#         linear_velocity = Vec3(lin_vel[0], lin_vel[1], lin_vel[2])
#         angular_velocity = EulerAngles(ang_vel[0], ang_vel[1], ang_vel[2])
#         return linear_velocity, angular_velocity
#
