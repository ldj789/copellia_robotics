#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  3 19:50:33 2020

Ref: 
    https://github.com/Troxid/vrep-api-python/blob/master/pyrep/sensors.py
    https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm

"""

# Import Libraries:
import sim                  #V-rep library
#import sys
#import time                #used to keep track of time
#import numpy as np         #array library
#import math
#import matplotlib as mpl   #used for image plotting
from .common import MatchObjTypeError, NotFoundComponentError
from .common import Vec3, EulerAngles


class ProximitySensor:

    def __init__(self, id, handle):
        self._id = id
        self._handle = handle
        # Blocking mode: https://www.coppeliarobotics.com/helpFiles/en/remoteApiModusOperandi.htm
        self._def_op_mode = sim.simx_opmode_oneshot_wait 
        
        
    def read(self) -> (bool, Vec3):
        """
        Reads the state of a proximity sensor.
        @return detection state and detected point
        @rtype (bool, Vec3)
        """
        code, state, point, handle, snv = sim.simxReadProximitySensor(
            self._id, self._handle, self._def_op_mode)
        return state, Vec3(point[0], point[1], point[2])
    
 
class VisionSensor:

    def __init__(self, id, handle):
        self._id = id
        self._handle = handle
        self._def_op_mode = sim.simx_opmode_oneshot_wait

    def read(self):
        code, state, aux_packets = sim.simxReadVisionSensor(
            self._id, self._handle, self._def_op_mode)
        return state, aux_packets

    def raw_image(self, is_grey_scale=False):
        """
        Retrieves the image of a vision sensor.
        @return the image data
        """
        num_of_clr = 3
        if is_grey_scale:
            num_of_clr = 1

        code, resolution, image = sim.simxGetVisionSensorImage(
            self._id, self._handle, int(is_grey_scale), self._def_op_mode)
        return image


    def depth_buffer(self):
        """
        Retrieves the depth buffer of a vision sensor.
        @return the buffer
        """
        code, resolution, buffer = sim.simxGetVisionSensorDepthBuffer(
            self._id, self._handle, self._def_op_mode)
        return buffer
        # values are in the range of 0-1 (0=closest to sensor, 1=farthest from sensor).


class ForceSensor:

    def __init__(self, id, handle):
        self._id = id
        self._handle = handle
        self._def_op_mode = sim.simx_opmode_oneshot_wait

    def read(self) -> (bool, Vec3, Vec3):
        """
        Reads the force and torque applied to a force sensor
        (filtered values are read), and its current state ('unbroken' or 'broken').
        """
        code, state, force, torque, snv = sim.simxReadForceSensor(
            self._id, self._handle, self._def_op_mode)
        force_vector = Vec3(force[0], force[1], force[2])
        torque_vector = Vec3(torque[0], torque[1], torque[2])
        return state, force_vector, torque_vector
    
    
class PositionSensor:

    def __init__(self, client_id, handle):
        self._id = client_id
        self._handle = handle
        self._def_op_mode = sim.simx_opmode_oneshot_wait

    def get_position(self) -> Vec3:
        """Retrieves the orientation.
        @rtype: Vec3
        """
        code, pos = sim.simxGetObjectPosition(self._id, self._handle, -1, self._def_op_mode)
        return Vec3(pos[0], pos[1], pos[2])

    def get_orientation(self) -> EulerAngles:
        """
        Retrieves the linear and angular velocity.
        @rtype EulerAngles
        """
        code, orient = sim.simxGetObjectOrientation(self._id, self._handle, -1, self._def_op_mode)
        return EulerAngles(orient[0], orient[1], orient[2])

    def get_velocity(self) -> (Vec3, EulerAngles):
        """
        Retrieves the linear and angular velocity.
        @rtype (Vec3, EulerAngles)
        """
        code, lin_vel, ang_vel = sim.simxGetObjectVelocity(self._id, self._handle, self._def_op_mode)
        linear_velocity = Vec3(lin_vel[0], lin_vel[1], lin_vel[2])
        angular_velocity = EulerAngles(ang_vel[0], ang_vel[1], ang_vel[2])
        return linear_velocity, angular_velocity
    
    

class Sensors:

    def __init__(self, id):
        self._id = id
        self._def_op_mode = sim.simx_opmode_oneshot_wait


    def proximity(self, name: str) -> ProximitySensor:
        handle = self._get_object_handle(name)
        if handle is not None:
            if self._check_object_type(handle, sim.sim_object_proximitysensor_type):
                return ProximitySensor(self._id, handle)
            else:
                raise MatchObjTypeError(name)
        else:
            raise NotFoundComponentError(name)


    def position(self, name: str) -> PositionSensor:
        handle = self._get_object_handle(name)
        if handle is not None:
            return PositionSensor(self._id, handle)
        else:
            raise NotFoundComponentError(name)


    def vision(self, name: str) -> VisionSensor:
        handle = self._get_object_handle(name)
        if handle is not None:
            if self._check_object_type(handle, sim.sim_object_visionsensor_type):
                return VisionSensor(self._id, handle)
            else:
                raise MatchObjTypeError(name)
        else:
            raise NotFoundComponentError(name)


    def force(self, name: str) -> ForceSensor:
        handle = self._get_object_handle(name)
        if handle is not None:
            if self._check_object_type(handle, sim.sim_object_forcesensor_type):
                return ForceSensor(self._id, handle)
            else:
                raise MatchObjTypeError(name)
        else:
            raise NotFoundComponentError(name)


    def _get_sensor(self, name, sensor_type, ctr):
        handle = self._get_object_handle(name)
        if handle is not None:
            if self._check_object_type(handle, sensor_type):
                return ctr(self._id, handle)
            else:
                raise MatchObjTypeError(name)
        else:
            raise NotFoundComponentError(name)


    def _check_object_type(self, handle, obj_type):
        code, handles, _, _, _ = sim.simxGetObjectGroupData(
            self._id, obj_type, 0, self._def_op_mode)
        return handles.__contains__(handle)


    def _get_object_handle(self, name):
        code, handle = sim.simxGetObjectHandle(self._id, name, self._def_op_mode)
        if code == sim.simx_return_ok:
            return handle
        else:
            return None
        
