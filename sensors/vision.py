import os
import time  # used to keep track of time
import numpy as np  # array library
import cv2
import sim

from datetime import datetime
from pathlib import Path

class VisionSensorP3DX:

    def __init__(self, client_id,handle_name='Pioneer_p3dx_vision'):
        self.client_id = client_id
        self.op_mode = sim.simx_opmode_oneshot_wait
        _, self._handle = sim.simxGetObjectHandle(
                self.client_id,
                handle_name,
                self.op_mode
        )

    def read(self):
        code, state, aux_packets = sim.simxReadVisionSensor(
            self.client_id, self._handle, self.op_mode)
        return state, aux_packets

    def raw_image(self, is_grey_scale=False):
        """
        Retrieves the image of a vision sensor.
        @return the image data
        """
        code, resolution, image = sim.simxGetVisionSensorImage(
            self.client_id, self._handle, int(is_grey_scale), self.op_mode)
        return image

    def image_resolution(self, is_grey_scale=False):
        """
        Retrieves the resolution of the image of a vision senor.
        @return the image resolution
        """
        code, resolution, image = sim.simxGetVisionSensorImage(
            self.client_id, self._handle, int(is_grey_scale), self.op_mode)
        return resolution

    def depth_buffer(self):
        """
        Retrieves the depth buffer of a vision sensor.
        @return the buffer
        """
        code, resolution, buffer = sim.simxGetVisionSensorDepthBuffer(
            self.client_id, self._handle, self.op_mode)
        return buffer
        # values are in the range of 0-1 (0=closest to sensor, 1=farthest from sensor).

    def save_images(self,scene_name):
        """
        Saves images from the vision sensor to the images directory in jpeg format.
        """
        if self.client_id != 1:
            print('Getting images')
            dirname = "images\\"
            os.makedirs(dirname)
            while self.client_id != 1:
                image = self.raw_image()
                resolution = self.image_resolution()
                img = np.array(image,dtype=np.uint8)
                img.resize([self.image_resolution()[1], self.image_resolution()[0], 3])
                img = cv2.flip(img, 0)
                cv2.imwrite(dirname + "\{0}.jpg".format(scene_name +"_"+ str(time.time())), img)
                time.sleep(0.1)
                if cv2.waitKey(0) == 0x1b:
                    break
        else:
            print('No connection')