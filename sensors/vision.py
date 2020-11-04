import sim


class VisionSensorP3DX:

    def __init__(self, client_id, handle):
        self.client_id = client_id
        self._handle = handle
        self.op_mode = sim.simx_opmode_oneshot_wait

    def read(self):
        code, state, aux_packets = sim.simxReadVisionSensor(
            self.client_id, self._handle, self._def_op_mode)
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