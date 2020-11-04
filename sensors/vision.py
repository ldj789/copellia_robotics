import sim


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


    def depth_buffer(self):
        """
        Retrieves the depth buffer of a vision sensor.
        @return the buffer
        """
        code, resolution, buffer = sim.simxGetVisionSensorDepthBuffer(
            self.client_id, self._handle, self.op_mode)
        return buffer
        # values are in the range of 0-1 (0=closest to sensor, 1=farthest from sensor).