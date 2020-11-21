import numpy as np
import sim

PI = np.pi  # constant

# TODO: Document how this is working
#  Update distances more efficiently
#  Understand why braitenberg min is 0 so often


class ProximitySensorP3DX:
    def __init__(self, client_id):
        self.client_id = client_id
        self._handles = []
        self._sensor_vals = np.zeros(16)
        self.op_mode = sim.simx_opmode_oneshot_wait

        self.sensor_locs = np.array([
            -PI / 2, -50 / 180.0 * PI, -30 / 180.0 * PI,
            -10 / 180.0 * PI, 10 / 180.0 * PI, 30 / 180.0 * PI,
            50 / 180.0 * PI, PI / 2, PI / 2,
            130 / 180.0 * PI, 150 / 180.0 * PI, 170 / 180.0 * PI,
            -170 / 180.0 * PI, -150 / 180.0 * PI, -130 / 180.0 * PI,
            -PI / 2
        ])

        # for loop to retrieve sensor arrays and initiate sensors
        for i in range(1, 16 + 1):
            # build list of handles
            _, sensor_handle = sim.simxGetObjectHandle(
                client_id,
                'Pioneer_p3dx_ultrasonicSensor' + str(i),
                self.op_mode)
            self._handles.append(sensor_handle)

        # Load Values
        self.update_distances()

    def update_distances(self):
        self._sensor_vals = np.zeros(16)
        for i, handle in enumerate(self._handles):
            _, det_state, det_point, det_obj_handle, det_surface_norm = \
                sim.simxReadProximitySensor(
                    self.client_id, handle, self.op_mode
                )

            # print(f"i: {i}\n"
            #       f"det_point: {det_point}\n"
            #       f"Norm {np.linalg.norm(det_point)}")

            self._sensor_vals[i] = np.linalg.norm(det_point) if np.linalg.norm(det_point) > 1e-10 else np.Inf

    def braitenberg_min(self):
        """Braitenberg pathing by avoiding the nearest wall

        The ...

        f(x) ~ -1 / x
        1: -PI / 2,          (neg) -.5  => 2 PI
        2: -50 / 180.0 * PI, (neg) -.3  => 3.33 PI
        3: -30 / 180.0 * PI, (neg) -.17 => 6 PI
        4: -10 / 180.0 * PI, (neg) -.06 => 18 PI
        5: 10 / 180.0 * PI,  (pos) .06  => -18 PI
        6: 30 / 180.0 * PI,  (pos) .17  => -6 PI
        7: 50 / 180.0 * PI,  (pos) -.3  => -3.33 PI
        8: PI / 2, PI / 2,   (pos) -.5  => -2 PI

        :return distance to nearest, angle_to_nearest, min_sensor_index:
        """
        # square the values of front-facing sensors 1-8
        sensor_sqs = self._sensor_vals[0:8] * self._sensor_vals[0:8]
        # print({i: sensor_sqs[i] for i in range(len(sensor_sqs))})
        min_ind = np.where(sensor_sqs == np.min(sensor_sqs))
        min_ind = min_ind[0][0]
        return self._sensor_vals[min_ind], self.sensor_locs[min_ind], min_ind
