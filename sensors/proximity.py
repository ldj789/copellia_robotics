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
        self._prior_buffer = 1

        self._sensor_locs = np.array([
            [0.10637688636779785, 0.1381988525390625, -PI / 2],  # 1
            [0.15540122985839844, 0.12504959106445312, -50 / 180.0 * PI],  # 2
            [0.19057726860046387, 0.08313345909118652, -30 / 180.0 * PI],  # 3
            [0.20916175842285156, 0.02727365493774414, -10 / 180.0 * PI],  # 4
            [0.20916152000427246, -0.027272939682006836, 10 / 180.0 * PI],  # 5
            [0.19054222106933594, -0.07850122451782227, 30 / 180.0 * PI],  # 6
            [0.15552043914794922, -0.1202390193939209, 50 / 180.0 * PI],  # 7
            [0.10637736320495605, -0.13814449310302734, PI / 2],  # 8
            [-0.11032390594482422, -0.13815093040466309, PI / 2],  # 9
            [-0.15956640243530273, -0.12024164199829102, 130 / 180.0 * PI],  # 10
            [-0.1945810317993164, -0.07850146293640137, 150 / 180.0 * PI],  # 11
            [-0.21320772171020508, -0.027275562286376953, 170 / 180.0 * PI],  # 12
            [-0.21320772171020508, 0.027273178100585938, -170 / 180.0 * PI],  # 13
            [-0.19458913803100586, 0.07849884033203125, -150 / 180.0 * PI],  # 14
            [-0.15956735610961914, 0.12026429176330566, -130 / 180.0 * PI],  # 15
            [-0.11032295227050781, 0.1381995677947998, -PI / 2]  # 16
        ])

        # for loop to retrieve sensor arrays and initiate sensors
        for i in range(1, 16 + 1):
            # build list of handles
            _, sensor_handle = sim.simxGetObjectHandle(
                client_id,
                'Pioneer_p3dx_ultrasonicSensor' + str(i),
                self.op_mode)
            self._handles.append(sensor_handle)

        for handle in self._handles:
            _, _, _, _, _ = sim.simxReadProximitySensor(self.client_id, handle, sim.simx_opmode_streaming)

        # Load Values
        self.update_distances()

    def update_distances(self):
        self._sensor_vals = np.zeros(16)
        for i, handle in enumerate(self._handles):
            _, det_state, det_point, det_obj_handle, det_surface_norm = \
                sim.simxReadProximitySensor(
                    self.client_id, handle, sim.simx_opmode_buffer
                    # self.client_id, handle, sim.simx_opmode_oneshot_wait
                )
            buffer_match = abs(det_point[2] - self._prior_buffer) < 1e-3
            self._prior_buffer = det_point[2]

            print(f"i: {i}\n"
                  f"buffer match: {buffer_match}\n"
                  f"det_point: {det_point}\n"
                  f"Norm {np.linalg.norm(det_point)}")

            # self._sensor_vals[i] = np.linalg.norm(det_point) if np.linalg.norm(det_point) > 1e-10 else np.Inf
            self._sensor_vals[i] = det_point[2] if (det_point[2] > 1e-8 and not buffer_match) else np.Inf

    def get_distances(self, **kwargs):
        if 'simple' in kwargs and kwargs['simple']:
            return self._sensor_vals
        return np.vstack((self._sensor_vals, self._sensor_locs[:, 2])).T

    def get_indexed_locations(self, **kwargs):
        """get locations for configuring proximity sensor on robot"""
        adj = [0, 0, 0] if 'pose' not in kwargs else kwargs['pose']
        res = []
        for i, handle in enumerate(self._handles):
            _, pos = sim.simxGetObjectPosition(self.client_id, handle, -1, self.op_mode)
            _, orientation = sim.simxGetObjectOrientation(self.client_id, handle, -1, self.op_mode)
            res.append([i+1, pos[0] - adj[0], pos[1] - adj[1], (orientation[1] - adj[0] + np.pi) % (2*np.pi) - np.pi])
        return res

    def get_related_info(self, related_object_handle):
        """get locations for configuring proximity sensor on robot"""
        res = []
        for i, handle in enumerate(self._handles):
            _, pos = sim.simxGetObjectPosition(self.client_id, handle, related_object_handle, self.op_mode)
            # _, orientation = sim.simxGetObjectOrientation(self.client_id, handle, related_object_handle, self.op_mode)
            res.append(pos)
        return res

    def get_proximate_objects(self, current_pose, cutoff=10):
        """calculate obstacle positions
        robot_x + distance * np.cos(obs_theta + robot_theta),
        robot_y + distance * np.sin(obs_theta + robot_theta)
        sensors have a relative facing angle and relative position angle (theta_rp)
        :param current_pose: current (x, y, theta) for robot
        :param cutoff: distance cutoff
        :return: list of (x, y) for observed obstacles
        """
        obstacle_distances = self.get_distances()
        res = []
        for i, obstacle in enumerate(obstacle_distances):
            if obstacle[0] < cutoff:
                x_rp = self._sensor_locs[i][0]
                y_rp = self._sensor_locs[i][1]
                d_rp = np.sqrt(x_rp**2 + y_rp**2)
                theta_rp = np.arctan2(y_rp, x_rp)
                dx = d_rp * np.cos(current_pose[2] + theta_rp)
                dy = d_rp * np.sin(current_pose[2] + theta_rp)
                # dx = 0
                # dy = 0
                res.append((
                    current_pose[0] + dx + obstacle[0] * np.cos(current_pose[2] - obstacle[1]),
                    current_pose[1] + dy + obstacle[0] * np.sin(current_pose[2] - obstacle[1])
                ))
                # obstacle_positions.append(obstacle_position)
                # print(
                #     f"obs: {obstacle_position}\n"
                #     f"bearing {current_pose[2]} adjustment {obstacle[1]}"
                # )
        return res

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
        return self._sensor_vals[min_ind], self._sensor_locs[:, 2][min_ind], min_ind
