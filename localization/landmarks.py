"""Landmark

Approximate objects in Coppelia scene as a bluetooth object capable of relaying
with a robot and reporting distance and its position

construct
 - kwarg 'robot_frame' is the name of the robot in the scene
 - kwarg 'landmark_frames' is a list of landmark names from the coppelia object
 - kwargs 'landmark_handles is a list of landmark handles (simx object handles)

"""
import sim
import numpy as np


class Landmark:
    """Helper class for Robot Landmarks"""
    def __init__(self, client_id, handle):
        self._client_id = client_id
        self._def_op_mode = sim.simx_opmode_oneshot_wait
        self._handle = handle
        _, self.position = sim.simxGetObjectPosition(self._client_id, self._handle, -1, self._def_op_mode)

    def update_position(self, **kwargs):
        _, self.position = sim.simxGetObjectPosition(self._client_id, self._handle, -1, self._def_op_mode)


class RobotLandmarks:
    def __init__(self, client_id, **kwargs):
        self._client_id = client_id
        self._def_op_mode = sim.simx_opmode_oneshot_wait

        if 'robot_handle' in kwargs:
            self._robot_handle = kwargs['robot_handle']
        else:
            _, self._robot_handle = sim.simxGetObjectHandle(
                self._client_id,
                kwargs.get('robot_frame', 'Pioneer_p3dx'),
                self._def_op_mode
            )

        self._landmark_handles = []
        if 'landmark_handles' in kwargs:
            # self.extend_landmarks_from handles(kwargs['landmark_handles'])
            for lm in kwargs['landmark_handles']:
                self.append_landmark_from_handle(lm)

        if 'landmark_frames' in kwargs:
            for lm in kwargs['landmark_frames']:
                self.append_landmark_from_frame(lm)

        self.landmarks = [Landmark(self._client_id, handle) for handle in self._landmark_handles]

    def __str__(self):
        return "<" + ", ".join([str(lm) for lm in self._landmark_handles]) + ">"

    def __len__(self):
        return len(self._landmark_handles)

    # TODO complete
    def add(self):
        raise NotImplementedError

    # TODO complete
    def remove(self):
        raise NotImplementedError

    def append_landmark_from_handle(self, lm_handle):
        self._landmark_handles.append(lm_handle)

    def append_landmark_from_frame(self, lm_frame):
        _, lm_handle = sim.simxGetObjectHandle(
            self._client_id,
            lm_frame,
            self._def_op_mode
        )
        self._landmark_handles.append(lm_handle)

    def get_robot_position(self):
        _, res = sim.simxGetObjectPosition(self._client_id, self._robot_handle, -1, self._def_op_mode)
        return res

    @staticmethod
    def calculate_range(robot_pov_pos, landmark_pos):
        """
        :param robot_pov_pos: np.array()
        :param landmark_pos: np.array()
        :return: scalar of distance
        """
        dx = robot_pov_pos[0] - landmark_pos[0]
        dy = robot_pov_pos[1] - landmark_pos[1]
        d = np.sqrt(dx ** 2 + dy ** 2)
        theta = np.arctan2(dy, dx) - robot_pov_pos[2]
        return np.array([d, theta])

    def landmark_ranges(self, **kwargs):
        """Get range and angle for each landmark"""
        # robot position -  private to this function
        robot_pov_pos = self.get_robot_position()
        print(robot_pov_pos)
        # res = []
        res = np.zeros((0, 2))
        for i, lm_handle in enumerate(self._landmark_handles):
            _, landmark_pos = sim.simxGetObjectPosition(self._client_id, lm_handle, -1, self._def_op_mode)
            # res.append(self.calculate_range(robot_pov_pos, landmark_pos))
            np.vstack(res, self.calculate_range(robot_pov_pos, landmark_pos))
        return res

    def is_visible(self, lm):
        """Check if landmark is within robot line of sight"""
        raise NotImplementedError

    def visible_landmarks(self):
        """Check which landmarks are visible

        :returns: array of visible landmarks corresponding
        """
        return filter(self.is_visible, self._landmark_handles)

    # TODO: move doc string note to calculate range or Kalman
    # def find_lm_pos(self, lm):
    #     """Find relative position of landmarks
    #
    #     calculate distance from robot to landmark
    #     calculate angle from robot to landmark
    #     d = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
    #     angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
    #     add noise
    #     convert from polar to euclidean w.r.t. robots position (robot is the origin)
    #     zp = np.zeros((2, 1))
    #
    #     zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    #     zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    #
    #     :returns:
    #     """
    #     # use robot position
    #     robot_pos = self.get_robot_position()
    #
    #     res = []
    #     for lm in self._landmark_handles:
    #         res.append(lm.get_polar_position())
    #     lm_abs_pos = lm
    #     raise NotImplementedError

    def get_landmark_positions(self):
        """Get a list of landmark (x, y) coordinates"""
        return [np.array(lm.position) for lm in self.landmarks]

    def update_landmark_positions(self):
        """update landmark positions"""
        for lm in self.landmarks:
            lm.update_position()
