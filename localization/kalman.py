import time
import numpy as np
from drive.navigate import pi_mod4q


# noinspection PyPep8Naming
class GpsOdometerKf:
    def __init__(self, gps, odometer, **kwargs):
        """
        kwargs
        - position: 2 component x y representation
        - pose: 3 component x, y, theta representation (will use only x y here)
        """
        self.gps = gps
        self.odometer = odometer

        # initialize position
        self.position = [0, 0]
        if "position" in kwargs:
            self.position = kwargs["position"]
        if "pose" in kwargs:
            self.position = [kwargs["pose"][0], kwargs["pose"][1]]

        self.prior_odometer_position = self.odometer.get_position()

        # initialize matrices for kf calculation
        self.P = np.identity(2)  # keep
        self.H = np.identity(2)  # keep

    def update(self):
        # [gps x - pred x, gps y - pred y]
        u_t_sensor = np.array(
            [self.gps.get_position()[0] - self.position[0],
             self.gps.get_position()[1] - self.position[1]]
        )

        # [odometer_t x - odometer_(t-1) x, odometer_t y - odometer_(t-1) y]
        u_t_control = np.array(
            [self.odometer.get_position()[0] - self.prior_odometer_position[0],
             self.odometer.get_position()[1] - self.prior_odometer_position[1]]
        )

        # curr_dat['theta_sensor'] - prior_dat['theta_sensor']])

        # r_control = 10e-2
        # Odometer
        J_control = np.array([
            [1, 0],
            [0, 1]
        ])

        # r_sensor = 10e-2
        # GPS
        J_sensor = np.array([
            [1, 0],
            [0, 1]
        ])

        # What to plugin to alpha 1-4?
        # alpha_1, alpha_2, alpha_3, alpha_4 = 10e-7, 15e-7, 5e-7, 20e-7
        # alphas are for gps
        # gps uniform(-.1, .1)
        # q_alphas are for odometer, what is a calculation for these?

        alpha_sensor_1, alpha_sensor_2 = (.05,) * 2
        alpha_odometer_1, alpha_odometer_2 = (.01 * self.odometer.get_velocity(),) * 2

        R = np.array([
            [alpha_sensor_1, 0],
            [0, alpha_sensor_2]
        ])

        Q = np.array([
            [alpha_odometer_1, 0],
            [0, alpha_odometer_2]
        ])

        # === Predict ===
        x_pred_control = self.position + u_t_control
        x_pred_sensor = self.position + u_t_sensor
        P_pred = J_control.dot(self.P).dot(J_control.T) + Q
        # === Update ===
        z_pred_control = self.H.dot(x_pred_control)
        z_pred_sensor = self.H.dot(x_pred_sensor)
        y = z_pred_sensor - z_pred_control
        S = J_sensor.dot(P_pred).dot(J_sensor.T) + R
        K = P_pred.dot(J_sensor.T).dot(np.linalg.inv(S))
        x_pred = x_pred_control + K.dot(y)

        # Review
        print(
            f"x_pred: {x_pred}"
            # # f"\nP {P}"
            # f"\ny: {y}"
            f"\n"
        )

        # Save to self
        self.position = [x_pred[0], x_pred[1]]
        self.prior_odometer_position = self.odometer.get_position()

    def get_position(self):
        return self.position


# noinspection PyPep8Naming
class LandmarkOdometerKf:
    def __init__(self, odometer, landmarks, **kwargs):
        """
        abuse of x in variable names and notations
        x (x, y) X (rx, ry, rth, lm1x, lm1y, ..., lmnx, lmny)
        X (capital x) for the correspondent kalman filter comprises all states

        :param: odometer: Odometer sensor class
        :param landmarks: Robotlandmarks sensor class
        :param kwargs:
         - position: 2 component x y representation
         - pose: 3 component x, y, theta representation (will use only x y here)
        """
        self.odometer = odometer
        self.landmarks = landmarks

        # X = (robot_x, robot_y, robot_theta, lm1_x, lm1_y, ..., lmn_x, lmn_y)
        # initialize position
        self.pose = [0, 0, 0]
        if 'init_pose' in kwargs:
            self.pose = kwargs['init_pose']
        self.S = len(self.pose)
        self.lmS = 2
        if "pose" in kwargs:
            self.pose = kwargs['pose']

        self.prior_odometer_position = self.odometer.get_position()

        # initialize matrices for kf calculation
        self.kf_p = np.eye(3 + len(self.landmarks) * 2)  # keep
        self.H = np.identity(3)  # keep
        # EKF state covariance
        self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2  # Change in covariance

    def update(self):
        """Update - Performs the Kalman filter update

         This is to be called from running robot event loop and operates
         by first grabbing the updated motion model position of the robot from
         the odometer class. This is the predict step and done with kf_predict.
         Next the each landmark is evaluated for an innovation and gain to
         update the robot position

         Completes with inplace update of self.pose
        """
        # kf_x = np.vstack((odometer_position, *[self.landmarks.landmark_positions]))
        kf_x = np.hstack((self.pose, self.landmarks.get_landmark_positions().flatten())).T
        # Predict
        # kf_x, kf_p, G, Fx = self.kf_predict(kf_x, u)
        kf_x, kf_p, G = self.kf_predict(kf_x)

        # Update
        kf_x, kf_p = self.kf_update(kf_x, kf_p)

        # Review
        print(
            f"x_pred: {kf_x[0:3]}"
            # # f"\nP {P}"
            # f"\ny: {y}"
            # f"\n"
        )

        # Save to self
        self.pose = kf_x[0:3]
        self.kf_p[0:3, 0:3] = kf_p[0:3, 0:3]

    def kf_predict(self, kf_x):
        """
        Performs the prediction of state based upon odometery motion model

        :param kf_x: update to position known from controls, nx1 state vector
         nx1 state vector representing X (pose, lmxy1, ..., lmxyn)
        # :param kf_p: nxn covariance matrix - COMING FROM SELF
        # :param u:    2x1 control vector
        :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
        """
        # Retrieve odometry update, (x, y, theta)
        odometer_position = np.array(self.odometer.get_position())

        # u_t_control is the update to odometery motion model at time t (now)
        u_t_control = np.array(
            [odometer_position[0] - self.prior_odometer_position[0],
             odometer_position[1] - self.prior_odometer_position[1],
             pi_mod4q(odometer_position[2] - self.prior_odometer_position[2])]
        )

        # update prior odometer position
        self.prior_odometer_position = odometer_position

        # predict state update
        kf_x[0:3] = kf_x[0:3] + u_t_control
        kf_x[2] = pi_mod4q(kf_x[2])

        # predict covariance
        # P = G.T @ P @ G + Cx
        d = np.sqrt(u_t_control[0] ** 2 + u_t_control[1] ** 2)

        J_control = np.array([
            [0, 0, .25 * d * np.sin(kf_x[2])],  # delta_time * u[0] * sin(kf_x[2])
            [0, 0, .25 * d * np.cos(kf_x[2])],  # delta_time * u[0] * sin(kf_x[2])
            [0, 0, 0]
        ])

        G = np.eye(self.S) + J_control

        # sigma = G*sigma*G.T + Noise
        # Upper left corner of covariance is updated
        kf_p = self.kf_p
        kf_p[0:3, 0:3] = G.T @ kf_p[0:3, 0:3] @ G + self.Cx
        return kf_x, kf_p, G

    def kf_update(self, kf_x, kf_p):
        """Performs the update step of EKF SLAM

        :param kf_x:  nx1 the predicted pose of the system and the pose of the landmarks
        :param kf_p:  nxn the predicted covariance
        # :param u:     2x1 the control function
        # :param z:     the measurements read at new position
        :returns:     the updated state and covariance for the system
        """
        # TODO visibility update
        lm_ranges = self.landmarks.landmark_ranges()
        n_lm = len(lm_ranges)

        for i_lm in range(len(lm_ranges[:, 0])):  # for each landmark
            # minid = search_correspond_LM_ID(xEst, PEst, z[iz, 0:2])  # associate to a known landmark

            # nLM = len(self.landmarks)
            #
            # if minid == nLM:  # Landmark is a NEW landmark
            #     print("New LM")
            #     # Extend state and covariance matrix
            #     xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz, :])))
            #     PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
            #                       np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            #     xEst = xAug
            #     PEst = PAug

            lm_x = self.kf_get_lm_position_from_state(kf_x, i_lm)
            print(f"i: {i_lm}, lm_x: {lm_x}")
            # y, S, H = calc_innovation(lm_x, xEst, PEst, z[iz, 0:2], minid)
            innovation, H, S = self.kf_calculate_innovation_polar(lm_x, lm_ranges[i_lm, 0:2], kf_x[0:3], kf_p, i_lm)
            # print(
            #     f"p {kf_p.shape}\n"
            #     f"H {H.shape}\n"
            #     f"S {S.shape}\n"
            # )

            K = (kf_p @ H.T) @ np.linalg.inv(S)  # Calculate kalman Gain
            print(
                # f"K {K}\n"
                f"K @ innovation {(K @ innovation)[0:3]}\n"
                f"est x {kf_x[0:3]}"
            )
            # k_innov = (K @ innovation)[0:3]
            # k_innov[2] = innovation[1]
            # kf_x[0:3] = kf_x[0:3] + k_innov[0:3]
            kf_x[0:3] = kf_x[0:3] + (K @ innovation)[0:3]
            kf_p = (np.eye(len(kf_x)) - (K @ H)) @ kf_p
            kf_x[2] = pi_mod4q(kf_x[2])

        return kf_x, kf_p

    @staticmethod
    def kf_get_lm_position_from_state(kf_x, iz):
        """
            (a, a, a, b, b, c, c, d, d, e, e)
        :return: lm_x (x, y) of the landmark to be returned from kf_x
        """
        return kf_x[3+iz*2:5+iz*2]

    @staticmethod
    def kf_calculate_innovation_pose(lm_x, lm_range, pose):
        """Calculate difference is position between current odometery pose
        and that working back from observed landmark and its known position

        :pose: (x, y, theta) for robot a.k.a. kf_x[0:3]
        :lm_x: (x, y) for landmark (absolute)
        :lm_range: (d, theta) for landmark (observed)
        :return: (dx, dy)
        """
        u = (
            (lm_x[0] - lm_range[0] * np.cos(pose[2] + lm_range[1])),
            (lm_x[1] - lm_range[0] * np.sin(pose[2] + lm_range[1]))
        )
        innovation = (
            (u[0] - pose[0]),
            (u[1] - pose[1])
        )
        print(
            f"lm range {lm_range}\n"
            f"lm robot pos (x,y) {u}\n"
            f"lmrp calc {lm_x[0]} - {lm_range[0]} * {np.cos(pose[2] + lm_range[1])}\n"
            f"prior est (x, y, theta) {pose[0:3]}\n"
            f"innovation {innovation}"
        )
        return innovation

    def kf_calculate_innovation_polar(self, lm_x, lm_range, pose, kf_p, i_lm):
        """Calculate difference is position between current odometery pose
        and that working back from observed landmark and its known position

        :pose: (x, y, theta) for robot a.k.a. kf_x[0:3]
        :lm_x: (x, y) for landmark (absolute)
        :lm_range: (d, theta) for landmark (observed)
        :return: (dx, dy)
        """
        est_x = (
            lm_x[0] - pose[0],
            lm_x[1] - pose[1]
        )
        est_theta = np.arctan2(est_x[1], est_x[0])
        est_polar = (
            np.sqrt(est_x[0] ** 2 + est_x[1] ** 2),
            pi_mod4q(est_theta - pose[2])
        )

        innov = np.array((
            lm_range[0] - est_polar[0],
            pi_mod4q(lm_range[1] - est_polar[1])
        ))

        q = est_polar[0]
        # print(f"q {q}")
        sq = np.sqrt(est_polar[0])
        G = np.array([[-sq * est_x[0], - sq * est_x[1], 0, sq * est_x[0], sq * est_x[1]],
                      [est_x[1], - est_x[0], -q, - est_x[1], est_x[0]]])
        G = G / q

        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * len(self.landmarks)))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * i_lm)),
                        np.eye(2), np.zeros((2, 2 * len(self.landmarks) - 2 * (i_lm + 1)))))

        F = np.vstack((F1, F2))

        H = G @ F
        # H = G / lm_range[0]
        # print(
        #     f"G {G.shape}\n"
        #     f"F {F.shape}\n"
        #     f"H {H.shape}\n"
        #     f"kf_p {kf_p.shape}\n"
        # )
        S = H @ kf_p @ H.T + np.diag([0.25, 0.25])

        print(
            f"obs polar {lm_range}\n"
            f"est polar (x,y) {est_polar}\n"
            f"innovation {innov}\n"
            f"H {H}\n"
            f"G {G}\n"
            f"S {S}\n"
        )
        return innov, H, S

    # Demos - Ideal List
    # [DONE] A Star Path Planning (assume map)
    # [DONE] Localization with known correspondences (assume map) (Highest Priority)
    # [DONE] Known Warehouse map for assuming in demos with proper shelve sizing
    # [DONE] Mapping (assume localization)
    # [NOW] Q-Learning and Mapping Followups
    #   - Create map class with grid map
    #   - Proximity to write points to grid s.t. when we add points together we do not duplicate
    #   - Review fine adjustments for proximity sensors
    #   - [Blocked by Map class] Q-Learning combined with AStar
    # Incorporate CNN into visible landmarks
    # [NEXT - Saturday] Simultaneous localization and mapping
    # Add a landmark mid routine or at start
    # Navigate and discover fire
    # Flask Server (Lowest Priority) (Next 4)

    # [NOW - PAPER] Accuracy Reporting - Confusion matrix for CNN
    # [NOW - PAPER] Accuracy Reporting - Localization Accuracy with hyper-parameters

    # Record Videos
    # A Star Path Planning
    # Localization with known correspondences
    # Mapping
    # Record SLAM without PP (Saturday) Localization and Mapping
