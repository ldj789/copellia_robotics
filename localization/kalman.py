import time
import numpy as np


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
        self.S = len(self.pose)
        self.lmS = 2
        if "pose" in kwargs:
            self.pose = kwargs['pose']

        self.prior_odometer_position = self.odometer.get_position()

        # initialize matrices for kf calculation
        self.P = np.identity(2)  # keep
        self.H = np.identity(2)  # keep

    def update(self):
        # Retrieve odometry update, (x, y, theta)
        odometer_position = np.array(self.odometer.get_position())
        # kf_x = np.vstack((odometer_position, *[self.landmarks.landmark_positions]))
        kf_x = np.hstack((odometer_position, *[self.landmarks.get_landmark_positions()])).T
        u_t_control = np.array(
            [odometer_position[0] - self.prior_odometer_position[0],
             odometer_position[1] - self.prior_odometer_position[1]]
        )
        # Predict
        x_pred, PEst, G, Fx = self.kf_predict(u_t_control, PEst, u)
        kf_x[0:3] = x_pred
        # Update
        # x_pred, PEst = self.kf_update(x_pred, PEst)

        # Review
        print(
            f"x_pred: {x_pred}"
            # # f"\nP {P}"
            # f"\ny: {y}"
            # f"\n"
        )

        # Save to self
        self.pose = [x_pred[0], x_pred[1], self.pose[2]]
        self.prior_odometer_position = odometer_position
        # return x_pred, PEst

    def kf_predict(self, u_t_control):
        """
        Performs the prediction of state based upon odometery motion model

        :param u_t_control: update to position known from controls, nx1 state vector
        # :param xEst: nx1 state vector
        # :param PEst: nxn covariacne matrix
        # :param u:    2x1 control vector
        :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
        """
        # predict state update
        x_pred_control = self.pose + u_t_control
        # X = x_pred_control.vstack(self.landmarks.landmark_ranges_map())

        # predict covariance
        # P = G.T @ P @ G + Cx
        Fx = np.hstack(
            (np.eye(self.S),
             np.zeros((self.S, self.lmS * len(self.landmarks))))
        )
        J_control = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])

        G = np.eye(self.S) + Fx.T @ J_control @ Fx

        # xEst[0:S] = motion_model(xEst[0:S], u)
        # Fx is an an identity matrix of size (STATE_SIZE)
        # sigma = G*sigma*G.T + Noise
        # Upper left corner of covariance is updated
        PEst[0:self.S, 0:self.S] = G.T @ PEst[0:self.S, 0:self.S] @ G + Fx.T @ Cx @ Fx
        return x_pred_control, PEst, G, Fx

    def kf_update(self, kf_x, PEst):
        """
            Performs the update step of EKF SLAM

            :param kf_x:  nx1 the predicted pose of the system and the pose of the landmarks
            :param PEst:  nxn the predicted covariance
            :param u:     2x1 the control function
            :param z:     the measurements read at new position
            :returns:     the updated state and covariance for the system
            """
        # TODO: Create initP: 2x2 an identity matrix acting as the initial covariance
        n_v = self.landmarks.visible_landmarks()
        z = self.landmarks.landmark_ranges()
        # TODO n_v becomes length of z after visibility update
        initP = np.eye(len(n_v))

        for iz in range(len(z[:, 0])):  # for each landmark
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

            # TODO
            #  1) Return specific landmark state
            #  2) Calculate innovation

            lm_x = get_LM_Pos_from_state(kf_x, iz)
            y, S, H = calc_innovation(lm_x, xEst, PEst, z[iz, 0:2], minid)

            K = (PEst @ H.T) @ np.linalg.inv(S)  # Calculate Kalman Gain, to start 0.5
            xEst = xEst + (K @ y)
            PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

        xEst[2] = pi_2_pi(xEst[2])
        return xEst, PEst
