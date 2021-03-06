import time
import numpy as np


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

    # noinspection PyPep8Naming
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
