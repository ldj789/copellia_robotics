import numpy as np


class ProximityMap:
    def __init__(self, client_id, proximity_sensors):
        """
        :param client_id: client_id for Coppelia
        :param proximity_sensors: sensors.proximity.ProximitySensorP3DX
        """
        self.client_id = client_id
        self.proximity_sensors = proximity_sensors

    def update(self, current_pose, turning, threshold=0.033):
        """Update map bases on proximity sensors data

        If turning is above threshold then no mapping is preformed

        :param current_pose: Current robot pose (x, y, theta)
        :param turning: Turning rate from navigate class
        :param threshold: Turning threshold over which we do not map new points
        :return: obstacle distances
        """
        proximate_objects = []
        obstacle_distances = self.proximity_sensors.get_distances()
        if np.abs(turning) < threshold:
            proximate_objects = self.proximity_sensors.get_proximate_objects(current_pose=current_pose)

        self.integrate_objects(proximate_objects)
        # Tracking of elements
        # obstacle_positions.extend(proximate_objects)
        return proximate_objects, obstacle_distances

    def integrate_objects(self, proximate_objects):
        pass
