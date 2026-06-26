import numpy as np


class PurePursuitController:
    """
    Pure Pursuit path tracking controller for a unicycle/bicycle-style robot.

    The controller selects a lookahead point on the path and computes the
    angular velocity needed to steer toward that point.
    """

    def __init__(self, lookahead_distance=0.8, linear_velocity=1.0):
        self.lookahead_distance = lookahead_distance
        self.linear_velocity = linear_velocity

    def find_lookahead_point(self, robot_position, path):
        distances = np.linalg.norm(path - robot_position, axis=1)

        candidate_indices = np.where(distances >= self.lookahead_distance)[0]

        if len(candidate_indices) == 0:
            return path[-1]

        nearest_index = np.argmin(distances)
        valid_indices = candidate_indices[candidate_indices >= nearest_index]

        if len(valid_indices) == 0:
            return path[-1]

        return path[valid_indices[0]]

    def update(self, state, path):
        x, y, heading = state
        robot_position = np.array([x, y])

        lookahead_point = self.find_lookahead_point(robot_position, path)

        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y

        target_angle = np.arctan2(dy, dx)
        heading_error = target_angle - heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        angular_velocity = (
            2.0
            * self.linear_velocity
            * np.sin(heading_error)
            / self.lookahead_distance
        )

        return self.linear_velocity, angular_velocity, lookahead_point
