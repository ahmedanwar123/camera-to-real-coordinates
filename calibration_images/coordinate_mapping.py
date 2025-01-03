import numpy as np
import cv2


class CoordinateMapping:
    def __init__(self, camera_matrix, dist, rvecs, tvecs):
        self.camera_matrix = camera_matrix
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs

    def pixel_to_world(self, pixel_coords):
        object_points = np.array(
            [[pixel_coords[0], pixel_coords[1], 0]], dtype=np.float32
        )
        rotation_matrix = cv2.Rodrigues(self.rvecs)[0]  # Get only the rotation matrix
        world_point = np.dot(rotation_matrix, object_points.T).T + self.tvecs
        return world_point.flatten()[:2]  # Return only x, y coordinates
