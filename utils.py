import numpy as np
from typing import Optional


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    """
    Normalize a vector to unit length.

    Args:
        vector: A numpy array representing the vector to normalize.

    Returns:
        A numpy array representing the normalized vector.
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm


def calculate_distance(point1: np.ndarray, point2: np.ndarray) -> float:
    """
    Calculate the Euclidean distance between two points.

    Args:
        point1: A numpy array representing the first point.
        point2: A numpy array representing the second point.

    Returns:
        A float representing the Euclidean distance.
    """
    return np.linalg.norm(point1 - point2)


def vector_to_homogeneous(vector: np.ndarray) -> np.ndarray:
    """
    Convert a 2D vector to homogeneous coordinates.

    Args:
        vector: A numpy array representing the 2D vector.

    Returns:
        A numpy array representing the vector in homogeneous coordinates.
    """
    return np.append(vector, 1)


def transform_coordinates(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """
    Transform a vector using a given transformation matrix (homogeneous transformation).

    Args:
        matrix: A 3x3 or 4x4 transformation matrix (2D or 3D).
        vector: A 2D or 3D vector to be transformed.

    Returns:
        A transformed vector (2D or 3D).
    """
    homogeneous_vector = vector_to_homogeneous(vector)
    transformed = np.dot(matrix, homogeneous_vector)
    return transformed[:2]  # return 2D point if 2D transformation
