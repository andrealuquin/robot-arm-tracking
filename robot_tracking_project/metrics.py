import numpy as np


def tracking_error(current_position, target_position):
    """Return the Euclidean distance between current and target 3D positions."""
    current = np.asarray(current_position, dtype=float)
    target = np.asarray(target_position, dtype=float)
    return float(np.linalg.norm(target - current))


def position_error_vector(current_position, target_position):
    """Return the vector from current position to target position."""
    current = np.asarray(current_position, dtype=float)
    target = np.asarray(target_position, dtype=float)
    return target - current


def yoshikawa_manipulability(J):
    """Return the Yoshikawa manipulability measure: sqrt(det(J @ J.T)).

    Clamps small negative determinant values (from numerical error) to 0.
    """
    J = np.asarray(J, dtype=float)
    det_value = np.linalg.det(J @ J.T)
    return float(np.sqrt(max(det_value, 0.0)))
