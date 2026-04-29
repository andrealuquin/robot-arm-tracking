import numpy as np

from robot_tracking_project.kinematics import forward_kinematics, jacobian


def pseudoinverse_control(J, error, gain=1.0):
    return np.linalg.pinv(J) @ (gain * error)


def dls_control(J, error, damping=0.1, gain=1.0):
    identity = np.eye(J.shape[0])
    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * identity) @ (gain * error)


def main():
    link_lengths = [0.5, 0.7, 0.5]
    q = np.array([0.2, 0.3, -0.2], dtype=float)
    target = np.array([0.6, 0.2, 0.8], dtype=float)

    x = forward_kinematics(q, link_lengths)
    J = jacobian(q, link_lengths)
    error = target - x

    q_dot_pinv = pseudoinverse_control(J, error)
    q_dot_dls = dls_control(J, error)

    print("Current position:", x)
    print("Error:", error)
    print("Pseudoinverse q_dot:", q_dot_pinv)
    print("DLS q_dot:", q_dot_dls)
