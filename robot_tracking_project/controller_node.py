import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from robot_tracking_project.kinematics import forward_kinematics, jacobian
from robot_tracking_project.metrics import yoshikawa_manipulability


def pseudoinverse_control(J, error, gain=1.0):
    return np.linalg.pinv(J) @ (gain * error)


def dls_control(J, error, damping=0.1, gain=1.0):
    identity = np.eye(J.shape[0])
    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * identity) @ (gain * error)

def run_controller( mode='pinv', link_lengths=None, q_init=None, target=None, gain=1.0, 	 damping=0.1, dt=0.05, tolerance=0.01,max_steps=200,):
	if link_lengths is None:
        link_lengths = np.array([0.5, 0.7, 0.5], dtype=float)
    else:
        link_lengths = np.asarray(link_lengths, dtype=float)

    if q_init is None:
        q = np.array([0.2, 0.3, -0.2], dtype=float)
    else:
        q = np.asarray(q_init, dtype=float)

    if target is None:
        target = np.array([0.6, 0.2, 0.8], dtype=float)
    else:
        target = np.asarray(target, dtype=float)

    if q.shape != (3,):
        raise ValueError("q_init must be a length-3 vector.")
    if link_lengths.shape != (3,):
        raise ValueError("link_lengths must be a length-3 vector.")
    if target.shape != (3,):
        raise ValueError("target must be a length-3 vector.")
    if dt <= 0:
        raise ValueError("dt must be positive.")
    if tolerance <= 0:
        raise ValueError("tolerance must be positive.")
    if max_steps <= 0:
        raise ValueError("max_steps must be positive.")
    if damping <= 0:
        raise ValueError("damping must be positive for DLS.")
        
    positions = []
    errors = []
    q_history = [q.copy()]
    qdot_history = []

    for step in range(max_steps):
        x = forward_kinematics(q, link_lengths)
        error = target - x
        error_norm = np.linalg.norm(error)
        J = jacobian(q, link_lengths)

        positions.append(x.copy())
        errors.append(error_norm)

        if error_norm < tolerance:
            print(f"Target reached in {step} steps.")
            break

            if mode == 'pinv':
                q_dot = pseudoinverse_control(J, error, gain=self.gain)
            else:
                q_dot = dls_control(J, error, damping=self.damping, gain=self.gain)

            qdot_norm = np.linalg.norm(q_dot)
            if qdot_norm > max_qdot_norm:
                max_qdot_norm = qdot_norm

            q = q + q_dot * self.dt

            print(
                f"Step {step:03d} | "
                f"pos={x} | "
                f"error_norm={error_norm:.6f} | "
                f"manipulability={manipulability:.6f} | "
                f"q_dot={q_dot}"
            )

        print(
            f'\n{label} summary | '
            f'steps={step} | '
            f'final_error={final_error:.6f} | '
            f'max_qdot_norm={max_qdot_norm:.6f} | '
            f'final_manipulability={final_manipulability:.6f} | '
            f'final_pos={final_position}'
        )

        return {
            'final_error': final_error,
            'final_position': final_position,
            'max_qdot_norm': max_qdot_norm,
            'final_manipulability': final_manipulability,
            'steps': step,
        }


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()