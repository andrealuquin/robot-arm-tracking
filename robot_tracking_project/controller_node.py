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


class ControllerNode(Node):
    def __init__(self,
                 link_lengths=None,
                 q_init=None,
                 gain=1.0,
                 damping=0.1,
                 dt=0.05,
                 tolerance=0.01,
                 max_steps=200):
        super().__init__('controller_node')

        self.q_init = (
            np.array([0.2, 0.3, -0.2], dtype=float)
            if q_init is None
            else np.asarray(q_init, dtype=float)
        )
        self.link_lengths = (
            np.array([0.5, 0.7, 0.5], dtype=float)
            if link_lengths is None
            else np.asarray(link_lengths, dtype=float)
        )
        self.gain = gain
        self.damping = damping
        self.dt = dt
        self.tolerance = tolerance
        self.max_steps = max_steps

        self.target = None

        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self._target_callback,
            10,
        )

        self.get_logger().info('ControllerNode started. Waiting for /target_position...')

    def _target_callback(self, msg):
        new_target = np.array([msg.x, msg.y, msg.z], dtype=float)
        if self.target is not None and np.allclose(new_target, self.target):
            return
        self.target = new_target
        self.get_logger().info(f'New target received: {new_target}. Running both controllers.')
        pinv_error, pinv_pos = self._run_controller('pinv', self.target)
        dls_error, dls_pos = self._run_controller('dls', self.target)
        print(
            f'\nFinal comparison:\n'
            f'Pseudoinverse final position: {pinv_pos}\n'
            f'Pseudoinverse final error: {pinv_error:.6f}\n'
            f'DLS final position: {dls_pos}\n'
            f'DLS final error: {dls_error:.6f}'
        )

    def _run_controller(self, mode, target):
        label = 'Pseudoinverse' if mode == 'pinv' else 'DLS'
        print(f'\n--- {label} Controller ---')
        q = self.q_init.copy()
        final_error = None
        final_position = None

        for step in range(self.max_steps):
            x = forward_kinematics(q, self.link_lengths)
            error = target - x
            error_norm = np.linalg.norm(error)
            J = jacobian(q, self.link_lengths)
            manipulability = yoshikawa_manipulability(J)

            final_error = error_norm
            final_position = x.copy()

            if error_norm < self.tolerance:
                print(
                    f"Step {step:03d} | "
                    f"pos={x} | "
                    f"error_norm={error_norm:.6f} | "
                    f"manipulability={manipulability:.6f}"
                )
                break

            if mode == 'pinv':
                q_dot = pseudoinverse_control(J, error, gain=self.gain)
            else:
                q_dot = dls_control(J, error, damping=self.damping, gain=self.gain)

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
            f'final_pos={final_position}'
        )
        return final_error, final_position


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

