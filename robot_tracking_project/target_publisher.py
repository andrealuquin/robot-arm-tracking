import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TargetPublisher(Node):
    
    
    #The targets are used as experimental cases for the controller.

    def __init__(self):
        super().__init__('target_publisher')

        # publisher for desired end-effector target position
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)

        # Experimental target cases:
        # 1. normal_case: baseline reachable target for standard tracking
        # 2. shifted_case: a second reachable target for comparison
        # 3. stretched_case: a farther target intended to encourage a more
        #    extended arm posture, which may produce more difficult or
        #    near-singular behavior
        self.targets = [
            ("normal_case", [0.6, 0.2, 0.8]),
            ("shifted_case", [0.5, 0.1, 0.75]),
            ("stretched_case", [1.15, 0.0, 0.5]),
        ]

        self.index = 0

        
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('TargetPublisher started.')

    def timer_callback(self):
      
        if self.index >= len(self.targets):
            self.get_logger().info('All targets published.')
            self.timer.cancel()
            return

        # Gets the current target label and coordinates
        label, coords = self.targets[self.index]

      
        msg = Point()
        msg.x = coords[0]
        msg.y = coords[1]
        msg.z = coords[2]

        self.publisher_.publish(msg)

        # Log which experimental target was sent
        self.get_logger().info(
            f'Published target {self.index + 1}/{len(self.targets)}: '
            f'{label} -> ({msg.x}, {msg.y}, {msg.z})'
        )

        # Move to the next target for the next timer callback
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()