import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(Point, '/target_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Point()
        msg.x = 0.6
        msg.y = 0.2
        msg.z = 0.8
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
