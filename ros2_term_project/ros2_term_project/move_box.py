import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class BoxMover(Node):
    def __init__(self):
        super().__init__('move_box')
        self.command_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_update_callback,
            10
        )
        self.create_timer(0.1, self.publish_movement_command)
        self.movement_direction = 1
        self.twist_command = Twist()
        self.current_y_position = None
        self.y_boundary_lower = -78
        self.y_boundary_upper = -62

    def odom_update_callback(self, odom_data: Odometry):
        self.current_y_position = odom_data.pose.pose.position.y
        if self.movement_direction == 1 and self.current_y_position > self.y_boundary_upper:
            self.movement_direction = -1
        elif self.movement_direction == -1 and self.current_y_position < self.y_boundary_lower:
            self.movement_direction = 1

    def publish_movement_command(self):
        self.twist_command.linear.y = 2.0 * self.movement_direction
        self.command_publisher.publish(self.twist_command)


def main(args=None):
    rclpy.init(args=args)
    box_mover_node = BoxMover()
    rclpy.spin(box_mover_node)
    box_mover_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()