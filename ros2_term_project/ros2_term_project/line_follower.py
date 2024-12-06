import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, car_name: str):
        super().__init__(f'{car_name}_line_follower')
        self.car_name = None
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self.current_y_position = 0.0
        self.line_tracker.stop_callback = self.stop

        # 구독자: 차량 시작 명령
        self.get_car = self.create_subscription(
            String,
            'start_car',
            self.get_car_callback,
            10
        )

        self._publisher = None
        self.twist = Twist()
        self.twist.linear.x = 5.0  # 기본 속도: 5 m/s
        self.img = None
        self.obstacle_detected = False  # 장애물 감지 상태
        self.stopped = False
        self.stop_bool = True

    def get_car_callback(self, msg: String):
        self.car_name = msg.data.strip()
        self.get_logger().info(f"Received car name: {self.car_name}")

        # 퍼블리셔 및 구독자 동적 설정
        if self.car_name in ['PR001', 'PR002']:
            self._publisher = self.create_publisher(
                Twist, f'/cmd_demo_{self.car_name}', 1
            )
            self.get_logger().info(f"Publisher created for: /cmd_demo_{self.car_name}")

            self._subscription = self.create_subscription(
                Image,
                f'/camera_{self.car_name}/image_raw',
                self.image_callback,
                10
            )
            self.get_logger().info(f"Subscribed to: /camera_{self.car_name}/image_raw")

            self._lidar_subscription = self.create_subscription(
                LaserScan,
                f'/scan_{self.car_name}',
                self.lidar_callback,
                10
            )
            self.get_logger().info(f"Subscribed to: /scan_{self.car_name}")
        else:
            self.get_logger().warn(f"Unknown car name: {self.car_name}")

    def image_callback(self, msg: Image):
        if self._publisher is None:
            self.get_logger().warn("Publisher not initialized. Ignoring image data.")
            return

        if self.line_tracker.end_detected:
            self.get_logger().info("End detected: Stopping vehicle permanently.")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self._publisher.publish(self.twist)
            self.stopped = True
            return

        if self.stopped:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.line_tracker.process(img)

            if self.line_tracker.stop_detected and self.stop_bool:
                self.line_stop()
                self.stop_bool = False
                self.create_timer(7.0, self.stop_bool_true)

            self.twist.angular.z = (-1) * self.line_tracker.delta / 110  # 조향 비율 조정
            self._publisher.publish(self.twist)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
            raise

    def lidar_callback(self, msg: LaserScan):
        if self._publisher is None:
            self.get_logger().warn("Publisher not initialized. Ignoring LiDAR data.")
            return

        try:
            if self.line_tracker.end_detected:
                return

            range_min = 0.5
            range_max = 8.0

            is_obstacle = any(range_min <= distance <= range_max for distance in msg.ranges if distance > 0)

            if not self.line_tracker.stop_detected:
                if is_obstacle:
                    self.obstacle_detected = True
                    self.stop()
                else:
                    self.obstacle_detected = False
                    self.resume()

        except Exception as e:
            self.get_logger().error(f"Error in lidar_callback: {e}")
            raise

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        if self._publisher:
            self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped.")

    def resume(self):
        if self.line_tracker.end_detected:
            self.get_logger().info("Resume skipped: End detected.")
            return

        self.stopped = False
        self.twist.linear.x = 5.0
        if self._publisher:
            self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle running.")

    def line_stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        if self._publisher:
            self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped for 3 seconds.")
        self.stopped = True

        self.create_timer(5.0, self.resume)

    def stop_bool_true(self):
        self.stop_bool = True
        self.get_logger().info("Stop_bool reset to True.")

    @property
    def publisher(self):
        return self._publisher


def main():
    rclpy.init()
    car_name = 'PR002'

    tracker = LineTracker()
    follower = LineFollower(tracker, car_name)

    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
