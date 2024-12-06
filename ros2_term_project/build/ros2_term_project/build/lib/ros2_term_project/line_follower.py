import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, car_name: str):
        super().__init__(f'{car_name}_line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()

        self._subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        self._publisher = self.create_publisher(Twist, f'/PR001/cmd_vel', 10)  # 자동차 이름에 맞는 명령 토픽 설정
        self.twist = Twist()
        self.twist.linear.x = 5.0  # 1m/sec
        self.img = None

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)

        # 조향 조정
        self.twist.angular.z = (-1) * self.line_tracker.delta / 430  # 조향 비율 조정
        # self.get_logger().info(f"angular.z = {self.twist.angular.z}")
        # self.get_logger().info(f"angular.x = {self.twist.linear.x}")
        self._publisher.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)

    @property
    def publisher(self):
        return self._publisher


def main():
    rclpy.init()
    # 런치 파일에서 'car_name' 파라미터로 자동차 이름을 받아옴
    car_name = 'PR001'  # 예시로 PR001 사용, 실제로는 launch 파일에서 동적으로 설정할 것

    tracker = LineTracker()
    follower = LineFollower(tracker, car_name)

    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop()

    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
