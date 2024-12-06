import rclpy
from nav_msgs.msg import Odometry
from rclpy import time
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, car_name: str):
        super().__init__(f'{car_name}_line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self.current_y_position = 0.0
        self.line_tracker.stop_callback = self.stop

        # 구독자: 카메라 이미지
        self._subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        # 구독자: LiDAR 데이터 (SDF에 정의된 LiDAR와 동일한 토픽)
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan_custom',  # SDF에서 설정한 LiDAR 데이터 토픽 이름
            self.lidar_callback,
            10
        )

        # 퍼블리셔: 자동차 제어 명령
        self._publisher = self.create_publisher(Twist, f'/cmd_demo', 1)

        self.twist = Twist()
        self.twist.linear.x = 5.0  # 기본 속도: 5 m/s
        self.img = None
        self.obstacle_detected = False  # 장애물 감지 상태
        self.stopped = False

        self.stop_duration = 5.0  # 정지 후 대기 시간 (초)

    def image_callback(self, msg: Image):
        if self.stopped:
            # 차량이 정지 상태라면 데이터를 무시
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.line_tracker.process(img)

            if self.line_tracker.stop_detected:
               self.line_stop()

            # 조향 조정
            self.twist.angular.z = (-1) * self.line_tracker.delta / 110  # 조향 비율 조정
            self._publisher.publish(self.twist)

        except Exception as e:
            raise

    def lidar_callback(self, msg: LaserScan):
        try:
            range_min = 0.5
            range_max = 8.0

            is_obstacle = any(range_min <= distance <= range_max for distance in msg.ranges if distance > 0)

            # self.get_logger().info(f"stop_detected={self.line_tracker.stop_detected}")
            if not self.line_tracker.stop_detected:
                if is_obstacle:
                    self.obstacle_detected = True
                    self.stop()
                else:
                    self.obstacle_detected = False
                    self.resume()

        except Exception as e:
            raise

    def stop(self):
        # 차량 정지 명령 발행
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle stop")

    def resume(self):
        # 차량 정지 해제 후 속도 재설정
        self.stopped = False
        self.twist.linear.x = 5.0  # 원래 설정된 속도로 복귀
        self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle run")

    def line_stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)
        self.get_logger().info("Vehicle stopped for 3 seconds.")
        self.stopped = True  # 정지 상태로 전환

        # 3초 후 resume 호출
        self.create_timer(5.0, self.resume)

    def resume_driving(self):
        self.get_logger().info("Resuming driving.")
        self.resume()
        self.stop_timer.cancel()  # 타이머 취소

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
