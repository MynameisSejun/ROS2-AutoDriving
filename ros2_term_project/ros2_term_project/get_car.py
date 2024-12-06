import rclpy
import sys
from rclpy.node import Node

from std_msgs.msg import String


class CarStarter(Node):

    def __init__(self):
        super().__init__('starter')
        # 지정된 차량 정보를 전달하는 publisher
        self.car_info_publisher_ = self.create_publisher(String, 'start_car', 10)
        self.car = sys.argv[1]
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        # 지정된 차량 정보가 올바른지 확인
        if self.car == 'PR001' or self.car == 'PR002':
            msg.data = self.car
            self.car_info_publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            self.get_logger().error('"%s": 해당 차량이 존재하지 않습니다.' % self.car)
        return


def main(args=None):
    rclpy.init(args=args)

    starter = CarStarter()

    rclpy.spin_once(starter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()