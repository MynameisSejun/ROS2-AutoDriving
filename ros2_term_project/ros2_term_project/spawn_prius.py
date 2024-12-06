import os

import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
import tf_transformations as tf


class PriusSpawner(Node):
    def __init__(self):
        super().__init__('prius_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /spawn_entity service...')

    def spawn(self, car_name, model_file, x_pos, y_pos, yaw_angle):
        pose = Pose()
        pose.position.x = x_pos
        pose.position.y = y_pos
        pose.position.z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = yaw_angle
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        request = SpawnEntity.Request()
        request.name = car_name  # 고유한 자동차 이름 사용
        request.xml = open(model_file).read()  # 모델 파일 경로
        request.robot_namespace = "/"
        request.initial_pose = pose
        future = self.spawn_client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    spawner = PriusSpawner()

    package_share_directory = get_package_share_directory('ros2_term_project')
    sdf_path = os.path.join(package_share_directory, 'worlds', 'model1.sdf')
    sdf_path2 = os.path.join(package_share_directory, 'worlds', 'model2.sdf')

    # PR001 자동차 소환
    future1 = spawner.spawn('PR001', sdf_path, 93.0, -11.9, -1.57)
    rclpy.spin_until_future_complete(spawner, future1)
    if future1.result() is not None:
       spawner.get_logger().info('Successfully spawned PR001!')
    else:
       spawner.get_logger().info('Failed to spawn PR001.')

    # PR002 자동차 소환
    future2 = spawner.spawn('PR002', sdf_path2, 93.0, -15.9, -1.57)
    rclpy.spin_until_future_complete(spawner, future2)
    if future2.result() is not None:
        spawner.get_logger().info('Successfully spawned PR002!')
    else:
        spawner.get_logger().info('Failed to spawn PR002.')

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
