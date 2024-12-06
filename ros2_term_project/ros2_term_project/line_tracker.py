import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node


class LineTracker(Node):
    def __init__(self, stop_callback=None):
        super().__init__('line_tracker')  # ROS2 노드 이름 설정
        self._delta = 0.0
        self._prev_left_cx = None  # 이전 왼쪽 차선 중심값
        self._prev_right_cx = None  # 이전 오른쪽 차선 중심값
        self.stop_detected = False # 정지 상태 플래그
        self.stop_callback = stop_callback

    def process(self, img: np.ndarray) -> None:
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # line
            lower_white = np.array([0, 0, 200])  # 흰색 범위
            upper_white = np.array([180, 30, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)

            # stop line
            lower_stop_line = np.array([0, 0, 200])
            upper_stop_line = np.array([180, 30, 255])
            stop_line_mask = cv2.inRange(hsv, lower_stop_line, upper_stop_line)

            # line ROI
            h, w, _ = img.shape
            search_top = int(h / 3 * 1.8)
            mask[0:search_top, 0:w] = 0  # 상단 부분 마스킹

            # stop_line ROI
            stop_line_top = int(h / 3 * 1.6)
            stop_line_bottom = int(h / 3 * 1.8)
            # search_left = int(w / 8 * 3)
            # search_right = int(w / 8 * 5)
            stop_line_mask[0:stop_line_top, :] = 0
            stop_line_mask[stop_line_bottom:h, :] = 0
            # stop_line_mask[:, 0:search_left] = 0
            # stop_line_mask[:, search_right:w] = 0

            # 정지선 흰색 픽셀 개수 계산
            white_pixel_count = cv2.countNonZero(stop_line_mask)
            # print(f"흰색 픽셀 개수: {white_pixel_count}")
            if white_pixel_count > 3000:
                self.stop_detected = True
            else:
                self.stop_detected = False  # 상태 초기화는 LineFollower에서 관리

            # 차선의 모멘트 계산
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(h / 4 * 3)

                # 차선 중심에 점 찍기
                cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)  # 인식된 지점을 노란색으로 그림
                self.get_logger().debug(f"Lane center detected at: ({cx}, {cy})")

            # 왼쪽 및 오른쪽 차선 마스크
            left_mask = mask[:, :w // 2]  # 왼쪽 절반
            right_mask = mask[:, w // 2:]  # 오른쪽 절반

            # 왼쪽 차선 모멘트
            left_M = cv2.moments(left_mask)
            left_cx = int(left_M['m10'] / left_M['m00']) if left_M['m00'] > 0 else None

            # 오른쪽 차선 모멘트
            right_M = cv2.moments(right_mask)
            right_cx = int(right_M['m10'] / right_M['m00']) + (w // 2) if right_M['m00'] > 0 else None

            # 이전 값을 대체
            if left_cx is None and self._prev_left_cx is not None:
                left_cx = self._prev_left_cx
            if right_cx is None and self._prev_right_cx is not None:
                right_cx = self._prev_right_cx

            # 중앙 조정 로직
            if left_cx is not None and right_cx is not None:
                lane_center = (left_cx + right_cx) / 2
                self._delta = lane_center - (w / 2)

            # 현재 값을 이전 값으로 저장
            self._prev_left_cx = left_cx
            self._prev_right_cx = right_cx

            # 결과 이미지 및 마스크 표시
            cv2.imshow("Lane Detection", img)
            cv2.imshow("Mask", mask)
            cv2.imshow("Stop Line Mask", stop_line_mask)
            cv2.waitKey(3)

        except Exception as e:
            self.get_logger().error(f"Error in process: {e}")
            raise

    @property
    def delta(self):
        return self._delta


def main(args=None):
    rclpy.init(args=args)
    tracker = LineTracker()
    for i in range(100):
        try:
            img = cv2.imread('../worlds/sample.jpeg')
            if img is None:
                tracker.get_logger().error("Failed to load image. Check the file path.")
                break
            tracker.process(img)
            time.sleep(0.1)
        except Exception as e:
            tracker.get_logger().error(f"Exception in main loop: {e}")
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
