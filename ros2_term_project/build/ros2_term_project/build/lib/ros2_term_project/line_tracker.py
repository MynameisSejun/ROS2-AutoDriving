import cv2
import numpy as np


class LineTracker:
    def __init__(self):
        self._delta = 0.0

    def process(self, img: np.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])  # 흰색 범위
        upper_white = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, _ = img.shape
        search_top = int(h / 3 * 1.5)
        mask[0:search_top, 0:w] = 0  # 상단 부분 마스킹

        # 차선의 모멘트 계산
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # 차선 중심에 점 찍기
            cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)  # 인식된 지점을 노란색으로 그림

        # 왼쪽 및 오른쪽 차선 마스크
        left_mask = mask[:, :w // 2]  # 왼쪽 절반
        right_mask = mask[:, w // 2:]  # 오른쪽 절반

        # 왼쪽 차선 모멘트
        left_M = cv2.moments(left_mask)
        left_cx = int(left_M['m10'] / left_M['m00']) if left_M['m00'] > 0 else -1

        # 오른쪽 차선 모멘트
        right_M = cv2.moments(right_mask)
        right_cx = int(right_M['m10'] / right_M['m00']) + (w // 2) if right_M['m00'] > 0 else -1

        # 왼쪽 차선 중심 시각화
        if left_cx != -1:
            cv2.circle(img, (left_cx, cy), 5, (255, 0, 0), -1)  # 왼쪽 차선에 점 찍기

        # 오른쪽 차선 중심 시각화
        if right_cx != -1:
            cv2.circle(img, (right_cx, cy), 5, (0, 0, 255), -1)  # 오른쪽 차선에 점 찍기

        # 중앙 조정 로직
        if left_cx != -1 and right_cx != -1:
            lane_center = (left_cx + right_cx) / 2
            self._delta = lane_center - (w / 2)

        # 결과 이미지 및 마스크 표시
        cv2.imshow("Lane Detection", img)
        cv2.imshow("Mask", mask)
        cv2.waitKey(3)

    @property
    def delta(self):
        return self._delta


def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('../worlds/sample2.png')
        tracker.process(img)
        time.sleep(0.1)


if __name__ == '__main__':
    main()
