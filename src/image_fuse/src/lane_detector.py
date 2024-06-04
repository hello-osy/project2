#!/usr/bin/env python3

'''
수정할 사항.
왼쪽차선 좌표2개 오른쪽 차선좌표 2개. 총 4개의 좌표를 토픽에 담아서 보내야함.

혹시 오류나면, 예전에 올린 버전으로 돌아갈 것.
'''

import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RoadLaneDetector:
    def __init__(self):
        self.poly_bottom_width = 0.85 #관심영역 선택할 때 필요한 값
        self.poly_top_width = 0.07 #관심영역 선택할 때 필요한 값
        self.poly_height = 0.4 #관심영역 선택할 때 필요한 값
        self.img_center = None
        self.left_detect = False
        self.right_detect = False
        self.left_m = None
        self.right_m = None
        self.left_b = None
        self.right_b = None

    def filter_colors(self, img_frame):
        img_hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)
        
        #어두운 흰색 차선을 감지하기 위한 흰색 범위 설정 (HSV)
        #색상, 채도, 명도
        lower_white = np.array([100, 0, 185]) 
        upper_white = np.array([225, 225, 255])
        white_mask = cv2.inRange(img_hsv, lower_white, upper_white)
        
        white_image = cv2.bitwise_and(img_frame, img_frame, mask=white_mask)
        
        #이거 나중에 지울 것
        # cv2.imshow("white_filtered", white_image)

        return white_image

    def limit_region(self, img_edges):
        height, width = img_edges.shape
        mask = np.zeros_like(img_edges)

        # 사다리꼴의 아랫부분 12%를 제외한 영역 설정
        lower_left = (0, int(height * 0.88))
        upper_left = (int(width * 0.2), height * 2 // 3)
        upper_right = (int(width * 0.8), height * 2 // 3)
        lower_right = (width, int(height * 0.88))

        points = np.array([[lower_left, upper_left, upper_right, lower_right]], dtype=np.int32)
        cv2.fillPoly(mask, points, 255)

        region_limited_image = cv2.bitwise_and(img_edges, mask)
        # 이거 나중에 지울 것
        # cv2.imshow("mask_region", mask)
        # cv2.imshow("region_limited", region_limited_image)
        return region_limited_image

    def hough_lines(self, img_mask):
        #입력 이미지, 거리 해상도, 각도 해상도, 직선으로 판단되기 위한 최소한의 투표 수, 검출된 직선의 최소 길이, 직선으로 간주할 최대 간격
        return cv2.HoughLinesP(img_mask, 1, np.pi / 180, 50, minLineLength=20, maxLineGap=30)

    def separate_lines(self, img_edges, lines):
        right_lines = []
        left_lines = []
        self.img_center = img_edges.shape[1] / 2

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
                if abs(slope) > 0.3:
                    if slope > 0 and x1 > self.img_center and x2 > self.img_center:
                        right_lines.append(line)
                        self.right_detect = True
                    elif slope < 0 and x1 < self.img_center and x2 < self.img_center:
                        left_lines.append(line)
                        self.left_detect = True

        return [right_lines, left_lines]

    def regression(self, separated_lines, img_input):
        output = [None] * 4
        right_points = []
        left_points = []

        if self.right_detect:
            for line in separated_lines[0]:
                for x1, y1, x2, y2 in line:
                    right_points.append((x1, y1))
                    right_points.append((x2, y2))

            if right_points:
                right_vx, right_vy, right_x, right_y = cv2.fitLine(np.array(right_points), cv2.DIST_L2, 0, 0.01, 0.01)
                self.right_m = right_vy / right_vx
                self.right_b = (right_x, right_y)

        if self.left_detect:
            for line in separated_lines[1]:
                for x1, y1, x2, y2 in line:
                    left_points.append((x1, y1))
                    left_points.append((x2, y2))

            if left_points:
                left_vx, left_vy, left_x, left_y = cv2.fitLine(np.array(left_points), cv2.DIST_L2, 0, 0.01, 0.01)
                self.left_m = left_vy / left_vx
                self.left_b = (left_x, left_y)

        y1 = img_input.shape[0]
        y2 = int(y1 * 0.6)

        if self.right_detect:
            right_x1 = int(((y1 - self.right_b[1]) / self.right_m) + self.right_b[0])
            right_x2 = int(((y2 - self.right_b[1]) / self.right_m) + self.right_b[0])
            output[0] = (right_x1, y1)
            output[1] = (right_x2, y2)

        if self.left_detect:
            left_x1 = int(((y1 - self.left_b[1]) / self.left_m) + self.left_b[0])
            left_x2 = int(((y2 - self.left_b[1]) / self.left_m) + self.left_b[0])
            output[2] = (left_x1, y1)
            output[3] = (left_x2, y2)

        return output

    def draw_line(self, img_input, lane):
        poly_points = np.array([lane[2], lane[0], lane[1], lane[3]], dtype=np.int32)
        overlay = img_input.copy()
        cv2.fillConvexPoly(overlay, poly_points, (0, 230, 30))
        cv2.addWeighted(overlay, 0.3, img_input, 0.7, 0, img_input)

        cv2.line(img_input, lane[0], lane[1], (0, 255, 255), 5)
        cv2.line(img_input, lane[2], lane[3], (0, 255, 255), 5)

        return img_input


def image_callback(msg, args):
    road_lane_detector, image_pub = args
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        img_filter = road_lane_detector.filter_colors(cv_image)
        img_gray = cv2.cvtColor(img_filter, cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(img_gray, 50, 150)
        img_mask = road_lane_detector.limit_region(img_edges)
        lines = road_lane_detector.hough_lines(img_mask)

        if lines is not None:
            separated_lines = road_lane_detector.separate_lines(img_mask, lines)
            lane = road_lane_detector.regression(separated_lines, cv_image)
            img_result = road_lane_detector.draw_line(cv_image, lane)
        else:
            img_result = cv_image

        #image_pub.publish(bridge.cv2_to_imgmsg(img_result, "bgr8"))
        msg = Float32MultiArray()
        msg.data = [lane[0][0], lane[1][0], lane[2][0], lane[3][0], lane[0][1], lane[1][1]] #lane은 4개의 점이다. 
        #right_x1, right_x2, left_x1, left_x2, y1, y2를 순서대로 보낸 것임.
        image_pub.publish(msg)

        #창 이름, 표시할 이미지
        #print("Showing result image")  # 디버깅 메시지 추가
        cv2.imshow("result", img_result) 

        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("ESC pressed")

    except CvBridgeError as e:
        rospy.logerr("cv_bridge exception: %s", e)


def main():
    rospy.init_node('road_lane_detector')
    road_lane_detector = RoadLaneDetector()

    bridge = CvBridge() #CvBridge로 ROS 이미지 메시지와 OpenCV 이미지를 왔다갔다 할 수 있다.
    first_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)
    cv_image = bridge.imgmsg_to_cv2(first_msg, "bgr8")

    #창 이름, 표시할 이미지
    #이거 나중에 지울 것
    #cv2.imshow("cv_input", cv_image) 

    codec = cv2.VideoWriter_fourcc(*'XVID')
    fps = 25.0
    filename = './result.avi'
    writer = cv2.VideoWriter(filename, codec, fps, (cv_image.shape[1], cv_image.shape[0]), True)

    if not writer.isOpened():
        print("Cannot save the video.")
        return -1

    #image_pub = rospy.Publisher('/lane_detector', Image, queue_size=10) 이 부분 수정해야 함.
    image_pub = rospy.Publisher('/lane_detector', Float32MultiArray, queue_size=10) #차선 정보를 담은 4개의 좌표를 보낸다.
    
    #구독할 토픽, 구독할 메시지의 타입, 메시지 수신했을때 호출할 콜백 함수, 콜백 함수에 추가로 전달할 인수들
    image_transport = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback, (road_lane_detector, image_pub))

    #result라는 이름의 창 생성
    cv2.namedWindow("result")
    
    rospy.spin() #현재 스레드에서 무한 루프를 실행해서 콜백함수가 호출될 수 있도록 대기함.


if __name__ == '__main__':
    main()