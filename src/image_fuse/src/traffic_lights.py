#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 퍼블리셔 초기화
green_light_pub = None
bridge = CvBridge()

def publish_green_light():
    global green_light_pub
    msg = Bool()
    msg.data = True
    green_light_pub.publish(msg)

def image_callback(msg):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # 이미지 크기
        height, width = cv_image.shape[:2]

        # 오른쪽 위 부분만 관심 영역으로 설정
        roi = cv_image[height//3:height//2, (2*width)//3:width]

        # BGR -> HSV 변환
        roi_img_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 초록색의 HSV 범위 설정
        lower_green = np.array([60, 100, 100])
        upper_green = np.array([70, 255, 255])

        # 정의된 색상 범위 내에 있는 픽셀을 추출하고 마스크 생성
        green_mask = cv2.inRange(roi_img_hsv, lower_green, upper_green)
        
        # 초록불 검출 (초록색 픽셀이 100개 이상일 때만)
        if cv2.countNonZero(green_mask) > 90:
            publish_green_light()

    except CvBridgeError as e:
        rospy.logerr("cv_bridge 예외: %s", str(e))

def main():
    global green_light_pub

    rospy.init_node('green_light_detector')

    # 토픽 퍼블리셔 초기화
    green_light_pub = rospy.Publisher('/green_light', Bool, queue_size=1)

    # 이미지 트랜스포트 초기화
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    main()