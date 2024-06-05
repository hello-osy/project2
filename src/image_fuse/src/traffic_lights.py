#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal

# 퍼블리셔 초기화
green_light_pub = None

def publish_green_light():
    global green_light_pub
    msg = Bool()
    msg.data = True
    rate = rospy.Rate(1)  # 1 Hz, 1초에 한 번씩 publish
    for _ in range(20):
        green_light_pub.publish(msg)
        rate.sleep()

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 이미지 크기
        height, width = cv_image.shape[:2]

        # 오른쪽 위 부분만 관심 영역으로 설정
        roi = cv_image[0:height//2, width//2:width]

        # BGR -> HSV 변환
        roi_img_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 초록색이라고 정의할 색의 범위 설정
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        # 정의된 색상 범위 내에 있는 픽셀을 추출하고 마스크 생성
        green_mask = cv2.inRange(roi_img_hsv, lower_green, upper_green)


        # 초록불 검출
        if cv2.countNonZero(green_mask) > 0:  # 이미지에 초록색이 있다면, 1
            print(1)
            publish_green_light()

        if cv2.waitKey(1) & 0xFF == 27:  # ESC 키를 누르면 종료
            rospy.signal_shutdown("ESC pressed")
            cv2.destroyAllWindows()

    except CvBridgeError as e:
        rospy.logerr("cv_bridge 예외: %s", str(e))

def signal_handler(sig, frame):
    rospy.signal_shutdown("Ctrl+C pressed")
    cv2.destroyAllWindows()

def main():
    global green_light_pub

    rospy.init_node('green_light_detector', anonymous=True)

    # 토픽 퍼블리셔 초기화
    green_light_pub = rospy.Publisher('/green_light', Bool, queue_size=1)

    # 이미지 트랜스포트 초기화
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # Ctrl+C 시그널 핸들러 설정
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()  # 콜백 함수를 호출하면서 계속 실행
    # 외부에서 종료 신호를 받기 전까지 계속 루프를 돌면서, 메시지가 들어올 때마다 콜백함수를 실행함.

if __name__ == '__main__':
    main()