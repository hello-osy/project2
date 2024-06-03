#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from xycar_msgs.msg import xycar_motor
#from image_fuse_msgs.msg import Point
import signal
import time

### 800x600
import numpy as np
import math

theta = 0
WIDTH = 800
LEGTH = 600

class CarController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.speed = 0
        self.angle = 0

    def compute(self, error, dt):
        """PID 제어 계산을 수행하고 조정된 제어 값을 반환."""
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def start_green(self, msg, args):
        car_controller, motor = args
        rospy.loginfo("green accepted")
        self.speed = 50  # 녹색 신호일 때 속도 설정
        msg = xycar_motor()
        msg.angle = self.angle
        msg.speed = self.speed
        motor.publish(msg)

    def set_velocity(self, msg):
        self.speed = msg.data  # 현재 속도 업데이트

    def set_orientation(self, msg):
        self.angle = msg.data  # 현재 조향 업데이트

    def steering_vanishing_point(self, x):
        standard_x = int(WIDTH/2)
        diff = standard_x - x 
        if diff > 0:   #좌회전
            theta = self.matching(diff, 0, WIDTH/2, 90, 45)
        elif diff < 0:
            theta = self.matching(diff, 0, -WIDTH/2, 90, 135)

        return theta

    def matching(self, x, input_min, input_max, output_min, output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    def control_speed(self, steering_angle): #속도 제어 함수, 나중에 수정할 것.
        #조향각이 커지면 속도를 좀 줄이고, 조향각이 작아지면 속도를 좀 빠르게 해주자.
        #비례적으로 속도를 조절합니다

        max_speed=50 #이건 우리 마음대로
        min_speed=20

        speed = max_speed - (max_speed - min_speed) * (abs(steering_angle - 90) / 90.0)

        return speed

#PID_controller = CarController(kp=3, ki=0.8, kd=0.7)

def lane_callback(msg, args):
    car_controller, motor = args

    #[(),(),(),()] 형태의 메시지임.
    right_x1, right_x2, left_x1, left_x2, y1, y2= msg[0][0], msg[1][0], msg[2][0], msg[3][0], msg[0][1], msg[0][1]

    m1 = (y2 - y1) / (right_x2 - right_x1)
    m2 = (y2 - y1) / (left_x2 - left_x1)
    
    b1 = y1 - m1 * right_x1
    b2 = y1 - m2 * left_x1
    
    if m1 == m2:
        return None  # 평행한 경우 교점이 없음
    
    #목표 지점의 x,y좌표
    x_intersect = (b2 - b1) / (m1 - m2)
    y_intersect = m1 * x_intersect + b1

    dt = 0.1  # 가정된 시간 간격, 실제로는 rospy.Time 사용해서 계산

    theta = car_controller.steering_vanishing_point(x_intersect) #좌표 오차를 각도 오차로 변환함 
    # PID 제어를 통해 각도 계산
    angle = car_controller.compute(theta, dt)
    speed = car_controller.control_speed(angle) #속도 제어 부분은 나중에 수정할 것
    
    # 각도와 속도를 퍼블리시
    msg = xycar_motor()
    msg.angle = angle
    msg.speed = speed
    motor.publish(msg)

def matching(x, input_min, input_max, output_min, output_max):  #x가 input_min과 input_max 사이에 있다면, 이를 output_min과 output_max 사이의 값으로 매핑합니다.
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

def main():
    rospy.init_node('control', anonymous=True)
    
    car_controller=CarController(kp=3, ki=0.8, kd=0.7)
    # 토픽 이름, 메시지 타입, 메시지 큐 크기
    motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

    # 토픽 이름, 메시지 타입, 콜백 함수
    #rospy.Subscriber('/target_point', image_fuse_msgs/Point, image_callback, (car_controller, motor)) 이거 대신 좌표4개를 받는 것으로 수정.
    rospy.Subscriber('/lane_detector', Float32MultiArray, lane_callback, (car_controller, motor))
    rospy.Subscriber('/green_light', Bool, car_controller.start_green, (car_controller, motor))
    rospy.Subscriber('/velocity', Float64, car_controller.set_velocity)
    rospy.Subscriber('/orientation', Float64, car_controller.set_orientation)

    # Ctrl+C 누르면 종료할 수 있게 만듦.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()  # 콜백 함수를 호출하면서 계속 실행

def signal_handler(sig, frame):
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__':
    main()
