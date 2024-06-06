#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from xycar_msgs.msg import xycar_motor
import signal
import time
import queue

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
        self.drive_mode =False
        self.start_time = 0
        self.size10queue = queue.Queue(10)
        
    def compute(self, error, dt):
        """PID 제어 계산을 수행하고 조정된 제어 값을 반환."""
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def start_green(self, msg, args):
        car_controller = args
        car_controller.drive_mode = True
        car_controller.start_time = time.time()

    def set_velocity(self, msg):
        self.speed = msg.data  # 현재 속도 업데이트

    def set_orientation(self, msg):
        self.angle = msg.data  # 현재 조향 업데이트

    def matching(self, x, input_min, input_max, output_min, output_max):
        return (x - input_min) * (output_max - output_min) / (input_max - input_min) + output_min # map() 함수 정의.

    def diff_queue(self, diff):
        d_queue = self.size10queue
        if diff is None:
            pass
        else:
            if d_queue.full():
                d_queue.get(block=False)
            d_queue.put(diff, False)

        # 큐가 비어있다면 0을 리턴
        if d_queue.empty():
            return 0

        # 큐의 모든 원소의 합과 개수를 계산하여 평균을 구함
        total_sum = 0
        total_count = 0
        temp_list = list(d_queue.queue)  # 큐의 원소를 리스트로 변환

        for item in temp_list:
            total_sum += item
            total_count += 1

        average = total_sum / total_count
        return average
            

    def car_position(self, list): # 차체 위치 에러
        x1, y1, x2, y2, x3, y3, x4, y4 = list
        center_line = (x1 + x3) / 2
        error = (WIDTH / 2) - center_line
        return error

    def steering_vanishing_point(self, x):
        standard_x = int(WIDTH / 2)
        diff = standard_x - x 
        if diff > 0:   # 좌회전
            theta = self.matching(diff, 0, -WIDTH / 2, 0, -10)
        elif diff < 0:
            theta = self.matching(diff, 0, WIDTH / 2, 0, 10)
        else:
            theta = 0

        return theta

    def steering_theta(self, w1, w2): # 차선 기울기 기준 조향
        if np.abs(w1) > np.abs(w2):  # 우회전
            if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
                w1 = -w1
                angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
                theta = self.matching(angle, 0, np.pi / 2, 0, 10)
            elif w1 * w2 > 0:  # 극한으로 틀어진 방향
                if w1 > w2:
                    theta = 0
                else:
                    theta = 0
            else:
                theta = 0
        elif np.abs(w1) < np.abs(w2):  # 좌회전
            if w1 * w2 < 0:  # 정방향 or 약간 틀어진 방향
                w1 = -w1
                angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1) * math.tan(w2)))
                theta = self.matching(angle, 0, np.pi / 2, 0, -10)
            elif w1 * w2 > 0:  # 극한으로 틀어진 방향
                if w1 > w2:
                    theta = 0
                else:
                    theta = 0
            else:
                theta = 0
        else:
            theta = 0

        return theta

    def steering_calcu(self, input_data): # 조향값 도출
        x3, y3, x4, y4, x1, y1, x2, y2 = input_data

        # 분모가 0인 경우를 예외 처리
        if x1 == x2 and x3 == x4:
            return None  # 두 점이 같은 x 좌표를 가지면 기울기가 무한대가 되므로 None을 반환
        else:
            # 기울기 계산
            left_calculated_weight = (y1 - y2) / (x2 - x1)
            right_calculated_weight = (y3 - y4) / (x4 - x3)

            # 절편 계산
            left_calculated_bias = y1 - left_calculated_weight * x1
            right_calculated_bias = y3 - right_calculated_weight * x3

            cross_x = (left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)
            cross_y = left_calculated_weight * ((left_calculated_bias - right_calculated_bias) / (right_calculated_weight - left_calculated_weight)) + right_calculated_bias
            
            if not np.isnan(cross_x) and not np.isnan(cross_y):
                if -5 < self.steering_theta(left_calculated_weight, right_calculated_weight) < 5:
                    # print('소실점 조향 서보모터 각도: ', self.steering_vanishing_point(cross_x))
                    steering_angle = self.steering_vanishing_point(cross_x)
                else:
                    # print("기울기 조향 서보모터 각도: ", self.steering_theta(left_calculated_weight, right_calculated_weight))
                    steering_angle = self.steering_theta(left_calculated_weight, right_calculated_weight)
            
        return steering_angle

def lane_callback(msg, args):
    car_controller, motor = args

    # 목표 지점과 현재 위치의 차이를 계산 (여기서는 단순히 가정) --> 좌표 오차를 각도 오차로 변환해야함
    # 목표 지점의 x 좌표를 거리 오차로 사용
    
    # dt = 0.001  # 가정된 시간 간격, 실제로는 rospy.Time 사용해서 계산
    
    theta = car_controller.steering_calcu(msg.data) # msg.data 넣어야 함. 그냥 msg에는 다른 정보도 들어있음.
    # angle = car_controller.compute(theta, dt)
    speed = 0.2 # car_controller.control_speed(angle) # 속도 제어 부분은 나중에 수정할 것
    elapsed_time = time.time()-car_controller.start_time
    # print(1, car_controller.start_time)
    # print(2, time.time())
    # print(3, elapsed_time)
    # 각도와 속도를 퍼블리시
    if car_controller.drive_mode == True:
        publish_msg = xycar_motor()
        if elapsed_time < 4.5:
            publish_msg.angle = -0.02
            publish_msg.speed = 0.5
        else:
            publish_msg.angle = theta
            publish_msg.speed = speed

        motor.publish(publish_msg)

    else:
        car_controller.start_time = time.time()

def main():
    rospy.init_node('control', anonymous=True)
    
    car_controller = CarController(kp=3, ki=0.8, kd=0.7)
    # 토픽 이름, 메시지 타입, 메시지 큐 크기
    motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

    # 토픽 이름, 메시지 타입, 콜백 함수
    rospy.Subscriber('/lane_detector', Float32MultiArray, lane_callback, (car_controller, motor))
    rospy.Subscriber('/green_light', Bool, car_controller.start_green, (car_controller))
    rospy.Subscriber('/velocity', Float64, car_controller.set_velocity)
    rospy.Subscriber('/orientation', Float64, car_controller.set_orientation)

    # Ctrl+C 누르면 종료할 수 있게 만듦.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.spin()  # 콜백 함수를 호출하면서 계속 실행

def signal_handler(sig, frame):
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__':
    main()
