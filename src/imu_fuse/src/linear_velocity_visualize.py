#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
import time

# 초기값 설정
initial_time = None
initial_accel_x = None
initial_accel_y = None

# 마지막 시간과 속도
last_time = 0
last_vel_x = 0
last_vel_y = 0

# 현재 속도
current_vel_x = 0
current_vel_y = 0

# 속도가 마지막으로 갱신된 시간
last_update_time = 0
reset_last_update_time = 0
reset_current_update_time = 0 
time_threshold = 0.5  # 속도가 변하지 않는 시간을 확인하는 임계값 (초 단위)

# 가속도를 적분하여 속도를 계산하는 함수
def integrate_acceleration(accel, time):
    global last_time, last_vel_x, last_vel_y, current_vel_x, current_vel_y, last_update_time, reset_last_update_time

    # 시간 간격 계산
    dt = time - last_time

    # 속도 계산 (적분)
    new_vel_x = last_vel_x + accel[0] * dt
    new_vel_y = last_vel_y + accel[1] * dt

    # 속도의 차이가 0.1 넘을 때 리셋 타임 초기화(속도 0으로 초기화용)
    # print(new_vel_x - last_vel_x)
    # print(rospy.get_time())
    if abs(new_vel_x - last_vel_x) > 0.05:
        reset_last_update_time = rospy.get_time()
        
    # 새로운 시간과 속도 저장
    last_time = time
    last_vel_x = new_vel_x
    last_vel_y = new_vel_y

    # 속도 갱신
    current_vel_x = new_vel_x
    current_vel_y = new_vel_y

    # 속도가 갱신된 시간 업데이트
    last_update_time = rospy.get_time()
    

# IMU 데이터 콜백 함수
def imu_callback(data):
    global initial_time, initial_accel_x, initial_accel_y, reset_current_update_time

    current_time = data.header.stamp.to_sec()
    reset_current_update_time = rospy.get_time()
    if initial_time is None:
        initial_time = current_time
        initial_accel_x = data.linear_acceleration.x
        initial_accel_y = data.linear_acceleration.y
        return

    accel_x = data.linear_acceleration.x - initial_accel_x
    accel_y = data.linear_acceleration.y - initial_accel_y
    # print(data.linear_acceleration.x)

    integrate_acceleration([accel_x, accel_y], current_time - initial_time)

# 속도 초기화 함수
def reset_velocity_if_needed():
    global last_vel_x, last_vel_y, current_vel_x, current_vel_y, reset_last_update_time, time_threshold, reset_current_update_time

    current_time = rospy.get_time()
    # print("reset_current_update_time:",reset_current_update_time )
    # print("reset_last_update_time:", reset_last_update_time)  
    if (reset_current_update_time - reset_last_update_time) > time_threshold:
        current_vel_x = 0
        current_vel_y = 0
        last_vel_x = 0
        last_vel_y = 0

# 그래프 업데이트 함수
def update_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(3, 1))
    while not rospy.is_shutdown():
        reset_velocity_if_needed()  # 속도를 초기화할지 확인
        ax.clear()
        ax.text(0.5, 0.5, f'Velocity X: {current_vel_x:.2f} m/s\nVelocity Y: {current_vel_y:.2f} m/s', 
                fontsize=15, ha='center', va='center')
        ax.axis('off')
        plt.pause(0.1)
    plt.ioff()
    plt.show()

# ROS 노드 초기화 및 구독자 설정
def main():
    rospy.init_node('imu_integrator')
    rospy.Subscriber('/imu', Imu, imu_callback)

    # 그래프 업데이트를 위한 함수 호출
    update_plot()

    rospy.spin()

if __name__ == '__main__':
    main()
