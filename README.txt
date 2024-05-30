센서 노드의 데이터를 받아 연산 후 시뮬레이터에 TOPIC을 줘서 제어하면 됨

수동 주행 동영상 링크
https://youtu.be/zLAAEC7dE_4

노드를 새로 만들면 (예를 들면 노드로 쓸 .py 파일) 빌드를 다시 해주어야함
작업공간에서
catkin_make

설치부터

# 5/30 carla_ctl 룰센서 on off 버전 릴리즈
deb 파일 다운로드 후 다음 명령어로 설치
sudo dpkg -i /path/to/new_version.deb

# carla_ctl node 생성과 파라미터
_rule_sensor_on:= 1/0 룰 센서 on/off
- 새로 다운받은 버전에서 사용 가능

rosrun carla_ctl carla_ctl \
_camera_sensor_x:=0.0 \
_camera_sensor_y:=1.8 \
_camera_sensor_z:=1 \
_lidar_sensor_x:=0.0 \
_lidar_sensor_y:=2.4 \
_lidar_sensor_z:=0.3 \
_host:="172.21.112.1" \
_rule_sensor_on:= 0
_port:=2000

