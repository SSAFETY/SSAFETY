#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Imu

# imu_data_listener 는 시뮬레이터에서 송신하는 IMU 센서 정보를 Subscriber 하는 예제 입니다.
# IMU 센서 정보인 /imu 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. Callback 함수 생성 및 데이터 출력

#TODO: (1) Callback 함수 생성 및 데이터 출력
def imu_callback(data):
    '''
    # 시뮬레이터의 GPS 데이터를 아래와 같은 형식으로 터미널 창에 출력한다.
    '''
    os.system('clear')
    rospy.loginfo('------------------ IMU Sensor Status ------------------')
    # rospy.loginfo(data)
    rospy.loginfo("orientation:")
    rospy.loginfo("x : {} y : {} z : {} w : {}".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    rospy.loginfo("angular_velocity:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    rospy.loginfo("linear_acceleration:")
    rospy.loginfo("x : {} y : {} z : {}".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))

def listener():
    rospy.init_node('imu_data_listener', anonymous=True)
    '''
    # Imu 라는 ROS 의 센서 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    '''
    rospy.Subscriber('/imu', Imu, imu_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
