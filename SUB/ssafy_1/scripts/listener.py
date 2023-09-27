#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

# listener 는 ROS 에서 가장 기본이 되는 Topic Subscriber(메세지 수신)의 기본 예제입니다.
# String 타입의 /ssafy 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def callback(data):
    rospy.loginfo('%s', data.data)

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('ros_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber('ssafy', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
