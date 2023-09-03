#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import EgoVehicleStatus
# Ego_status_listener 는 시뮬레이터에서 송신하는 Ego 차량 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Ego 차량의 정보인 /Ego_topic 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
'''
# Ego 차량의 상태 데이터를 담고 있는 EgoVehicleStatus 메세지에는 
# Ego 차량의 위치 속도 가속도 heading 값을 담고 있습니다. 
# 위치와 속도 가속도 heading 값을 아래 형식에 맞춰서 작성하여 터미널 창에 출력해볼 수 있습니다. 

'''
def EgoStatus_callback(data):

    rospy.loginfo('------------------Ego Vehicle Status------------------')
    rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.position.x, data.position.y, data.position.z))
    #rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format( Ego 차량의 x y z 좌표 속도 데이터를 입력합니다. ))
    #rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format( Ego 차량의 x y z 좌표 가속도 데이터를 입력합니다. ))
    #rospy.loginfo('heading      : {} deg'.format( Ego 차량의 heading 값을 입력합니다.))

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Ego_status_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    # EgoVehicleStatus 라는 Morai ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    rospy.Subscriber( "/Ego_topic", EgoVehicleStatus, EgoStatus_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
