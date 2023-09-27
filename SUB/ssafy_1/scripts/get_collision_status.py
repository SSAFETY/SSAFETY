#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import CollisionData

# Collision_listener 는 시뮬레이터에서 송신하는 Collision 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Ego 차량의 충돌 정보인 /CollisionData 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def Collision_callback(data):
    os.system('clear')
    for i in range(len(data.collision_object)) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('Collision Object Name : {}'.format(data.collision_object[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.collision_object[i].position.x,data.collision_object[i].position.y,data.collision_object[i].position.z))
def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Collision_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    '''
    # CollisionData 라는 Morai ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    rospy.Subscriber( 변수 1 , 변수 2 , Collision_callback)

    '''
    rospy.Subscriber( '/CollisionData' , CollisionData , Collision_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
