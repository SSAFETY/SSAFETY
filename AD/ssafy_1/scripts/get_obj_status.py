#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import ObjectStatusList

# Obj_status_listener 는 시뮬레이터에서 송신하는 Object 정보를 Subscriber 하는 예제 입니다.
# 시뮬레이터 내 Object 정보인 /Object_topic 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
'''
# Object들의 상태 데이터를 담고 있는 ObjectStatusList 메세지에는 
# Object들의 위치 속도 가속도 heading 값을 담고 있습니다. 
# Object의 종류는 크게 NPC Vehicle, Pedestrian, Obstacle 3가지로 되어 있습니다.
# 각 Object의 위치와 속도 가속도 heading 값을 아래 형식에 맞춰서 작성하여 터미널 창에 출력해볼 수 있습니다. 
# 아래 형식과 같이 반복문을 이용해 모든 Object 정보를 출력 해보세요.

'''
def Object_callback(data):
    os.system('clear')
    rospy.loginfo('-------------------- NPC Vehicle -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_npcs))
    for i in range(data.num_of_npcs) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.npc_list[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.npc_list[i].position.x,data.npc_list[i].position.y,data.npc_list[i].position.z))
        rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.npc_list[i].velocity.x,data.npc_list[i].velocity.y,data.npc_list[i].velocity.z))
        rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.npc_list[i].acceleration.x,data.npc_list[i].acceleration.y,data.npc_list[i].acceleration.z))
        rospy.loginfo('heading      : {} deg'.format(data.npc_list[i].heading))
        rospy.loginfo('size         : x = {0} , y = {1}, z = {2} m'.format(data.npc_list[i].size.x,data.npc_list[i].size.y,data.npc_list[i].size.z))
    '''
    rospy.loginfo('-------------------- Pedestrian -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_pedestrian))
    for i in range(data.num_of_pedestrian) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))

    rospy.loginfo('-------------------- Obstacle -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_obstacle))
    for i in range(data.num_of_obstacle) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.obstacle_list[i].name))
        
    '''

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('Obj_status_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    '''
    # ObjectStatusList 라는 Morai ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    rospy.Subscriber( 변수 1 , 변수 2 , Object_callback)

    '''
    rospy.Subscriber( '/Object_topic' , ObjectStatusList, Object_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
