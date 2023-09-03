#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

# my_name_talker 는 Custom Msgs 를 이용한 Topic Publisher(메세지 송신) 예제입니다.
# /my_name 라는 메세지를 Publish 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. ROS 노드 이름 선언
# 3. 코드 반복 시간 설정 및 반복 실행
# 4. 송신 될 메세지 변수 생성 및 터미널 창 출력
# 5. /my_name 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    '''
    # student 라는 직접 만든 Custom ROS 메세지 형식을 사용하여 Topic ublisher 를 완성한다.
    # Topic 이름은 'my_name' 으로 설정한다.
    publisher = rospy.Publisher( 변수 1 , 변수 2 , queue_size=10)

    '''
    publisher = rospy.Publisher('/myname' , student , queue_size=10)

    #TODO: (2) ROS 노드 이름 선언
    rospy.init_node('my_name_talker', anonymous=True)

    count = 0

    #TODO: (3) 코드 반복 시간 설정 및 반복 실행    
    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        #TODO: (4) 송신 될 메세지 변수 생성 및 터미널 창 출력 
        '''
        # 송신 될 메세지 변수를 만든뒤 출력 결과를 확인한다.        
        my_name = student()
        my_name.first_name = 
        my_name.last_name = 
        my_name.age = 
        my_name.score = 
        rospy.loginfo('\n my name : %s %s \n my age : %i \n SSAFY score : %i', my_name.first_name,my_name.last_name,my_name.age,my_name.score)
        
        '''
        
        my_name = student()
        my_name.first_name = "CHUNG"
        my_name.last_name = "LEE"
        my_name.age = 23
        my_name.score = 50
        rospy.loginfo('\n my name : %s %s \n my age : %i \n SSAFY score : %i', my_name.first_name,my_name.last_name,my_name.age,my_name.score)
        

        #TODO: (5) /my_name 메세지 Publish 
        publisher.publish(my_name)
        
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
