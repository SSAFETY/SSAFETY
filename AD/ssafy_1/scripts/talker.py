#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

# talker 는 ROS 에서 가장 기본이 되는 Topic Publisher(메세지 송신)의 기본 예제입니다.
# String 타입의 /ssafy 라는 메세지를 Publish 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. ROS 노드 이름 선언
# 3. 코드 반복 시간 설정 및 반복 실행
# 4. 송신 될 String 메세지 변수 생성 및 터미널 창 출력
# 5. /ssafy 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    publisher = rospy.Publisher('ssafy', String, queue_size=10)

    #TODO: (2) ROS 노드 이름 선언
    rospy.init_node('ros_talker', anonymous=True)    
    
    count = 0

    #TODO: (3) 코드 반복 시간 설정 및 반복 실행    
    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        #TODO: (4) 송신 될 String 메세지 변수 생성 및 터미널 창 출력 
        hello_ssafy = "hello ssafy %s" % count
        rospy.loginfo(hello_ssafy)

        #TODO: (5) /ssafy 메세지 Publish 
        publisher.publish(hello_ssafy)

        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
