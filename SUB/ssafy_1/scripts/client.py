#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from ssafy_1.srv import AddTwoInts

# ros_client 는 Custom Msgs 를 이용한 Client Node 작성 예제입니다.
# 두 정수 값을 송신 후 결과 값을 반환하는 Client Node 를 생성 합니다.

# 노드 실행 순서 
# 1. Service 가 생성 대기 함수 선언
# 2. Service 호출
# 3. Service 호출 결과 값 확인

def srv_client():
    rospy.init_node('ros_client', anonymous=True)
    #TODO: (1) Service 가 생성 대기 함수 선언
    rospy.wait_for_service('AddTwoInts')

    a = 10
    b = 11

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            #TODO: (2) Service 호출
            AddTwoInts_srv = rospy.ServiceProxy('AddTwoInts', AddTwoInts)
            result = AddTwoInts_srv(a,b)

            #TODO: (3) Service 호출 결과 값 확인
            rospy.loginfo('(Result) %i + %i = %i', a,b,result.sum)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
