#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from ssafy_1.srv import AddTwoInts,AddTwoIntsResponse

# AddTwoInts_server 는 Custom Msgs 를 이용한 Service Node 작성 예제입니다.
# 두 정수 값을 받아 더한 값을 반환하는 Service Node 를 생성 합니다.

# 노드 실행 순서 
# 1. Service 선언
# 2. 반환 값 계산 및 출력

#TODO: (2) 반환 값 계산 및 출력
def add_two_ints(req):
    rospy.loginfo('(AddTwoInts) %i + %i = ', req.a,req.b)
    return AddTwoIntsResponse(req.a + req.b)

def srv_server():
    rospy.init_node('AddTwoInts_server', anonymous=True)

    #TODO: (1) Service 선언
    s = rospy.Service('AddTwoInts', AddTwoInts, add_two_ints)

    rospy.spin()

if __name__ == '__main__':
    srv_server()
