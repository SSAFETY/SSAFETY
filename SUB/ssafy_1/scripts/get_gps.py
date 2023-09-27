#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import GPSMessage

# gps_data_listener 는 시뮬레이터에서 송신하는 gps 센서 정보를 Subscriber 하는 예제 입니다.
# gps 센서 정보인 /gps 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. Callback 함수 생성 및 데이터 출력

#TODO: (1) Callback 함수 생성 및 데이터 출력
def gps_callback(data):
    '''
    # 시뮬레이터의 GPS 데이터를 아래와 같은 형식으로 터미널 창에 출력한다.
    # GPS 센서의 위도 경도 고도, Offset 값을 확인 할 수 있다.
    # Offset 값은 시뮬레이터 좌표계를 계산하는데 사용되며 사용 된다.
    rospy.loginfo("latitude {}".format( GPS 의 위도 데이터 ))
    rospy.loginfo("longitude {}".format( GPS 의 경도 데이터 ))
    rospy.loginfo("eastOffset {}".format( 시뮬레이터의 맵 좌표계의 offset 값 ))
    rospy.loginfo("northOffset {}".format( 시뮬레이터의 맵 좌표계의 offset 값 ))
    '''

    os.system('clear')
    print("\n ------------------------------------ \n")
    rospy.loginfo("latitude {}".format(data.latitude))
    rospy.loginfo("longitude {}".format(data.longitude))
    rospy.loginfo("altitude {}".format(data.altitude))
    rospy.loginfo("eastOffset {}".format(data.eastOffset))
    rospy.loginfo("northOffset {}".format(data.northOffset))


def listener():
    rospy.init_node('gps_data_listener', anonymous=True)
    '''
    # GPSMessage 라는 Morai ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    '''
    rospy.Subscriber('/gps', GPSMessage, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
