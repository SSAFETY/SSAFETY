#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import os
#TODO: (0) pyproj 라이브러리 Import [ pip install pyproj ]
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage, EgoVehicleStatus

# gps_parser 는 GPS의 위경도 데이터를 UTM 좌표로 변환하는 예제입니다.
# Pyproj 라이브러리를 사용 

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언  
# 2. 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성 
# 3. 위도 경도 데이터를 UTM 좌표로 변환   
# 4. 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인  

class LL2UTMConverter:
    def __init__(self, zone=52) :
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        # 초기화
        self.x, self.y = None, None

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        '''
        # GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
        # 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
        # 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존재한다.
        # 맵 좌표계는 m 단위를 사용한다.
        # 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
        # https://pyproj4.github.io/pyproj/stable/api/proj.html
        # " proj= , zone= , ellps =  , preserve_units = "
        self.proj_UTM = Proj( 좌표 변환을 위한 변수 입력 )

        '''

    #TODO: (2) 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성
    def navsat_callback(self, gps_msg):
        '''
        GPS 센서에서 수신되는 위도 경도 데이터를 확인한다.
        self.lat = 
        self.lon = 

        '''
        self.convertLL2UTM()

        utm_msg = Float32MultiArray()

        #TODO: (4) 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인
        '''
        UTM 으로 변환 된 좌표 데이터와 위도 경도 데이터를 터미널 창에 출력되도록 한다.
        utm_msg.data = [self.x, self.y]
        os.system('clear')
        print(' lat : ', 위도 데이터)
        print(' lon : ', 경도 데이터)
        print(' utm X : ', utm 좌표로 변환한 x 좌표)
        print(' utm Y : ', utm 좌표로 변환한 y 좌표)

        '''


    #TODO: (3) 위도 경도 데이터를 UTM 좌표로 변환
    def convertLL2UTM(self):
        '''
        # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
        xy_zone = self.proj_UTM(위도 데이터, 경도 데이터)

        self.x = xy_zone[0]
        self.y = xy_zone[1]

        '''

if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()
        
