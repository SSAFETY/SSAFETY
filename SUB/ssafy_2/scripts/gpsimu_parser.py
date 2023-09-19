#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi

# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 죄표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        '''
        # GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
        # 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
        # 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존제한다.
        # 맵 좌표계는 m 단위를 사용한다.
        # 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
        # https://pyproj4.github.io/pyproj/stable/api/proj.html
        # " proj= , zone= , ellps =  , preserve_units = "
        self.proj_UTM = Proj( 좌표 변환을 위한 변수 입력 )

        '''

        #TODO: (2) 송신 될 Odometry 메세지 변수 생성
        '''
        # ROS 메세지 중 물체의 위치와 자세 데이터를 나타내는 Odometry 메세지를 사용한다.
        # 차량의 현재 위치와 자세 데이터를 GPS IMU 센서에 담아서 Publsih 한다.
        # 이때 frame_id 는 '/odom' child_frame_id 는 '/base_link' 로 한다.

        self.odom_msg = 
        self.odom_msg.header.frame_id = 
        self.odom_msg.child_frame_id = 

        '''

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()

                #TODO: (5) Odometry 메세지 Publish
                '''
                # Odometry 메세지 를 전송하는 publisher 를 만든다.
                self.odom_pub.
                
                '''

                os.system('clear')
                print(" ROS Odometry Msgs Pose ")
                print(self.odom_msg.pose.pose.position)
                print(" ROS Odometry Msgs Orientation ")
                print(self.odom_msg.pose.pose.orientation)

                rate.sleep()

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):
        '''
        # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
        # 변환 시 이전 gps_parser.py 예제와 달리 시뮬레이터 GPS 센서의 offset 값을 적용 한다.
        # GPS 센서에서 출력되는 Offset 값은 시뮬레이터에 맵 좌표계로 변경을 위한 값이다.
        # UTM 좌표로 변환 된 x, y 값에 offset 값을 빼주면 된다.
        xy_zone = self.proj_UTM(위도 데이터, 경도 데이터)

        # if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        '''

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        '''
        # Offset 을 적용하여 시뮬레이터 맵 좌표계 값으로 변환 된 좌표 데이터를 Odometry 메세지에 넣는다.
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = 
        self.odom_msg.pose.pose.position.y = 
        self.odom_msg.pose.pose.position.z =

        '''

    def imu_callback(self, data):

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        '''
        # IMU 를 통해 받은 물체의 자세 데이터를 Odometry 메세지에 넣는다.
        # if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

        '''
        self.is_imu=True

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
