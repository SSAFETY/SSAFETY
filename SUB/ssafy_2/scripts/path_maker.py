#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry

# path_maker 는 차량의 위치 데이터를 받아 txt 파일로 저장하는 예제입니다.
# 저장한 txt 파일은 차량의 주행 경로가 되며 경로 계획에 이용 할 수 있습니다.

# 노드 실행 순서 
# 1. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
# 2. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 3. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
# 4. 이전 위치보다 0.5m 이상일 때 위치를 저장

class pathMaker :    
    def __init__(self, pkg_name = 'ssafy_2', path_name = 'make_path'):
        rospy.init_node('path_maker', anonymous=True)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_odom=False

        #TODO: (1) 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        '''
        # Path 데이터를 기록 하고 저장 할 경로와 txt 파일의 이름을 정한다.
        # 이후 쓰기 모드로 연다.
        # pkg_name 과 path_name 은 22 번 줄 참고한다.
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = 이곳에 txt 파일이 저장될 경로와 이름을 적는다
        self.f = 

        '''
        while not rospy.is_shutdown():
            if self.is_odom == True :
                # Ego 위치 기록
                self.path_make()
        self.f.close()

    def path_make(self):
        x = self.x
        y = self.y
        z = 0.0
        #TODO: (3) 콜백함수에서 이전 위치와 현재 위치의 거리 계산
        '''
        # 현재 차량의 위치와 이전에 지나온 위치의 좌표 데이터를 구한다.
        # 구해진 좌표 사이의 거리를 계산한다.
        # 이전 위치 좌표는 아래 #TODO: (4)에서 정의 한다.
        distance = 

        '''

        #TODO: (4) 이전 위치보다 0.5m 이상일 때 위치를 저장        
        if distance >0.5:
            '''
            # distance 가 0.5 보다 커야지만 동작한다.
            # 현재 위치 좌표를 data 에 담은 뒤 txt 파일로 작성한다.
            # data 는 문자열 이며 x y z 사이는 \t 로 구분한다
            data ='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data 변수를 넣는 위치이다)
            self.prev_x = 
            self.prev_y = 
            self.prev_z = 
            
            print(기록 된 위치 좌표를 출력한다)

            '''

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (2) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표 
        self.y = 물체의 y 좌표

        '''
if __name__ == '__main__' :
    try:
        p_m=pathMaker()
    except rospy.ROSInternalException:
        pass
            