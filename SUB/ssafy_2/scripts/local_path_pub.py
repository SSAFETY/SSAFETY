#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        '''
        # Global Path 와 Odometry 데이터 subscriber 를 생성한다.
        # 콜백 함수의 이름은 self.global_path_callback, self.odom_callback 로 한다.
        rospy.Subscriber( odometry 메세지 콜백 완성하기 )
        rospy.Subscriber( global path 메세지 콜백 완성하기 )

        '''

        #TODO: (2) Local Path publisher 선언
        '''
        # local Path 데이터를 Publish 하는 변수를 선언한다.
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

        '''
        
        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        '''
        # Local Path 의 크기를 지정한다.
        # 차량이 주행 시 Local Path 의 크기 만큼의 정보를 가지고 주행하게 된다
        # 너무 작지도 크기지도 않은 값을 사용한다 (50 ~ 200)
        self.local_path_size = 

        '''
        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                '''
                # global Path 에서 차량의 현재 위치를 찾습니다.
                # 현제 위치는 WayPoint 로 기록하며 현재 차량이 Path 에서 몇번 째 위치에 있는지 나타내는 값이 됩니다.
                # 차량의 현재 위치는 Local Path 를 만드는 시작 위치가 됩니다.
                # 차량의 현재 위치를 탐색하는 반복문은 작성해 current_waypoint 찾습니다.
                min_dis = float('inf')
                current_waypoint = -1
                for  in  :

                '''
                
                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
                '''
                # 차량의 현재 위치 부터 local_path_size 로 지정한 Path 의 크기 만큼의 Path local_path 를 생성합니다.
                # 차량에 남은 Path 의 길이가 local_path_size 보다 작은 경우가 있음으로 조건 문을 이용하여 해당 조건을 예외 처리 합니다.
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    
                    else :

                '''

                print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                '''
                # Local Path 메세지 를 전송하는 publisher 를 만든다.
                self.local_path_pub.
                
                '''

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

        '''
        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
        self.x = 물체의 x 좌표 
        self.y = 물체의 y 좌표

        '''

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg        

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
