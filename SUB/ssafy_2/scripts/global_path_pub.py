#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# global_path_pub 은 txt 파일로 저장한 Path 데이터를 global Path (전역경로) 로 읽어오는 예제입니다.
# 만들어진 global Path(전역경로) 는 Local Path (지역경로) 를 만드는데 사용 된다.

# 노드 실행 순서 
# 1. Global Path publisher 선언 및 Global Path 변수 생성 
# 2. 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. Global Path 정보 Publish


class global_path_pub :
    def __init__(self, pkg_name = 'ssafy_2', path_name = 'kcity'):
        rospy.init_node('global_path_pub', anonymous = True)

        #TODO: (1) Global Path publisher 선언 및 Global Path 변수 생성 
        '''
        # Global Path 데이터를 Publish 하는 변수와 메세지를 담고있는 변수를 선언한다.
        # 이때 Global Path 는 map 좌표계를 기준으로 생성한다.
        self.global_path_pub = 
        self.global_path_msg = 
        self.global_path_msg.header.frame_id = 

        '''

        #TODO: (2) 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
        '''
        # Path 데이터가 기록 된 txt 파일의 경로와 이름을 정한다.
        # 이후 읽기 모드로 연다.
        # pkg_name 과 path_name 은 21 번 줄 참고한다.
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = 
        self.f =         
        lines = self.f.readlines()

        '''

        #TODO: (3) 읽어 온 경로 데이터를 Global Path 변수에 넣기
        '''
        # 읽어온 x y z 좌표 데이터를 self.global_path_msg 변수에 넣는다.
        # 넣어준 반복 문을 이용하여 작성한다.
        for line in lines :
            tmp = line.split()
            read_pose = 
            read_pose.pose.position.x = 
            read_pose.pose.position.y = 
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)        
        self.f.close()

        '''

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            '''
            # Global Path 메세지 를 전송하는 publisher 를 만든다.
            self.global_path_pub.
            
            '''
            rate.sleep()

if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
