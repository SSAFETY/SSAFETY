#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf

# pure_pursuit 은 차량의 차량의 횡 방향 제어 예제입니다.
# 차량이 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 차량의 제어 입력은 CtrlCmd 메세지를 이용 하며 종방향 제어 입력은 longlCmdType 2(Velocity control) 이용하여 등속 운동 하도록 합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 좌표 변환 행렬 생성
# 3. Steering 각도 계산
# 4. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# Pure Pursuit 은 차량의 Kinematic Model Based 경로 추종 알고리즘 입니다.
# 현재 차량의 Heading 각도와 실제 Path 의 각도 오차를 계산하여 차량의 조향 각도를 계산합니다.
# 전방주시거리(Look Forward Distance) 라는 변수를 가지고 있습니다.
# 전방주시거리는 Pure Pursuit 알고리즘에서 기준으로 하는 추종 거리로 조향 각도를 계산하는데 이용 합니다.
# 전방주시거리(Look Forward Distance)는 해당 예제에서 "self.lfd" 라는 변수로 사용합니다.
# 직접 "self.lfd" 변수의 값에 적절한 값을 넣어 제어 알고리즘의 성능을 올릴 수 있습니다.
# 아래 정의 된 변수 중 "self.vehicle_length" 와 "self.lfd" 를 변경 하여서 직접 제어기 성능을 튜닝 해보세요.

'''
class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        '''
        # Local Path 와 Odometry 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        rospy.Subscriber("local_path" )
        rospy.Subscriber("odom" )
        self.ctrl_cmd_pub = 

        '''

        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False

        self.is_look_forward_point=False

        self.forward_point=Point()
        self.current_postion=Point()

        self.vehicle_length = 1
        self.lfd = 1

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True  :

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                    self.ctrl_cmd_msg.velocity = 20.0
                    print(self.ctrl_cmd_msg.steering)
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                #TODO: (4) 제어입력 메세지 Publish
                '''
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                self.ctrl_cmd_pub.
                
                '''

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=tf.transformations.euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def calc_pure_pursuit(self,):                
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (2) 좌표 변환 행렬 생성
        '''
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

        trans_matrix = np.array([   [                       ,                       ,               ],
                                    [                       ,                       ,               ],
                                    [0                      ,0                      ,1              ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point = 

            global_path_point = [ , , 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = 
                if dis >= self.lfd :
                    self.forward_point = 
                    self.is_look_forward_point = True
                    break

        '''
        
        #TODO: (3) Steering 각도 계산
        '''
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta = 
        steering = 

        '''

        return steering


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
