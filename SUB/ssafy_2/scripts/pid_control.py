#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

# pid_contorl 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 좌표 변환 행렬 생성
# 3. Steering 각도 계산
# 4. PID 제어 생성
# 5. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# PID 제어는 대표적인 피드백 제어 이론입니다.
# 현재 값과 원하는 목표 값 차이를 비교하여 제어하는 방식 입니다.
# 수식이 매우 간단하며, 구현 난이도에 비해 탁월한 성능을 가집니다.
# 해당 예제에서는 원하는 목표 속도에 도달하기 위해 현재 속도와의 값을 비교하여 PID 제어를 진행합니다.
# Ego_Topic 을 이용하여 차량의 현재 속도 값을 알아냅니다.
# PID 제어는 원하는 값에 도달하기 위해 P, PI, PD, PID 등 제어 대상에 맞게 제어 방식을 선택해서 사용 할 수 있습니다.
# P I D 는 각 비례항 적분항 미분항 으로 구분 됩니다.
# P 비례항은 오차 값에 따라 출력이 변경됩니다.
# I 적분항은 누적되는 오차를 보안하는 역활을 합니다.
# D 미분항은 오차의 변화율에 반응하여 오차의 변화율이 크다면 빠르게 안정화 시키는 역활을 합니다.
# PID 각각의 Gain 변수 값을 변경해 제어 성능을 올릴 수 있습니다.
# Gain 변수 값에 변화에 따라 변화하는 속도를 관찰하며 직접 제어기를 만들어보세요. 

'''
class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        '''
        # Local Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        rospy.Subscriber("local_path" )
        rospy.Subscriber("odom" )
        rospy.Subscriber("/Ego_topic" )
        self.ctrl_cmd_pub = 

        '''

        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1

        self.is_path=False
        self.is_odom=False          
        self.is_status=False

        self.is_look_forward_point=False

        self.forward_point=Point()
        self.current_postion=Point()

        self.vehicle_length = 1
        self.lfd = 5
        self.target_vel = 60

        self.pid = pidControl()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0

                output = self.pid.pid(self.target_vel,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (5) 제어입력 메세지 Publish
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
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg

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

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        '''
        # 종방향 제어를 위한 PID 제어기는 현재 속도와 목표 속도 간 차이를 측정하여 Accel/Brake 값을 결정 합니다.
        # 각 PID 제어를 위한 Gain 값은 "class pidContorl" 에 정의 되어 있습니다.
        # 각 PID Gain 값을 직접 튜닝하고 아래 수식을 채워 넣어 P I D 제어기를 완성하세요.

        p_control = 
        self.i_control += 
        d_control = 

        output = 
        self.prev_error = 

        '''

        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
