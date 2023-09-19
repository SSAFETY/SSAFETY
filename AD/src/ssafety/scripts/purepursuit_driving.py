#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 차량 종/횡 방향 제어에 대한 코드
# Purpusuit 알고리즘의 Look Ahead Distance(전방 주시 거리) 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올린다.
# 횡방향 제어 입력은 주행할 Local Path(지역경로)와 Odometry(차량의 상태 정보)를 받아 차량을 제어
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#========================================================#
from morai_msgs.msg import ObjectStatusList, GetTrafficLightStatus
from morai_msgs.msg import EventInfo, Lamps
# from morai_msgs.srv import MoraiEventCmdSrv
#========================================================#

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# Look Ahead Distance 값을 현재 속도에 비례하여 설정해서 최대/최소 값을 정함
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경한다.
'''

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        '''
        #TODO: ros Launch File <arg> Tag 
        # ros launch 파일에 arg 태그 이용하여 변수 정의 가능        
        '''
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        # arg 값으로 받은 변수(local path)로 사용한다 - L61 대체
        rospy.Subscriber(local_path_name, Path, self.path_callback)

        #TODO: (1) subscriber, publisher 선언
        # Subscriber; Local Path, Global Path & Odometry Ego Status
        # Publisher; CtrlCmd(User -> Simul)
        # Ego topic 데이터는 차량의 "현재 속도"를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        # rospy.Subscriber("/local_path", Path, self.path_callback)     # L58로 대체
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # 현재 신호등 정보 받기 - (trafficLightType, trafficLightStatus)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)

        # User -> Simul; 모터 동작 메시지
        self.ctrl_cmd_msg = CtrlCmd()
        # 제어 방식을 결정하는 인덱스( 1: Throttle control, 2: Velocity control, 3: Acceleration control )
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        #========================================================#
        # 신호등에 대한 데이터
        self.is_traffic_light = False

        self.traffic_direction = [] # 지금 지나갈 수 있는 링크 속성 (직진, 좌회전, 우회전)
        #========================================================#


        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        # 전방 주시 거리(lfd, look forward distance)
        self.lfd = 10
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 40

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()         # time gap - use for pid control
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                
                #  횡방향 제어 - steering = theta = calc_pure_pursuit
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                
                # 종방향 제어 - pid(타겟 속도, 현재 속도)
                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (8) 제어입력 메세지 Publish
                
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self, msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self, msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True


    def traffic_light_callback(self, data):
        self.is_traffic_light = True
        traffic_light_status = data.trafficLightStatus

        # print('신호등 상태: ', traffic_light_status)
        # print("신호등 상태: ", traffic_light_status)
        # rospy.loginfo("self 값:", self.is_traffic_light)
        rospy.loginfo("신호등 상태: %d", traffic_light_status)

        # 신호등 상태에 따라 교통 방향 제시
        if traffic_light_status == 1:   # Red
            self.traffic_direction = []

    
    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2) + pow(dy,2))
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self, ):

        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정        
        # 차량 속도에 비례한 전방주시거리(Look Forward Distance) 수식 구현 - 가변값
        # 이때 'self.lfd' 값은 최소(self.min_lfd) 값과 최대(self.max_lfd) 값 사이로, self.lfd_gain 까지 미리 정의
        # 최소 최대 전방주시거리(Look Forward Distance) 값과 속도에 비례한 lfd_gain 값을 직접 변경 가능

        self.lfd = self.lfd_gain * min(self.max_lfd, max(self.min_lfd, self.status_msg.velocity.x))
        rospy.loginfo(self.lfd)
        
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        # translation - 변환 좌표 ???
        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성

        # path 데이터 -> 현재 차량 기준(ego_topic) 좌표계로 좌표 변환 필요  for Pure Pursuit Algorithm
        # 좌표계 변환 후 전방주시거리(lfd, Look Forward Distance)와 가장 가까운 Path Point 를 찾는다. -> 조향 각도 계산
        # 좌표 변환 행렬을 이용해 paht 데이터를 차량 기준 좌표계로 바꾸는 반복문을 이용해 lfd 와 가까운 path point 계산 반복


        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), 0],
                                 [0                    , 0                     , 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for idx, pose in enumerate(self.path.poses):
            path_point = pose.pose.position
            
            global_path_point = [path_point.x - translation[0], path_point.y - translation[1], 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            # 후진하지 않기 위해서 - local_path_point 는 무조건 양수
            if local_path_point[0] > 0:
                dis = (local_path_point[0]**2 + local_path_point[1]**2)**0.5

                if dis >= self.lfd:
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (4) Steering 각도 계산

        # 위에서 구한 lfd와 가장 가까운 Path Point 좌표의 각도를 계산한다.
        # Streeing 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용한 조향 각도

        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        try:
            theta = atan2(self.forward_point[1], self.forward_point[0])
        except:
            theta = 0
            
        steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        #TODO: (5) PID 제어 생성 - 종방향 제어

        # 현재와 목표 속도 갭
        error = target_vel - current_vel

        p_control = self.p_gain * error         # p 제어
        self.i_control += error * self.controlTime    # 누적 속도(i) 차이 - 에러 값들의 총합
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime  # 미분(d) 값

        # PID 제어 반영
        output = p_control + self.i_gain*self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.
            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.

            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)
            
            r = (a**2 + b**2 - c)**0.5


            #TODO: (7) 곡률 기반 속도 계획
            
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.

            v_max = (r * 9.8 * 0.8) ** 0.5


            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass

