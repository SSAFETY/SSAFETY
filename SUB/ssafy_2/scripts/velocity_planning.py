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

# velocity_planning 은 차량의 종 횡 방향 제어 예제입니다.
# 차량의 곡률 기반 속도 계획을 세워 주행 경로에 맞는 속도 계획을 할 수 있는 예제 입니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 좌표 변환 행렬 생성
# 3. Steering 각도 계산
# 4. PID 제어 생성
# 5. 도로의 곡률 계산
# 6. 곡률 기반 속도 계획
# 7. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# 자율주행 차량은 정해진 특정 속도로만 주행하는 것이 아닌 내 외부적인 요인을 고려하여 주행합니다.
# 주행 하기 전 돌발 상황이 없다면 미리 어떤 속도로 주행할지 속도 계획을 세우게 됩니다.
# 외부 환경적인 요인인 도로의 곡률을 이용해 차량의 주행 속도를 계산하는 방법을 학습 합니다.
# 곡률이 큰 도로의 경우 곡률에 맞는 속도 계획이 필요합니다.
# 곡률이 큰 도로에서 곡률 최대 속도 보다 큰 속도로 주행하면 차량이 차선을 유지 하며 주행 할 수 없습니다.
# 그렇기 때문에 도로곡률에 맞는 가장 적절한 속도로 맞춰 주행 하기위한 속도 계획이 필요합니다.
# 곡률을 계산하는 방법에는 최소 자승법을 이용한(Pseudo Inverse)곡률 반경 계산 방식, 
# 원의 좌표와 반지름 계산 수식 "(x-a)^2+(y-b)^2=r^2" 을 활용한 행렬 계산 방식등이 있습니다.
# 곡률을 계산하는 방식은 여러 가지가 있기 때문에 원하는 방식을 사용하면됩니다.
# EX)   원의 좌표와 반지름 계산 수식을 이용한 행렬식
#
#       (x-a)^2+(y-b)^2=r^2
#       x^2 + y^2 - 2ax - 2by + a^2 + b^2 - r^2 = 0
#       c = a^2 + b^2 + r^2
#       x^2 + y^2 - 2ax - 2by + c = 0
#       - 2ax - 2by + c = - x^2 - y^2
#       [-2x1,  -2y1,   1][a]   [ -x1^2 - y1^2  ]
#       [    ,      ,    ][b] = [               ]
#       [-2xn,  -2yn,   1][c]   [ -xn^2 - yn^2  ]
#       {행렬 계산을 이용 한다}
#
'''
class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        '''
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        rospy.Subscriber("/global_path" )
        rospy.Subscriber("local_path" )
        rospy.Subscriber("odom" )
        rospy.Subscriber("/Ego_topic" )
        self.ctrl_cmd_pub = 

        '''

        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 1
        self.lfd = 5
        self.target_velocity = 60

        self.pid = pidControl()

        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (7) 제어입력 메세지 Publish
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
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

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

            #TODO: (5) 도로의 곡률 계산
            '''
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.
            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.

            r = 

            '''

            #TODO: (6) 곡률 기반 속도 계획
            '''
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.
            v_max = 

            '''
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
