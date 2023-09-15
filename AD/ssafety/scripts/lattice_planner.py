#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

# lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
# 차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
# 충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 경로상의 장애물 탐색
# 3. 좌표 변환 행렬 생성
# 4. 충돌회피 경로 생성
# 5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
# 6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
# 7. 선택 된 새로운 지역경로 (/lattice_path) 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# Lattice Planner 는 격자(Lattice) 점을 이용해 충돌 회피 경로를 만드는 예제 입니다.
# Lattice 경로를 만들기 위해 차량 기준 좌표계를 기준으로 경로의 폭과 완만함 정도를 결정합니다.
# Lattice Path 가 만들어질 경로를 3차 방정식을 이용하여 3 차 곡선을 생성 합니다.
# 위 방식을 이용해 차량이 주행 방향으로 회피가 가능한 여러 Lattice 경로 곡선을 만듭니다.
# 만들어진 경로 중 어떤 경로를 주행해야지 안전하기 주행 할 수 있을 지 판단합니다.
# 아래 예제는 경로 상 장애물의 유무를 판단하고 장애물이 있다면 Lattice Path 를 생성하여 회피 할 수 있는 경로를 탐색합니다.

'''
class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        '''
        #TODO: ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        '''
        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_callback)

        #TODO: (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=10)


        self.is_path = False
        self.is_status = False
        self.is_obj = False

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                # 장애물이 있는지 탐색한다.
                if self.checkObject(self.local_path, self.object_data):
                    # 장애물이 있다면
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #TODO: (7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        #TODO: (2) 경로상의 장애물 탐색
        
        # 경로 상에 존재하는 장애물을 탐색합니다.
        # 경로 상 기준이 되는 지역 경로(local path)에서 일정 거리 이상 가까이 있다면
        # in_crash 변수를 True 값을 할당합니다.

        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:
                dis = sqrt((path.pose.position.x - obstacle.position.x)**2 + (path.pose.position.y - obstacle.position.y)**2)
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break
            
            if is_crash:
                break

        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        '''
        # 충돌 회피 경로를 생성 한 이후 가장 낮은 비용의 경로를 선택 합니다.
        # lane_weight 에는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
        # 이 중 장애물이 있는 차선에는 가중치 값을 추가합니다.
        # 모든 Path를 탐색 후 가장 비용이 낮은 Path를 선택하게 됩니다.
        # 장애물이 존제하는 차선은 가중치가 추가 되어 높은 비용을 가지게 되기 떄문에 
        # 최종적으로 가장 낮은 비용은 차선을 선택 하게 됩니다. 

        '''
        
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))                    
        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

        #print(msg.num_of_npcs)
        #print(msg.num_of_pedestrian)
        #print(msg.num_of_obstacle)
        #for obstacle in msg.obstacle_list:
         #   print(obstacle.position)

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        
        if look_distance < 20 : #min 10m   
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta), -sin(theta), translation[0]], 
                                            [sin(theta), cos(theta), translation[1]], 
                                            [         0,          0,             1 ]])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            # 시작점(global_ref_start_point)을 기준으로 변환한 끝점(global_ref_end_point) 좌표
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            # 시작점(global_ref_start_point)을 기준으로 변환한 자동차(world_ego_vehicle_position) 좌표
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            # [-3.0, -1.75, -1, 1, 1.75, 3.0] [-40.0, -20.75, -10, 10, 20.75, 40.0]
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
                
            for end_point in local_lattice_points:
                # lane_change_3.py의 내용 참고
                waypoints_x=[]
                waypoints_y=[]
                x_interval = 0.5 # 생성할 Path 의 Point 간격을 0.5 로 한다.
                x_start=0
                x_end=end_point[0]

                y_start = 0.0
                y_end = end_point[1]

                x_num = x_end/x_interval
                # End Point 까지의 길이를 Point 간 간격으로 나눠 필요한 Point 의 수를 계산한다.
                # 계산된 Point 의 숫자 만큼 X 좌표를 생성한다.
                for i in range(x_start,int(x_num)) : 
                    waypoints_x.append(i*x_interval)

                # 3차 곡선을 이용한 주행 경로 생성

                # 3 차 방정식을 이용하여 부드러운 차량 거동을 위한 차선변경 경로를 만듭니다.
                # 시작 위치와 목표 위치 사이 부드러운 곡선 경로를 주행하도록 합니다.
                # 차량이 차선 변경 시 알맞을 조향 각도를 조절하여 차선을 변경합니다.
                # 이때 큰 조향각을 가진다면 차량의 횡방향 가속도가 커지고 거동의 안정성이 떨어집니다.
                # 차량이 안정적으로 주행 할 수 있는 경로를 만들어 주는 예제입니다.
                # 
                # 차량이 안정적으로 주행 할 수 있는 경로를 만들기 위해 3차 방정식을 이용합니다.
                # EX )  3차 방정식
                #       
                #       f(x) = a*x^3 + b*x^2 + c*x + d
                # 
                # 위에 예시로 작성한 3차 방정식을 아래 예제에 작성 한다.
                d = 0
                c = 0
                b = 3*(y_end - y_start) / x_end**2
                a = -2*(y_end - y_start) / x_end**3

                for x in waypoints_x:
                    # 3 차 방정식 수식을 작성한다. (f(x) = a*x^3 + b*x^2 + c*x + d)
                    result = a*x**3 + b*x**2 + c*x + d
                    waypoints_y.append(result)
                    

                # ros path 메시지 형식 경로 데이터 생성
                
                # out_path 변수에 정의한 ros path 데이터 형식에 맞춰 경로 데이터를 만든다.
                # Local Result 는 차선 변경 시작 위치 기준 좌표의 Point 정보이고
                # Global Result 는 map 기준 좌표의 Point 좌표 이다.
                # 좌표 변환 행렬을 통해 Local Result 를 이용해 Global Result 를 계산한다.
                # Global Result 는 차선 변경 Path 의 데이터가 된다.

                temp_path = Path()
                temp_path.header.frame_id='/map'
                for i in range(0,len(waypoints_y)) :
                    local_result = np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0.
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1

                    temp_path.poses.append(read_pose)

                out_path.append(temp_path)

            # Add_point            
            # 3 차 곡선 경로가 모두 만들어 졌다면 이후 주행 경로를 추가 합니다.
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            for i in range(len(out_path)):          
                # Ros에 globals라는 함수 내에서 딕셔너리로 기본적으로 제공되는 변수가 있는데 lattice_pub_1라는 키가 있고 
                # 해당 키에 /lattice_path_1 같은 이름으로 publisher를 지정할 수 있는 것 같다.
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])


        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
