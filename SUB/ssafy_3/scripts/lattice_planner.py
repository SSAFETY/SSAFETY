#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
from morai_msgs.msg  import EgoVehicleStatus, ObjectStatusList
from geometry_msgs.msg import Point, PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

# lattice_planner - lane change 에서 발전시켜 충돌 회피 경로 생성 및 선택
# 차량 경로 상의 장애물을 탐색해 지역 경로(/local_path)와 장애물 정보(/Object_topic)를 받아 판단
# L 그렇다면 차량 변경(lane_change) 와 함께 만들 수 있을 거 같은데?
# 만약 회피 경로를 생성 및 선택한다면 새로운 지역 경로(/lattice_path)를 생성
# L 충돌 안 하면 local_path 는 어떻게 됌? 그대로 ? ... 였던 거 같긴 함


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

        # ros Launch File <arg> Tag - Object 정보
        arg = rospy.myargv(argv= sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_callback)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size= 10)

        self.is_path = False
        self.is_status = False
        self.is_obj = False

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #TODO: (7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        #TODO: (2) 경로상의 장애물 탐색
        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:
                dis = sqrt((path.pose.position.x - obstacle.position.x)**2 + (path.pose.position.y - obstacle.position.y)**2)
                if dis < 2.5:       # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.5 미만일때 충돌이라 판단.
                    is_crash = True
                    break

            # for 문 2개 -> 이거 위에서 return 해줘도 될 거 같은데?
            if is_crash:
                break

        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 가중치 가장 낮은 경로 선택

        selected_lane = -1
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path - 좌/우 6개

        for obstacle in object_data.obstacle_list:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
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

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        if look_distance < 20 : #min 10m
            look_distance = 20

        if len(ref_path.poses) > look_distance :
            #TODO: (3) 좌표 변환 행렬 생성 - 시작 지점 Point 에서 끝 지점 Point 상대 위치 계산

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix        = np.array([    [cos(theta),    -sin(theta),    translation[0]], 
                                                [sin(theta),     cos(theta),    translation[1]], 
                                                [         0,              0,                1 ] ])

            det_trans_matrix    = np.array([    [trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                                [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                                [                 0,                  0,                                                                            1]  ])

            # 시작점 기준으로 변환된 경로 끝 Point 좌표
            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            # 시작점 기준으로 변환된 자동차 좌표
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            # local 좌표계로 변경 후 3차 곡선을 만들고 다시 Map 좌표계로 가져온다.
            # lane_change_3.py 의 L308~ 참고
            # 생성된 lattice_path 는 out_path 변수에 List 형식으로 넣는다.

            for end_point in local_lattice_points :
                waypoints_x = []
                waypoints_y = []
                x_interval = 0.5 # 생성할 Path 의 Point 간격을 0.5 로 한다.
                x_start = 0
                x_end = end_point[0]

                y_start = 0.0
                y_end = end_point[1]

                x_num = x_end / x_interval
                # End Point 까지의 길이를 Point 간 간격으로 나눠 필요한 Point 수를 계산한다.
                # 계산된 Point 숫자 만큼 X 좌표를 생성한다.
                for i in range(x_start, int(x_num)):
                    waypoints_x.append(i*x_interval)

                #TODO: 3차 곡선을 이용한 주행 경로 생성
                # 3차 방정식을 이용한 차선변경 경로 생성
                # 안정적으로 주행할 수 있는 조향각 계산
                # 이때 큰 조향각을 가진다면 차량의 횡방향 가속도가 커지고 거동의 안정성이 떨어짐

                d = 0
                c = 0
                b = 3*(y_end-y_start) / x_end**2
                a = -2*(y_end-y_start) / x_end**3

                for i in waypoints_x:
                    # 3 차 방정식 수식을 작성한다. (f(x) = a*x^3 + b*x^2 + c*x + d)
                    result = a * i**3 + b * i**2 + c * i + d
                    waypoints_y.append(result)

                #TODO: ros path 메시지 형식 경로 데이터 생성
                # local Result는 차선 변경 시작 위치 기준 좌표의 Point 정보이고
                # Global Result는 map 기준 좌표 Point 좌표이다.

                # Local Result 는 차선 변경 시작 위치 기준 좌표의 Point 정보이고
                # Global Result 는 map 기준 좌표의 Point 좌표 이다.
                # 좌표 변환 행렬을 통해 Local Result 를 이용해 Global Result 를 계산한다.
                # Global Result 는 차선 변경 Path 의 데이터가 된다.

                temp_path = Path()
                temp_path.header.frame_id = '/map'
                for i in range(0,len(waypoints_y)) :
                    local_result = np.array([[waypoints_x[i]], [waypoints_y[i]],[1]])
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
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass
