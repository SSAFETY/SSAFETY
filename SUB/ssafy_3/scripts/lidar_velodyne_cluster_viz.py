#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray,Pose, Point32
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,ObjectStatus
import sensor_msgs.point_cloud2 as pc2

from tf.transformations import euler_from_quaternion,quaternion_from_euler


# Lida Velodyne Cluster Viz Node 는 Lida Velodyne Cluster Node 에서 송신하는 장애물 정보를 받아 시각화 하는 예제입니다.
# Lida Velodyne Cluster Node 에서 DBSCAN 을 통해 장애물의 위치 정보를 얻을 수 있습니다.
# 이번 예제에서는 수신 받은 DBSCAN 장애물 정보를 시각화를 위한 Point Cloud 데이터와 
# 시뮬레이터에서 나오는 morai 메시지 ObjectStatus 메시지 형식으로 변경하여 다시 ros 형식으로 송신합니다.

# 노드 실행 순서 
# 1. 좌표 변환 행렬 생성 
# 2. PointCloud와 ObjectStatus 형식에 맞춘 메시지 데이터 생성
# 3. PointCloud와 ObjectStatus 메시지 송신

class Cluster_viz:

    def __init__(self):

        rospy.Subscriber("/clusters", PoseArray, self.callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.object_pointcloud_pub = rospy.Publisher('object_pointcloud_data',PointCloud, queue_size=1)
        self.object_data_pub = rospy.Publisher('Object_topic_to_lidar',ObjectStatusList, queue_size=1)

        self.is_odom = False
        self.cluster_status = False

        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            if self.is_odom == True and self.cluster_status == True:

                #TODO: (1) 좌표 변환 행렬 생성
                '''
                trans_matrix = np.array([
                                            [       ,       ,       ],
                                            [       ,       ,       ],
                                            [0,0,1]])

                '''
                obj_data=PointCloud()
                obj_data.header.frame_id='map'

                cluster_obj_data = ObjectStatusList()

                cluster_obj_data.num_of_npcs = len(self.cluster_data.poses)
                cluster_obj_data.num_of_obstacle = len(self.cluster_data.poses)

                for num,i in enumerate(self.cluster_data.poses) :

                    #TODO: (2) PointCloud와 ObjectStatus 형식에 맞춘 메시지 데이터 생성
                    '''
                    local_result = 
                    global_result = 

                    tmp_point=Point32()
                    tmp_point.x = 
                    tmp_point.y = 
                    tmp_point.z = 1.
                    obj_data.points.append(tmp_point)

                    cluster_obj_data_npc = ObjectStatus()
                    cluster_obj_data_npc.type = 1
                    cluster_obj_data_npc.position.x = 
                    cluster_obj_data_npc.position.y = 
                    cluster_obj_data_npc.position.z = 1.
                    cluster_obj_data.npc_list.append(cluster_obj_data_npc)

                    cluster_obj_data_obstacle = ObjectStatus()
                    cluster_obj_data_obstacle.type = 2
                    cluster_obj_data_obstacle.position.x = 
                    cluster_obj_data_obstacle.position.y = 
                    cluster_obj_data_obstacle.position.z = 1.
                    cluster_obj_data.obstacle_list.append(cluster_obj_data_obstacle)

                    '''

                #TODO: (3) 3. PointCloud와 ObjectStatus 메시지 송신

                self.object_pointcloud_pub.publish(obj_data)
                self.object_data_pub.publish(cluster_obj_data)

            rate.sleep()

    def callback(self, msg):    
        self.cluster_data = msg

        self.cluster_status = True

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.status_msg=msg    

        self.vehicle_yaw = self.status_msg.heading/180*math.pi
        self.vehicle_pos_x = self.status_msg.position.x
        self.vehicle_pos_y = self.status_msg.position.y

        self.is_status = True

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.vehicle_pos_x=msg.pose.pose.position.x
        self.vehicle_pos_y=msg.pose.pose.position.y

if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    Cluster_visualazation = Cluster_viz()

    rospy.spin() 
