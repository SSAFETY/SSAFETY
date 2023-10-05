#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
from ssafety.msg import global_data

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_pub 은 Mgeo 데이터를 읽어온 뒤 도로 정보를 Point Cloud Data 로 변환하는 예제입니다.
# Point Cloud 형식으로 변환 후 Rviz 를 이용해 정밀도로지도 데이터를 시각화 할 수 있습니다.

# 노드 실행 순서 
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. Link 정보 Point Cloud 데이터로 변환
# 3. Node 정보 Point Cloud 데이터로 변환
# 4. 변환한 Link, Node 정보 Publish

class get_mgeo :
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.link_pub = rospy.Publisher('/link', PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('/node', PointCloud, queue_size=1)

        self.global_data_pub = rospy.Publisher('/global_data', global_data, queue_size=1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.link_msg = self.getAllLinks()
        self.node_msg = self.getAllNode()

        # print('# of nodes: ', len(node_set.nodes))
        # print('# of links: ', len(link_set.lines))

        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():

            #TODO: (4) 변환한 Link, Node 정보 Publish
            # 변환한 Link, Node 정보 를 전송하는 publisher 를 만든다.
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)

            global_data_msg = global_data()
            global_data_msg.nodes_idx = list(self.nodes.keys())
            global_data_msg.links_idx = list(self.links.keys())

            # Publish global_data_msg
            self.global_data_pub.publish(global_data_msg)

            rate.sleep()


    def getAllLinks(self):
        # PointCloud 선언 및 초기화
        all_link = PointCloud()
        all_link.header.frame_id = 'map'

        #TODO: (2) Link 정보 Point Cloud 데이터로 변환
        all_link.serialize
        for link_idx in self.links:
            for link_point in self.links[link_idx].points:
                tmp_point = Point32()
                tmp_point.x = link_point[0]
                tmp_point.y = link_point[1]
                tmp_point.z = link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node = PointCloud()
        all_node.header.frame_id = 'map'

        #TODO: (3) Node 정보 Point Cloud 데이터로 변환
        for node_idx in self.nodes:
            tmp_point = Point32()
            tmp_point.x = self.nodes[node_idx].point[0]
            tmp_point.y = self.nodes[node_idx].point[1]
            tmp_point.z = self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node


if __name__ == '__main__':
    test_track = get_mgeo()
