#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../'))) # mgeo가 있는 경로를 추가한다.

from class_defs import *
from utils.version import Version

import numpy as np
import json


def load_node_and_link(node_save_info_list, line_save_info_list, global_info):
    file_ver = Version(global_info['maj_ver'], global_info['min_ver'])
    # load 기본 타입은 dict 이다.

    # node_save_info_list에서 node_set 생성
    node_set = NodeSet()
    link_set = LineSet()
    junction_set = JunctionSet()

    # 노드 생성하기
    for save_info in node_save_info_list:
        idx = save_info['idx']
        point = save_info['point']
        try:
            node_type = save_info['node_type']
        except:
            node_type = None

        try:
            on_stop_line = save_info['on_stop_line']
        except:
            on_stop_line = None
        
        node = Node(idx)
       
        node.point = np.array(point)
        node.node_type = node_type
        node.on_stop_line = on_stop_line

        # 교차로 생성하기 (노드 생성하면서 같이 수행)
        if file_ver >= Version(2,5):
            junction_list = save_info['junction']

            if junction_list is None:
                continue
            elif len(junction_list) == 0:
                node.junctions = list()
            else:
                for junction_id in junction_list:
                    if junction_id in junction_set.junctions.keys():
                        repeated_jc = junction_set.junctions[junction_id]
                        repeated_jc.add_jc_node(node)
                    else:
                        new_junction = Junction(junction_id)
                        new_junction.add_jc_node(node)

                        junction_set.append_junction(new_junction)
        
        elif file_ver >= Version(2,3):
            junction_id = save_info['junction']

            if junction_id is not None:
                if junction_id in junction_set.junctions.keys():
                    repeated_jc = junction_set.junctions[junction_id]
                    repeated_jc.add_jc_node(node)
                else:
                    new_junction = Junction(junction_id)
                    new_junction.add_jc_node(node)

                    junction_set.append_junction(new_junction)
                    

        node_set.append_node(node, create_new_key=False)

    # 링크 생성하기
    for save_info in line_save_info_list:
        idx = save_info['idx']
        from_node = node_set.nodes[save_info['from_node_idx']] if save_info['from_node_idx'] in node_set.nodes else None
        to_node = node_set.nodes[save_info['to_node_idx']] if save_info['to_node_idx'] in node_set.nodes else None
        points = save_info['points']
        lazy_init = save_info['lazy_init']
        link_type = save_info['link_type']
        try:
            force_width_start = save_info['force_width_start']
            width_start = save_info['width_start']
            force_width_end = save_info['force_width_end']
            width_end = save_info['width_end']
            enable_side_border = save_info['enable_side_border']
        except:
            force_width_start, width_start, force_width_end, width_end = Link.get_default_width_related_values()
            enable_side_border = False

        # 우선 위 값만 가지고 링크를 먼저 세팅한다
        link = Link(idx=idx, lazy_point_init=lazy_init)
        link.set_from_node(from_node)
        link.set_to_node(to_node)
        link.set_width_related_values(force_width_start, width_start, force_width_end, width_end)
        
        if type(points[0][0]) == str:
            link.set_points(np.array([[float(x) for x in y] for y in points]))
        else:
            link.set_points(np.array(points))

        # link.set_points(np.array(points))
        link.link_type = link_type
        link.enable_side_border = enable_side_border


        # 버전에 따라 체크할 필요가 없이 그냥 key 값의 유무로 체크한다. 
        
        # 버전 2.2에서 추가된 데이터
        if 'max_speed' in save_info:
            link.set_max_speed_kph(save_info['max_speed'])


        # 버전 2.4에서 추가된 데이터
        if 'road_id' in save_info:
            link.road_id = save_info['road_id']
        
        if 'ego_lane' in save_info:
            link.ego_lane = save_info['ego_lane']
        
        if 'lane_change_dir' in save_info:
            link.lane_change_dir = save_info['lane_change_dir']
        
        if 'hov' in save_info:
            link.hov = save_info['hov']
        

        # 버전 2.6에서 추가된 데이터
        if 'geometry' in save_info:
            link.geometry = save_info['geometry']


        # 아래 데이터는 버전 트래킹 안 되어 있음
        if 'can_move_left_lane' in save_info:
            link.can_move_left_lane = save_info['can_move_left_lane']

        if 'can_move_right_lane' in save_info:
            link.can_move_right_lane = save_info['can_move_right_lane']

        if 'road_type' in save_info:
            link.road_type = save_info['road_type']

        if 'related_signal' in save_info:
            link.related_signal = save_info['related_signal']

        if 'its_link_id' in save_info:
            link.its_link_id = save_info['its_link_id']
        
        if 'lane_mark_left' in save_info:
            link.lane_mark_left = save_info['lane_mark_left']
        
        if 'lane_mark_right' in save_info:
            link.lane_mark_right = save_info['lane_mark_right']

        if 'link_type_def' in save_info:
            link.link_type_def = save_info['link_type_def']
        
        # oppTraffic -> opp_traffic
        if 'opp_traffic' in save_info:
            link.opp_traffic = save_info['opp_traffic']
        elif 'oppTraffic' in save_info:
            link.opp_traffic = save_info['oppTraffic']
        else:
            link.opp_traffic = False

        if 'is_entrance' in save_info:
            link.is_entrance = save_info['is_entrance']
        
        if 'is_exit' in save_info:
            link.is_exit = save_info['is_exit']

        if 'speed_unit' in save_info:
            link.speed_unit = save_info['speed_unit']

        # speed_start -> speed_offset으로 변경
        if 'speed_offset' in save_info:
            link.speed_offset = save_info['speed_offset']
        elif 'speed_start' in save_info:
            link.speed_offset = save_info['speed_start']
        else:
            link.speed_offset = []

        if 'speed_list' in save_info:
            link.speed_list = save_info['speed_list']
        else:
            link.speed_list = dict()

        if 'recommended_speed' in save_info:
            link.recommended_speed = save_info['recommended_speed']
        else:
            link.recommended_speed = 0

        link_set.append_line(link, create_new_key=False)
    

    for save_info in line_save_info_list:
        idx = save_info['idx']
        link = link_set.lines[idx]

        # 각 링크에 대해 다음을 설정
        if not link.is_it_for_lane_change():
            # 차선 변경이 아닐 경우, 차선 변경으로 진입 가능한 링크를 설정
            if save_info['left_lane_change_dst_link_idx'] is not None:
                dst_link = link_set.lines[save_info['left_lane_change_dst_link_idx']]
                link.set_left_lane_change_dst_link(dst_link)
                if link.link_type in ['1', '2', '3']:
                        link.can_move_left_lane = False
                # tomtom 데이터에 NON_DRIVABLE_LANE, EMERGENCY_LANE...
                elif link.link_type in ['DRIVABLE_LANE', 'NON_DRIVABLE_LANE', 'EMERGENCY_LANE']:
                        pass
                else:
                    link.can_move_left_lane = True

            if save_info['right_lane_change_dst_link_idx'] is not None:
                dst_link = link_set.lines[save_info['right_lane_change_dst_link_idx']]
                link.set_right_lane_change_dst_link(dst_link)
                if link.link_type in ['1', '2', '3']:
                        link.can_move_right_lane = False
                # tomtom 데이터에 NON_DRIVABLE_LANE, EMERGENCY_LANE...
                elif link.link_type in ['DRIVABLE_LANE', 'NON_DRIVABLE_LANE', 'EMERGENCY_LANE']:
                        pass
                else:
                    link.can_move_right_lane = True

        else:
            # 차선 변경일 경우, 

            # 우선 인덱스로 표시된 lane_ch_link_path를 link에 대한 reference로 변경
            lane_ch_link_path_idx = save_info['lane_ch_link_path']
            lane_ch_link_path = []
            for idx in lane_ch_link_path_idx:
                lane_ch_link_path.append(link_set.lines[idx])

            # 이 값을 통해서 link 내부 값 설정
            link.set_values_for_lane_change_link(lane_ch_link_path)
    
    
    # 모든 링크에 대한 cost 계산
    for key, link in link_set.lines.items():
        link.calculate_cost()

    for node_id in node_set.nodes:
        cnode = node_set.nodes[node_id]
        from_links = cnode.from_links
        new_from_links = list()
        for fl in from_links:
            if fl.idx not in link_set.lines:
                continue
            new_from_links.append(fl)
        cnode.from_links = new_from_links

        to_links = cnode.to_links
        new_to_links = list()
        for tl in to_links:
            if tl.idx not in link_set.lines:
                continue
            new_to_links.append(tl)
        cnode.to_links = new_to_links

    return node_set, link_set, junction_set