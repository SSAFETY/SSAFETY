#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

import numpy as np 
from class_defs.node_set import NodeSet
from class_defs.node import Node
from class_defs.key_maker import KeyMaker
       

class LineSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.lines = dict()
        self.key_maker = KeyMaker(prefix='LN')

    def append_line(self, line_obj, create_new_key=False):       
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.lines.keys():
                idx = self.key_maker.get_new()
                
            line_obj.idx = idx
            
        self.lines[line_obj.idx] = line_obj
    
    def remove_line(self, line_obj):
        if line_obj.idx in self.lines.keys():
            self.lines.pop(line_obj.idx)

    def draw_plot(self, axes):
        for idx,  line in self.lines.items():
            line.draw_plot(axes)

    def erase_plot(self):
        for idx, line in self.lines.items():
            line.erase_plot()

    def get_ref_points(self):
        ref_points = list()

        for idx, line in self.lines.items():

            # 리스트의 해당 값이 None이면 (prealloc 이후 할당을 안 한 것)
            # Skip한다 (파일로부터 읽어들일 때 1-based indexing을 하면서
            # self.lines[0]을 비워두는 경우들이 있다
            if line == None: 
                continue

            mid_point = int(len(line.points)/2.0)
            
            point_start = line.get_point_dict(0)
            point_mid = line.get_point_dict(mid_point)
            point_end = line.get_point_dict(-1)

            ref_points.append(point_start)
            ref_points.append(point_mid)
            ref_points.append(point_end)

        return ref_points

    def create_node_set_for_all_lines(self, node_set=None, dist_threshold=0.1):
        '''
        각 line의 끝에 node를 생성한다.
        이 때 argument로 전달된 거리값 이내에 다른 선이 존재하면, 같은 node로 판별하고 연결한다 
        '''
        if node_set is None:
            node_set = NodeSet()

        # ref point 함수는 이 함수가 리턴되기까지 동일할 것이므로, 한번만 계산한다
        ref_points = self.get_ref_points()

        for idx, current_link in self.lines.items():
            # Start 방향에 노드가 없으면 생성한다
            if current_link.get_from_node() is None:
                # 만들기
                new_node = Node()
                new_node.point = current_link.points[0]
                node_set.append_node(new_node, create_new_key=True)

                # 현재 선의 from node로 등록
                current_link.set_from_node(new_node)

                # 이 node에 대해 연결될 선이 존재하는가?
                for pts in ref_points:
                    # 현재 체크하는 점이 이 line 위의 점이면 skip
                    if current_link is pts['line_ref']:
                        continue

                    # 현재 체크하는 점이 mid 이면 skip
                    if pts['type'] == 'mid':
                        continue

                    # 근처에 위치한 점이 있는지 체크
                    dist = np.sqrt(
                        (pts['coord'][0] - new_node.point[0])**2 +\
                        (pts['coord'][1] - new_node.point[1])**2)
                    if dist < dist_threshold:
                        if pts['type'] == 'end':
                            # 여기서는 end로 기대되는게 일반적임
                            pts['line_ref'].set_to_node(new_node)
                        else: # start
                            pts['line_ref'].set_from_node(new_node)


            # End 방향에 노드가 없으면 생성한다
            if current_link.get_to_node() == None:
                # 만들기
                new_node = Node()
                new_node.point = current_link.points[-1]
                node_set.append_node(new_node, create_new_key=True)

                # 현재 선의 to node로 등록
                current_link.set_to_node(new_node)

                # 이 node에 대해 연결될 선이 존재하는가?
                for pts in ref_points:
                    # 현재 체크하는 점이 이 line 위의 점이면 skip
                    if current_link is pts['line_ref']:
                        continue

                    # 현재 체크하는 점이 mid 이면 skip
                    if pts['type'] == 'mid':
                        continue

                    # 근처에 위치한 점이 있는지 체크
                    dist = np.sqrt(
                        (pts['coord'][0] - new_node.point[0])**2 +\
                        (pts['coord'][1] - new_node.point[1])**2)
                    if dist < dist_threshold:
                        if pts['type'] == 'start':
                            # 여기서는 start로 기대되는게 일반적임
                            pts['line_ref'].set_from_node(new_node)
                        else: # end 이면, end point에서 만난 것임
                            pts['line_ref'].set_to_node(new_node)
                            
        return node_set

    def set_vis_mode_all_different_color(self, on_off):
        '''
        NOTE: list, dict를 모두 지원하게 만들었으므로, 향후 변경이 필요없다
        '''
        for var in self.lines: # dict 일 경우 key를 반환하는 것
            if isinstance(self.lines, list):
                line = var
            elif isinstance(self.lines, dict):
                line = self.lines[var] 

            line.set_vis_mode_all_different_color(on_off)

    @staticmethod
    def merge_two_sets(setA, setB):
        new_set = LineSet()
    
        setA.lines.update(setB.lines)
        return setA
        # import sys
        # if sys.version_info[0] >= 3:
        #     # TEMP: python2에서 실행하면 실행하기 이전에 오류가 발생한다
        #     # new_set.lines = {**setA.lines, **setB.lines}
        #     # return new_set
        #     pass
        # else:
        #     setA.lines.update(setB.lines.update)
        #     return setA # NOTE: setA를 리턴했을 때 괜찮은지 확인 필요
        
    def merge_line_set(self, a_lines):
        for line in a_lines:
            if line not in self.lines.keys():
                self.lines[line] = a_lines[line]
                self.lines[line].copy_attributes(self.lines[line], a_lines[line])
        return self.lines
