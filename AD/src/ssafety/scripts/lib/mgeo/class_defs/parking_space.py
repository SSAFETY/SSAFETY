#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.surface_marking import SurfaceMarking
from collections import OrderedDict


class ParkingSpace(SurfaceMarking):
    def __init__(self, points=None, idx=None):
        super(ParkingSpace, self).__init__(points, idx)
        self.points = points
        self.center_point = self.calculate_centroid()

        self.parking_type = None
        self.parking_target_type = None
        self.parking_direction = None
        self.distance = 0 # 가장 가까운 링크까지의 거리
        self.width = 2.5 # 일반형 주차장 폭
        self.length = 5 # 일반형 주차장 길이
        self.angle = 90 # 주차장 입구의 진입 각도
        self.linked_left_list_idx = []
        self.linked_right_list_idx = []

    def set_points(self, points):
        super(ParkingSpace, self).set_points(points)

    def getLinkedLeftListIdx(self, link_set):
        return_list = []
        for i in self.linked_left_list_idx:
            link = link_set.lines[i]
            return_list.append(link)
        return return_list

        
    def getLinkedRightListIdx(self, link_set):
        return_list = []
        for i in self.linked_right_list_idx:
            link = link_set.lines[i]
            return_list.append(link)
        return return_list
    
    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist() if type(self.points) != list else self.points }
        prop_data['parking_type'] = {'type' : 'string', 'value' : self.parking_type}
        prop_data['parking_target_type'] = {'type' : 'string', 'value' :  self.parking_target_type}
        prop_data['parking_direction'] = {'type' : 'string', 'value' :  self.parking_direction}
        prop_data['distance'] = {'type' : 'float', 'value' :  self.distance}
        prop_data['width'] = {'type' : 'float', 'value' :  self.width}
        prop_data['length'] = {'type' : 'float', 'value' :  self.length}
        prop_data['angle'] = {'type' : 'float', 'value' :  self.angle}
        prop_data['linked_left_list_idx'] = {'type' : 'list<string>', 'value' :  self.linked_left_list_idx}
        prop_data['linked_right_list_idx'] = {'type' : 'list<string>', 'value' :  self.linked_right_list_idx}
        
        return prop_data

    def to_dict(self):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
    
        dict_data = {
            'idx': self.idx,
            'points': self.pointToList(self.points),
            'center_point':self.pointToList(self.calculate_centroid()),
            'parking_type':self.parking_type,
            'parking_target_type': self.parking_target_type,
            'parking_direction': self.parking_direction,
            'distance': self.distance,
            'width': self.width,
            'length': self.length,
            'angle': self.angle,
            'linked_left_list_idx': self.linked_left_list_idx,
            'linked_right_list_idx': self.linked_right_list_idx,
        }
        return dict_data
    
    @staticmethod
    def from_dict(dict_data, link_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
      
        idx = dict_data['idx']
        points = dict_data['points']
        parking_type = dict_data['parking_type']
        parking_target_type = dict_data['parking_target_type']
        parking_direction = dict_data['parking_direction']
        distance = dict_data['distance']
        width = dict_data['width']
        length = dict_data['length']
        angle = dict_data['angle']
        linked_left_list_idx = dict_data['linked_left_list_idx']
        linked_right_list_idx = dict_data['linked_right_list_idx']

        """STEP #2 인스턴스 생성"""
        obj = ParkingSpace(points, idx)
        obj.parking_type = parking_type
        obj.parking_target_type = parking_target_type
        obj.parking_direction = parking_direction
        obj.distance = distance
        obj.width = width
        obj.length = length
        obj.angle = angle
        obj.linked_left_list_idx = linked_left_list_idx
        obj.linked_right_list_idx = linked_right_list_idx

        return obj
        
    
    def isList(self, val):
        try:
            list(val)
            return True
        except ValueError:
            return False

    def pointToList(self, points):
        return_points = []
        for point in points:
            point_list = point.tolist() if type(point) != list else point
            return_points.append(point_list)
        return return_points