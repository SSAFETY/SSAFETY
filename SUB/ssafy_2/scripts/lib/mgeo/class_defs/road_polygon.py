#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from collections import OrderedDict

from class_defs.base_plane import BasePlane

class RoadPolygon(BasePlane):
    def __init__(self, points=None, faces=None, uv=None, type="road", idx=None):
        super(RoadPolygon, self).__init__(points, idx)

        self.points = points
        self.faces = faces
        self.uv = uv
        if self.uv is None :
            self.uv = list()
        self.type = type

    def set_points(self, points):
        super(RoadPolygon, self).set_points(points)

    def set_faces(self, faces):
        self.faces = faces

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['type'] = {'type' : 'string', 'value' : self.type }
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist() if type(self.points) != list else self.points }
        prop_data['faces'] = {'type' : 'list<list<int>>', 'value' : self.faces.tolist() if type(self.faces) != list else self.faces }
        prop_data['uv'] = {'type' : 'list<list<float>>', 'value' : self.uv.tolist() if type(self.uv) != list else self.uv }

        return prop_data

    @staticmethod
    def to_dict(obj):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
        for i, point in enumerate(obj.points):
            if type(point) == tuple:
                obj.points[i] = list(point)
            elif type(point) != list:
                obj.points[i] = point.tolist()

        dict_data = {
            'idx': obj.idx,
            'type':obj.type,
            'points': obj.points.tolist() if type(obj.points) != list else obj.points,
            'faces': obj.faces.tolist() if type(obj.faces) != list else obj.faces,
            'uv': obj.uv.tolist() if type(obj.uv) != list else obj.uv,
        }
        return dict_data
    
    @staticmethod
    def from_dict(dict_data):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
      
        idx = dict_data['idx']
        type = dict_data['type']
        points = dict_data['points']
        faces = dict_data['faces']
        uv = dict_data['uv']
        """STEP #2 인스턴스 생성"""
        obj = RoadPolygon(points, faces, uv, type, idx)

        return obj
