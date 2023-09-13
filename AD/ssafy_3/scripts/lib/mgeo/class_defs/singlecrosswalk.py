#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))


from class_defs.surface_marking import SurfaceMarking
from collections import OrderedDict

class SingleCrosswalk(SurfaceMarking):
    def __init__(self, points=None, idx=None, cw_type=None):
        super(SingleCrosswalk, self).__init__(points, idx)

        self.orign_points = [] # 원본 points
        self.points = points
        self.sign_type = cw_type
        self.ref_crosswalk_id = ''
        self.link_id_list = []
    
    def remove_ref_crosswalk_id(self, id):
        if self.ref_crosswalk_id == id:
            self.ref_crosswalk_id = ''

    def set_points(self, points):
        super(SingleCrosswalk, self).set_points(points)
    
    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist() if type(self.points) != list else self.points }
        prop_data['sign_type'] = {'type' : 'string', 'value' : self.sign_type}
        prop_data['ref_crosswalk_id'] = {'type' : 'string', 'value' :  self.ref_crosswalk_id}
        
 
        return prop_data

    def to_dict(self):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
    
        dict_data = {
            'idx': self.idx,
            'points': self.pointToList(self.points),
            'sign_type':self.sign_type,
            'ref_crosswalk_id': self.ref_crosswalk_id,
            'link_id_list': self.link_id_list,
        }
        return dict_data
    
    @staticmethod
    def from_dict(dict_data, link_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
      
        idx = dict_data['idx']
        points = dict_data['points']
        sign_type = dict_data['sign_type']
        ref_crosswalk_id = dict_data['ref_crosswalk_id']
        link_id_list = []
        if 'link_id_list' in dict_data:
            link_id_list = dict_data['link_id_list']
        

        """STEP #2 인스턴스 생성"""
        obj = SingleCrosswalk(points, idx)
        obj.ref_crosswalk_id = ref_crosswalk_id
        obj.sign_type = sign_type
        obj.link_id_list = link_id_list

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
   
