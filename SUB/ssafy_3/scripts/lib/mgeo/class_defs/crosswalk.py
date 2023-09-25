#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

import numpy as np
from collections import OrderedDict


class Crosswalk(object):
    def __init__(self, idx=None):
        super(Crosswalk, self).__init__()
        self.idx = idx
        
        self.single_crosswalk_list = []

        self.ref_traffic_light_list = []

        self.cent_point = [] # 화면에 표시하기 위한 포인트(json 파일에 안들어감)

        self.scw_id_list=[]
        self.tl_id_list=[]
        
        self.scw_dic = dict()
        self.tl_dic = dict()

        
    def get_list_id(self):
        for cw in self.single_crosswalk_list:
            if cw.idx not in self.scw_id_list:
                self.scw_id_list.append(cw.idx)
           
        for tl in self.ref_traffic_light_list:
            if tl.idx not in self.tl_id_list:
                self.tl_id_list.append(tl.idx)

    
    def get_dictionary(self):
        for scw in self.single_crosswalk_list:
            self.scw_dic[scw.idx] = scw
        for tl in self.ref_traffic_light_list:
            self.tl_dic[tl.idx] = tl


    @staticmethod
    def to_dict(self):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
        self.get_list_id()
        self.get_dictionary()

        dict_data = {
            'idx': self.idx,
            
            'single_crosswalk_list': self.scw_id_list,
            'ref_traffic_light_list': self.tl_id_list
            
        }
        return dict_data
    
    @staticmethod
    def from_dict(dict_data, scw_set, tl_set=None, ):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""

        idx = dict_data['idx']
        scw_id_list = dict_data['single_crosswalk_list']
        tl_id_list = dict_data['ref_traffic_light_list']
        
        single_crosswalk_list =[]
        ref_traffic_light_list = []

        tl_dic = dict()
        scw_dic = dict()
        
        if tl_set != None and len(tl_set.signals) > 0:
            for tl_id in tl_id_list:
                ref_traffic_light_list.append(tl_set.signals[tl_id])
                tl_dic[tl_id] = tl_set.signals[tl_id]
        
        if len(scw_set.data) > 0:
            # TODO: error 출력
            pass

        for scw_id in scw_id_list:
            single_crosswalk_list.append(scw_set.data[scw_id])
            scw_dic[scw_id] = scw_set.data[scw_id]

        """STEP #2 인스턴스 생성"""
        obj = Crosswalk(idx)
        obj.single_crosswalk_list = single_crosswalk_list
        obj.ref_traffic_light_list = ref_traffic_light_list
        obj.scw_id_list = scw_id_list
        obj.tl_id_list = tl_id_list
        obj.tl_dic = tl_dic
        obj.scw_dic = scw_dic


        return obj
        
    def append_single_scw_list(self, scw):
        self.single_crosswalk_list.append(scw)
        if scw.idx not in self.single_crosswalk_list:
            self.scw_id_list.append(scw.idx)
        self.scw_dic[scw.idx] = scw

    
    def append_ref_traffic_light(self, tl):
        if tl.dynamic:
            self.ref_traffic_light_list.append(tl)
            if tl.idx not in self.ref_traffic_light_list:
                self.tl_id_list.append(tl.idx)
            self.tl_dic[tl.idx] = tl
        
    
    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['single_crosswalk_list'] = {'type' : 'list<string>', 'value' : self.scw_id_list}
        prop_data['ref_traffic_light_list'] = {'type' : 'list<string>', 'value' :  self.tl_id_list}
        
        return prop_data

    def get_centroid_points(self):
        tl_points = []
        scw_points = []
        if len(self.single_crosswalk_list) > 0 and len(self.ref_traffic_light_list) > 1:
            for tl in self.ref_traffic_light_list:
                tl_points.append(tl.point)

        else:
            if len(self.single_crosswalk_list) > 0:  
                for scw in self.single_crosswalk_list:
                    scw_points.extend(scw.points)
        
        if len(tl_points)>0:
            self.cent_point = self.calculate_centroid(np.array(tl_points))
        elif len(scw_points)>0:
            self.cent_point = self.calculate_centroid(np.array(scw_points))
            

    def calculate_centroid(self, points):
        sx = sy= sz = sL = 0
        for i in range(len(points)):
            x0, y0, z0 = points[i - 1]     # in Python points[-1] is last element of points
            x1, y1, z1 = points[i]
            L = ((x1 - x0)**2 + (y1 - y0)**2 + (z1-z0)**2) ** 0.5
            sx += (x0 + x1)/2 * L
            sy += (y0 + y1)/2 * L
            sz += (z0 + z1)/2 * L
            sL += L
            
        centroid_x = sx / sL
        centroid_y = sy / sL
        centroid_z = sz / sL

        return np.array([centroid_x,centroid_y, centroid_z])