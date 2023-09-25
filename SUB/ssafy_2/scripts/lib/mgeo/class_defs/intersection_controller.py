#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from collections import OrderedDict


class IntersectionController(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, id=None):
        self.idx = id      
        self.point = None
        #self.synced_signal_id_list = []
        #self.synced_signal_set = SyncedSignalSet()
        self.TL = list()
        self.TL_dict = dict()

    def new_synced_signal(self) :
        synced_signal_list = list()
        self.TL.append(synced_signal_list)

    def append_signal(self, signal) :
        if len(self.TL) == 0 :
            return
        synced_signal_list = self.TL[-1]
        synced_signal_list.append(signal.idx)
        self.TL_dict[signal.idx] = signal
        self.point = signal.point

    def get_signal_id_list(self) :
        id_list = []
        for idxs in self.TL:
            id_list.extend(idxs)
        return id_list

    def get_signal_list(self) :
        return self.TL_dict.values()

    def get_intersection_controller_points(self):
        points = list()
        for signal_id in self.TL_dict:
            signal = self.TL_dict[signal_id]
            points.append(signal.point)
        
        return points

    @staticmethod
    def to_dict(obj):
        """json 파일등으로 저장할 수 있는 dict 데이터로 변경한다"""

        to_list = []
        for i in obj.TL:
            to_list.append(list(i))
        dict_data = {
            'idx': obj.idx,
            'TL': to_list
        }

        return dict_data


    @staticmethod
    def from_dict(dict_data, light_set):
        """json 파일등으로부터 읽은 dict 데이터에서 IntersectionController 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']

        # 연결된 객체 참조용 정보
        signal_id_list = dict_data['TL']
        
        """STEP #2 인스턴스 생성"""
        # 필수 정보
        obj = IntersectionController(idx)
        obj.TL = signal_id_list

        """STEP #3 인스턴스 메소드 호출해서 설정할 값들 설정하기"""       
        if light_set is not None:
            for synced_signal_list in signal_id_list:
                for synced_signal_id in synced_signal_list :
                    if synced_signal_id in light_set.signals :
                        obj.TL_dict[synced_signal_id] = light_set.signals[synced_signal_id]

        keys = list(obj.TL_dict.keys())
        if len(keys) > 0 :
            obj.point = obj.TL_dict[keys[0]].point

        return obj

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['TL'] = {'type' : 'list<list<string>>', 'value' : self.TL}

        return prop_data


    def is_out_of_xy_range(self, xlim, ylim):
        """NOTE: XY 축에 대해서만 확인한다"""
        return_bool = True
        for signal_id in self.TL_dict:
            return_bool = return_bool and self.TL_dict[signal_id].is_out_of_xy_range(xlim, ylim)
        return return_bool

