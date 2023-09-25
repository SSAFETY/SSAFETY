#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.signal_set import SignalSet

import numpy as np

from collections import OrderedDict

class SyncedSignal(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, id=None):
        self.idx = id
        self.point = None
        self.link_id_list = []
        self.intersection_controller_id = None
        self.signal_id_list = []
        self.signal_set = SignalSet()


    def get_signal_set(self):
        return self.signal_set


    def get_synced_signal_points(self):
        points = []
        for signal_id in self.signal_set.signals:
            points.append(self.signal_set.signals[signal_id].point)
        
        return points


    @staticmethod
    def to_dict(obj):
        """json 파일등으로 저장할 수 있는 dict 데이터로 변경한다"""

        dict_data = {
            'idx': obj.idx,
            'link_id_list': obj.link_id_list,          
            'point': np.array(obj.get_synced_signal_points()).tolist(),
            'intersection_controller_id': obj.intersection_controller_id,
            'signal_id_list': obj.signal_id_list,    
        }

        return dict_data


    @staticmethod
    def from_dict(dict_data, link_set=None, tl_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 SyncedSignal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']
        point = dict_data['point']

        # 연결된 객체 참조용 정보
        link_id_list = dict_data['link_id_list']
        signal_id_list = dict_data['signal_id_list']
        
        """STEP #2 인스턴스 생성"""
        # 필수 정보
        obj = SyncedSignal(idx)
        obj.point = np.array(point)
        obj.intersection_controller_id = dict_data['intersection_controller_id']

        # 연결된 객체 참조용 정보
        obj.link_id_list = link_id_list
        obj.signal_id_list = signal_id_list

        """STEP #3 인스턴스 메소드 호출해서 설정할 값들 설정하기"""       
        if tl_set is not None:
            for signal_id in signal_id_list:
                if signal_id in tl_set.signals.keys():
                    signal = tl_set.signals[signal_id]
                    obj.signal_set.append_signal(signal)

        return obj

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['point'] = {'type' : 'list<float>', 'value' : self.get_synced_signal_points()}
        prop_data['intersection_controller_id'] = {'type' : 'string', 'value' : self.intersection_controller_id}
        prop_data['signal_id_list'] = {'type' : 'list<string>', 'value' : self.signal_id_list}

        return prop_data