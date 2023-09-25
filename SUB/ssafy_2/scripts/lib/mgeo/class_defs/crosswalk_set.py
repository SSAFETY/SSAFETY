#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker

class CrossWalkSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('CW')

    def append_data(self, cw, create_new_key=True):
        if create_new_key:
            # idx = ''
            idx = self.key_maker.get_new()
            
            cw.get_centroid_points()
            
            cw.idx = idx

        self.data[cw.idx] = cw

    def remove_data(self, cw):
        for idx in cw.scw_dic.keys():
            cw.scw_dic[idx].remove_ref_crosswalk_id(cw.idx)
        
        for idx in cw.tl_dic.keys():
            cw.tl_dic[idx].remove_ref_crosswalk_id(cw.idx)

        self.data.pop(cw.idx)
    
    def cw_remove_list_data(self, cw):
        self.data.pop(cw)
    
    def isDuplicationCheck(self, new_cw):
        for cw_id in self.data:
            duplicated_scwlist = [a == b for a in self.data[cw_id].single_crosswalk_list for b in new_cw.single_crosswalk_list]
            duplicated_tllist = [a == b for a in self.data[cw_id].ref_traffic_light_list for b in new_cw.ref_traffic_light_list]

            if len(new_cw.ref_traffic_light_list) == 0:
                return False

            if duplicated_scwlist.count(True) == 2 and duplicated_tllist.count(True) == 2:
                return True
        
        return False

