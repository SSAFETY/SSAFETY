#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker

class IntersectionControllerSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.intersection_controllers = dict()
        self.key_maker = KeyMaker('')
        self.key_maker.num = 0


    def append_controller(self, ic_obj, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            key_idx = 'IntTL{}'.format(idx)
            while key_idx in self.intersection_controllers.keys():
                idx = self.key_maker.get_new()
                key_idx = 'IntTL{}'.format(idx)
            ic_obj.idx = key_idx

        self.intersection_controllers[ic_obj.idx] = ic_obj
        
        
    def remove_ic_signal(self, ic_obj):
        self.intersection_controllers.pop(ic_obj.idx)
    
    
    def remove_data(self, ic):
        self.intersection_controllers.pop(ic)
