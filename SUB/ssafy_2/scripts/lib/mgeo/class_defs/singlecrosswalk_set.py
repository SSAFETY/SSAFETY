#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker


class SingleCrosswalkSet():
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('CW')
        self.ref_crosswalk_id = ''

    def append_data(self, scw, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.data.keys():
                idx = self.key_maker.get_new()
            scw.idx = idx
        self.data[scw.idx] = scw

    def remove_data(self, scw):
        self.data.pop(scw.idx)

    def draw_plot(self, axes):
        for idx, scw in self.data.items():
            scw.draw_plot(axes)

    def erase_plot(self):
        for idx, scw in self.data.items():
            scw.erase_plot()

    def get_singlecrosswalk_contain_crosswalkid(self, cw_id):
        scw_list = []
        for idx, scw in self.data.items():
            if scw.ref_crosswalk_id == cw_id:
                scw_list.append(idx)
        
        return scw_list