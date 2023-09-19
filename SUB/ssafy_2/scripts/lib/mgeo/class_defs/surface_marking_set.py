#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker


class SurfaceMarkingSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('SM')

    def append_data(self, sm, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.data.keys():
                idx = self.key_maker.get_new()

            sm.idx = idx

        self.data[sm.idx] = sm

    def remove_data(self, sm):
        self.data.pop(sm.idx)

    def draw_plot(self, axes):
        for idx, sm in self.data.items():
            sm.draw_plot(axes)

    def erase_plot(self):
        for idx, sm in self.data.items():
            sm.erase_plot()