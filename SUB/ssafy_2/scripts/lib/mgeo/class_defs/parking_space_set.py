#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker


class ParkingSpaceSet(object):
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('P')

    def append_data(self, parking_space, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.data.keys():
                idx = self.key_maker.get_new()
            parking_space.idx = idx
        self.data[parking_space.idx] = parking_space

    def remove_data(self, parking_space):
        self.data.pop(parking_space.idx)

    def draw_plot(self, axes):
        for idx, scw in self.data.items():
            scw.draw_plot(axes)

    def erase_plot(self):
        for idx, scw in self.data.items():
            scw.erase_plot()