#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker

class RoadPolygonSet():
    def __init__(self):
        self.data = dict()
        self.key_maker = KeyMaker('RP')

    def append_data(self, RoadPoly, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            for idx in self.data.keys():
                idx = self.key_maker.get_new()
            RoadPoly.idx = idx
        self.data[RoadPoly.idx] = RoadPoly

    def remove_data(self, Poly):
        self.data.pop(Poly.idx)
    """
    def draw_plot(self, axes):
        for idx, Poly in self.data.items():
            Poly.draw_plot(axes)

    def erase_plot(self):
        for idx, Poly in self.data.items():
            Poly.erase_plot()
    """
