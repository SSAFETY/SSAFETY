#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

class ConnectingRoad(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, _idx=None):
        self.idx = _idx
        self.connecting = None
        self.incoming = None
        self.from_lanes = list()
        self.to_lanes = list()

    def add_lanes(self, lane_id):
        self.from_lanes.append(lane_id)

    def get_lanes(self):
        return self.from_lanes