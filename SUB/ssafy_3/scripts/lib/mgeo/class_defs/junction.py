#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from collections import OrderedDict

class Junction(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, _id=None):
        self.idx = _id
        self.jc_nodes = list()
        self.connecting_road = dict()


    def add_jc_node(self, node):
        if node not in self.jc_nodes:
            self.jc_nodes.append(node)

        if self not in node.junctions:
            node.junctions.append(self)


    def remove_jc_node(self, node):
        self.jc_nodes.remove(node)


    def get_jc_nodes(self):
        return self.jc_nodes


    def get_jc_node_points(self):
        pts = []
        for node in self.jc_nodes:
            pts.append(node.point)
        
        return pts


    def get_jc_node_indices(self):
        indices = []
        for node in self.jc_nodes:
            indices.append(node.idx)

        return indices

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['jc nodes id'] = {'type' : 'list<string>', 'value' : self.get_jc_node_indices()}

        return prop_data