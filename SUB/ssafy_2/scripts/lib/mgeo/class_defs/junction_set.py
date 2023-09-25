#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker


class JunctionSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.junctions = dict()
        self.key_maker = KeyMaker(prefix='1')

    def append_junction(self, junction, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.junctions.keys():
                idx = self.key_maker.get_new()

            junction.idx = idx
            

        self.junctions[junction.idx] = junction


    def merge_junction_set(self, a_junctions):
        for junction in a_junctions:
            if junction in self.junctions.keys():
                for node in a_junctions[junction].jc_nodes:
                    if node.idx not in self.junctions[junction].get_jc_node_indices():
                        self.junctions[junction].add_jc_node(node)
            else:
                self.junctions[junction] = a_junctions[junction]
                for node in a_junctions[junction].jc_nodes:
                    self.junctions[junction].add_jc_node(node)
                    
        return self.junctions