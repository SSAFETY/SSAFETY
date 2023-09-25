#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.key_maker import KeyMaker
       

class NodeSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.nodes = dict()
        self.key_maker = KeyMaker('ND')

    def append_node(self, node_obj, create_new_key=False):
        if create_new_key:
            idx = self.key_maker.get_new()
            while idx in self.nodes.keys():
                idx = self.key_maker.get_new()
                
            node_obj.idx = idx 

        self.nodes[node_obj.idx] = node_obj

    def remove_node(self, node_obj):
        self.nodes.pop(node_obj.idx)


    def delete_dangling_nodes(self):
        will_be_removed = []

        for key in self.nodes:
            node = self.nodes[key]
            if len(node.to_links) == 0 and len(node.from_links) == 0:
                will_be_removed.append(key)

        for key in will_be_removed:
            self.nodes.pop(key)

        return

    def draw_plot(self, axes):
        for idx, node in self.nodes.items():
            node.draw_plot(axes)
            
    def erase_plot(self):
        for idx, node in self.nodes.items():
            node.erase_plot()

    def merge_node_set(self, a_nodes):
        for node in a_nodes:
            if node in self.nodes.keys():
                to_links = a_nodes[node].get_to_links()
                for link in to_links:
                    if link.idx not in self.nodes[node].get_to_links_idx_list():
                        self.nodes[node].to_links.append(link)

                from_links = a_nodes[node].get_from_links()
                for link in from_links:
                    if link.idx not in self.nodes[node].get_from_links_idx_list():
                        self.nodes[node].from_links.append(link)

                junctions = a_nodes[node].get_junctions()
                for junction in junctions:
                    if junction.idx not in self.nodes[node].get_junctions_idx_list():
                        self.nodes[node].junctions.append(junction)
            else:
                self.nodes[node] = a_nodes[node]

        return self.nodes
