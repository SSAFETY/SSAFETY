#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))


from class_defs.base_point import BasePoint

from collections import OrderedDict

class Node(BasePoint):
    """두 선을 연결하는 기능을 갖는 점"""
    def __init__(self, _id=None):
        super(Node, self).__init__(_id)

        self.to_links = list()
        self.from_links = list()
        self.junctions = list()

        self.node_type = None
        self.included_in_plane = None
        self.on_stop_line = False
        

    def set_node_type(self, node_type):
        self.node_type = node_type

    
    def add_to_links(self, link):
        if link.from_node is not None:
            raise BaseException('link (id = {}) already has from_node. Remove it first.'.format(link.idx))
        
        link.from_node = self

        if link not in self.to_links:
            self.to_links.append(link)
        

    def add_from_links(self, link):
        if link.to_node is not None:
            raise BaseException('link (id = {}) already has to_node. Remove it first.'.format(link.idx))
        
        link.to_node = self

        if link not in self.from_links:
            self.from_links.append(link)
                

    def add_junction(self, junction):
        if junction not in self.junctions:
            self.junctions.append(junction)

        if self not in junction.get_jc_nodes():
            junction.jc_nodes.append(self)


    # def delete_self(self):
    #     # 아직 링크에 대해서는 지원하지 않는다. 해당 링크가 노드 없는 링크가 되어버리는 오류가 발생하므로

    #     for jc in self.junctions:
    #         # 자기 자신이 포함된 junction에서 자신을 제외시킨다
    #         if self in jc.jc_nodes:
    #             jc.jc_node.remove(self)


    def remove_to_links(self, line_to_delete):
        if line_to_delete in self.to_links:
            self.to_links.remove(line_to_delete)
        line_to_delete.from_node = None


    def remove_from_links(self, line_to_delete):
        if line_to_delete in self.from_links:
            self.from_links.remove(line_to_delete)
        line_to_delete.to_node = None


    def remove_junctions(self, junction):
        if junction in self.junctions:
            self.junctions.remove(junction)

        if self in junction.get_jc_nodes():
            junction.jc_nodes.remove(self)


    def get_to_links(self):
        return self.to_links
    

    def get_from_links(self):
        return self.from_links


    def get_to_links_idx_list(self):
        idx_list = []
        for link in self.get_to_links():
            idx_list.append(link.idx)
        return idx_list


    def get_from_links_idx_list(self):
        idx_list = []
        for link in self.get_from_links():
            idx_list.append(link.idx)
        return idx_list


    def get_junctions_idx_list(self):
        
        if self.junctions is None:
            node_junc_id = []
        elif isinstance(self.junctions, list):
            id_list = []
            for junc in self.junctions:
                id_list.append(junc.idx)
            node_junc_id = id_list
        else:
            raise BaseException('Unexpected node.junctions (expected: list, actual type: {})'.format(type(self.junctions)))
        
        return node_junc_id


    def get_from_nodes(self):
        from_nodes = []
        for link in self.get_from_links():
            # NOTE: 아래 예시와 같이 Node 01 과 Node 10을 이어주는 링크가 여러개 존재할 수 있다
            # 따라서, from_nodes 내부에 이미 Node 01이 들어있는지 체크하지 않으면, Node 01이 2번 들어가게된다.
            #         ↗ Link 1 ↘ 
            # Node 01 -> Link 2 -> Node 10
            # Node 02 -> Link 3 ↗ 
            if link.from_node not in from_nodes:
                from_nodes.append(link.from_node)
        return from_nodes


    def get_to_nodes(self):
        to_nodes = []
        for link in self.get_to_links():
            # NOTE: 아래 예시와 같이 Node 01 과 Node 10을 이어주는 링크가 여러개 존재할 수 있다
            # 따라서, to_nodes 내부에 이미 Node 10이 들어있는지 체크하지 않으면, Node 10이 2번 들어가게된다.
            #         ↗ Link 1 ↘ 
            # Node 01 -> Link 2 -> Node 10
            #         ↘ Link 3 -> Node 11
            if link.to_node not in to_nodes:
                to_nodes.append(link.to_node)
        return to_nodes


    def find_shortest_link_leading_to_node(self, to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        # NOTE: 
        to_links = []
        for link in self.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        if len(to_links) == 0:
            raise BaseException('[ERROR] Error @ Dijkstra.find_shortest_path : Internal data error. There is no link from node (id={}) to node (id={})'.format(self.idx, to_node.idx))

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost


    def get_junctions(self):
        return self.junctions


    def is_dangling_node(self):
        """어떠한 링크에도 연결되지 않은 노드인지 검색한다"""
        if len(self.to_links) != 0:
            return False
        if len(self.from_links) != 0:
            return False
        return True


    def is_end_node(self):
        if (len(self.to_links) == 0
            or len(self.from_links) == 0):
            return True
        return False


    def is_on_stop_line(self):
        # on_stop_line 정보가 없는 경우 (국토부 데이터 등..)
        if self.on_stop_line is None :
            return False
        # on_stop_line 정보가 있는 경우
        else :
            return self.on_stop_line

    def draw_plot(self, axes):
        """MPLCanvas 사용시, 본 클래스의 인스턴스를 plot하기 위한 함수"""

        """별도 style이 지정되어 있을 경우, 지정된 스타일로 그린다"""
        if self.vis_mode_size is not None and \
            self.vis_mode_color is not None:
            self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
                markersize=self.vis_mode_size,
                marker='D',
                color=self.vis_mode_color)

            # matplotlib.text.Text 인스턴스를 반환
            if not self.vis_mode_no_text:
                self.plotted_objs_text = axes.text(self.point[0], self.point[1]+0.1,
                    self.idx,
                    fontsize=10)
            return

        """별도 style이 지정되어 있지 않을 경우, 아래의 디폴트 스타일로 그린다"""
        # 이는 list of matplotlib.lines.Line2D 인스턴스를 반환
        if not self.included_in_plane:
            # 기본 Plot 스타일
            self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
                markersize=7,
                marker='D',
                color='g')
        else:
            self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
                markersize=7,
                marker='D',
                color='r')

        # matplotlib.text.Text 인스턴스를 반환
        if not self.vis_mode_no_text: 
            self.plotted_objs_text = axes.text(self.point[0], self.point[1]+0.1,
                self.idx,
                fontsize=10)


    def to_dict(self):
        # 교차로 정보 없으면 Emtpy List로 
        if self.junctions is None:
            node_junc_id = []
        elif isinstance(self.junctions, list):
            id_list = []
            for junc in self.junctions:
                id_list.append(junc.idx)
            node_junc_id = id_list
        else:
            raise BaseException('Unexpected node.junctions (expected: list, actual type: {})'.format(type(self.junctions)))

        dict_data = {
            'idx': self.idx,
            'node_type': self.node_type,
            'junction': node_junc_id,
            'point': self.point.tolist(),
            'on_stop_line': self.on_stop_line
        }
        return dict_data

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx}
        prop_data['point'] = {'type' : 'list<float>', 'value' : self.point.tolist()}
        prop_data['to_links'] = {'type' : 'list<string>', 'value' : self.get_to_links_idx_list()}
        prop_data['from_links'] = {'type' : 'list<string>', 'value' : self.get_from_links_idx_list()}
        prop_data['on_stop_line'] = {'type' : 'boolean', 'value' : self.on_stop_line}
        prop_data['junctions'] = {'type' : 'list<string>', 'value' : self.get_junctions_idx_list()}

        return prop_data