#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))


class Plane(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, idx):
        self.nodes = list()
        self.line_connection = list()
        self.internal_nodes = list()
        self.idx = idx

        # bbox
        self.bbox_x = None
        self.bbox_y = None
        self.bbox_z = None
    
    def init_from_node_idx_list(self, node_set, node_idx_list):
        for node_idx in node_idx_list:
            node = node_set.nodes[node_idx]
            self.append_node(node)

    def append_node(self, node):
        # 이미 저장한 노드에 들어있다면 에러를 발생,
        # 단, 처음 노드는 제외.
        # 마지막 노드까지 연결하여 plane을 정의
        if node in self.nodes[1:]:
            raise BaseException('[ERROR] Choose a new point to add to plane creation')

        # 처음 노드이면 그냥 append
        if len(self.nodes) == 0:
            self.nodes.append(node)
            return 
        
        # 이전 노드가 있으면, 이전 노드에서 현재 노드로의 링크와 방향을 찾는다
        start_node = self.nodes[-1]
        end_node = node

        # 연결되는 방향을 확인한다
        if end_node in start_node.get_to_nodes():
            # end_node가 start_node의 to_node로 있으면
            # 해당 방향으로 원래의 링크 방향이다.
            link_reverse = False
        elif end_node in start_node.get_from_nodes():
            # end_node가 start_node의 from_node로 있으면
            # 해당 방향과 반대의 링크이다.
            link_reverse = True
        else:
            raise BaseException('[ERROR] There is no direct link from node {} to node {}'.format(start_node.idx, end_node.idx))
        
        # 해당 링크를 찾는다
        if link_reverse == False: # line moves away from source
            for line in start_node.to_links:
                start_node_dest = line.get_to_node()
                if start_node_dest.idx == end_node.idx:
                    link_line = line
                    #plane_creation_vertices += line.points.tolist()

        if link_reverse == True: # line moves towards source
            for line in start_node.from_links:
                start_node_dest = line.get_from_node()
                if start_node_dest.idx == end_node.idx:
                    link_line = line
                    #plane_creation_vertices += line.points.tolist()

        self.nodes.append(node)
        self.line_connection.append({'line': link_line,
            'line_idx': link_line.idx,
            'reverse': link_reverse})
        
    
    def to_string(self):
        ret_str = '----- Plane id={:<5} -----\n'.format(self.idx)
        for i in range(len(self.line_connection)):
            start_node = self.nodes[i]
            end_node = self.nodes[i+1]
            link_line = self.line_connection[i]['line']
            link_reverse = self.line_connection[i]['reverse']
            ret_str += '  node={:<5} -> line={:<5} -> node={:<5}, reverse={}\n'.format(
                start_node.idx, link_line.idx, end_node.idx, link_reverse)
        ret_str += '-------------------------'
        return ret_str

        # idx_str = ''
        # for i in range(len(self.nodes)):
        #     if i == len(self.nodes) - 1:
        #         # 마지막일 때는 , 붙일 필요 없으므로 
        #         idx_str += '{}'.format(self.nodes[i].idx)
        #     else:
        #         idx_str += '{}, '.format(self.nodes[i].idx)

        # return 'Plane id={}, nodes=[{}]'.format(self.idx, idx_str)

    def is_closed(self):
        # plane은 최소 3개의 점으로 구성되어야 하므로, 
        # node는 최소 4개, 시작과 끝이 같아야 한다
        if len(self.nodes) < 4:
            return False
        
        if self.nodes[0] == self.nodes[-1]:
            return True
        
        return False

    def get_plane_nodes(self):
        return self.nodes

    def reset_plane(self):
        self.nodes.clear()
        self.line_connection.clear()

    def get_node_idx_list(self):
        node_idx_list = []
        for node in self.nodes:
            node_idx_list.append(node.idx)
        return node_idx_list

    def append_internals(self, point):
        self.internal_nodes.append(point)

    def determine_bbox(self):
        if self.nodes == []:
            return

        x_list = []
        y_list = []
        z_list = []
        for node in self.nodes:
            x_list.append(node.point[0])
            y_list.append(node.point[1])
            z_list.append(node.point[2])

        self.bbox_x = [min(x_list), max(x_list)]
        self.bbox_y = [min(y_list), max(y_list)]
        self.bbox_z = [min(z_list), max(z_list)]
        