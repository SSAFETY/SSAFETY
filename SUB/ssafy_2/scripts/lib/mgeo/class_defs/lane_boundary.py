#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

import numpy as np

from .line import Line

from collections import OrderedDict


class LaneBoundary(Line):
    """도로의 차선을 표현하는 선, Mesh 생성을 위해 사용된다."""
    def __init__(self, points=None, idx=None):
        super(LaneBoundary, self).__init__(points, idx)

        # Visualization 모드
        self.set_vis_mode_all_different_color(True)

        self.lane_type_def = ''
        # lane_type, color, shape -> list로 변경
        self.lane_type = []
        self.lane_sub_type = 0

        self.lane_color = []
        self.lane_shape = []

        # dash solid_line_interval
        self.lane_width = 0.15 # U턴구역선이면 0.35로 변경할 것
        self.dash_interval_L1 = 30 # 도색 길이
        self.dash_interval_L2 = 50 # 빈 길이
        self.double_line_interval = 0.10 # 겹선일 때 두 선 사이의 거리, 규성상 0.10~0.15, 차선 종류와 관계없이 일정.

        # mesh를 생성하기 위한 vertices
        self.mesh_gen_vertices = [] 
        self.mesh_gen_vertex_subsets_for_each_face = [] # face를 구성할 vertices의 index를 기록한다
        self.mesh_gen_vertex_uv_coords = [] # uv 좌표
        
        # 
        self.vis_mode_marker_size = 0
        self.vis_mode_marker_style = ""

        # passRestr -> pass_restr으로 변경
        self.pass_restr = ''

        # 차선 shape/color 범위
        self.lane_type_offset = []


    def get_lane_num(self):
        return len(self.lane_shape)


    def set_lane_type_list(self, start):
        self.lane_type_offset.append(start)


    def is_every_attribute_equal(self, another):
        """attribute가 같은지 확인한다"""
        if self.lane_type_def != another.lane_type_def:
            return False
        
        if self.lane_type != another.lane_type:
            return False

        if self.lane_sub_type != another.lane_sub_type:
            return False

        if self.lane_color != another.lane_color:
            return False

        if self.lane_shape != another.lane_shape:
            return False

        if self.lane_width != another.lane_width:
            return False

        if self.dash_interval_L1 != another.dash_interval_L1:
            return False

        if self.dash_interval_L2 != another.dash_interval_L2:
            return False

        if self.double_line_interval != another.double_line_interval:
            return False
        
        return True

  
    def get_attribute_from(self, src):
        LaneBoundary.copy_attribute(src, self)


    def to_dict(self):
        dict_data = {
            'idx': self.idx,
            'from_node_idx': self.from_node.idx if self.from_node else None,
            'to_node_idx': self.to_node.idx if self.to_node else None,
            'points': self.points.tolist(),
            'lane_type_def': self.lane_type_def,
            'lane_type': self.lane_type,
            'lane_sub_type': self.lane_sub_type,
            'lane_color': self.lane_color,
            'lane_shape': self.lane_shape,
            'lane_width': self.lane_width,
            'dash_interval_L1': self.dash_interval_L1,
            'dash_interval_L2': self.dash_interval_L2,
            'double_line_interval': self.double_line_interval,
            'geometry': self.geometry,
            'pass_restr' : self.pass_restr,
            'lane_type_offset': self.lane_type_offset
        }
        return dict_data
            
    def rorate_around_z_axis(self, angle, point):
        rotation = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle),  np.cos(angle), 0.0],
            [0.0, 0.0, 1.0]])

        transform_pt = rotation.dot(point)
        return transform_pt

    def rotate_around_vector_axis(self, angle, axis, point) :
        mat = self.rotation_matrix(axis, angle)

        transform_pt = mat.dot(point)
        return transform_pt

    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        
        axis = np.asarray(axis)
        axis = axis / np.math.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    
    def draw_plot(self, axes):
        # if self.get_lane_num() != 2:
        #     return
        # if self.lane_type != 502:
        #     return
            
        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=self.vis_mode_marker_size,
                marker=self.vis_mode_marker_style)
            return
        

        if self.get_vis_mode_all_different_color():
            # 모두 다르게 그리라고하면, 색을 명시하지 않으면 된다
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=self.vis_mode_marker_size,
                marker=self.vis_mode_marker_style)
        
        else:
            # 이 경우에는 선의 종류에 따라 정해진 색과 모양으로 그린다
            
            if not self.included_plane:
                # 이는 list of matplotlib.lines.Line2D 인스턴스를 반환
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    markersize=self.vis_mode_marker_size,
                    marker=self.vis_mode_marker_style,
                    color='k')
            else:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    markersize=self.vis_mode_marker_size,
                    marker=self.vis_mode_marker_style,
                    color='b')


    @staticmethod
    def from_dict(dict_data, node_set=None):
        idx = dict_data['idx']
        from_node_idx = dict_data['from_node_idx']
        to_node_idx = dict_data['to_node_idx']
        points = np.array(dict_data['points'])
        start_node = None
        end_node = None
        
        """이제 node와 연결해준다"""
        if node_set is not None:
            if from_node_idx in node_set.nodes :
                start_node = node_set.nodes[from_node_idx]
            if to_node_idx in node_set.nodes :
                end_node = node_set.nodes[to_node_idx]

        lane_boundary = LaneBoundary(points=points, idx=idx)
        if start_node != None :
            lane_boundary.set_from_node(start_node)
        if end_node != None :
            lane_boundary.set_to_node(end_node)

        if 'lane_type_def' in dict_data:
            lane_boundary.lane_type_def = dict_data['lane_type_def']
        elif 'lane_code_def' in dict_data:
            lane_boundary.lane_type_def = dict_data['lane_code_def']
        
        # lane_type, lane_color, lane_shape -> list로 변경
        lane_type = None
        lane_type_list = []
        if 'lane_type' in dict_data:
            lane_type = dict_data['lane_type']
        elif 'lane_code' in dict_data:
            lane_type = dict_data['lane_code']
        if 'lane_type_list' in dict_data:
            lane_type_list = dict_data['lane_type_list']
        if len(lane_type_list) == 0 and lane_type is not None:
            if type(lane_type) == int:
                lane_type_list.append(lane_type)
            else:
                lane_type_list = lane_type
        lane_boundary.lane_type = lane_type_list
            
        if 'lane_sub_type' in dict_data:
            lane_boundary.lane_sub_type = dict_data['lane_sub_type']

        # ex) 'white' -> ['white']
        lane_color = dict_data['lane_color']
        lane_color_list = []
        if 'lane_color_list' in dict_data:
            lane_color_list = dict_data['lane_color_list']
        if len(lane_color_list) == 0 and lane_color is not None:
            if type(lane_color) == str:
                lane_color_list.append(lane_color)
            else:
                lane_color_list = lane_color
        lane_boundary.lane_color = lane_color_list

        lane_boundary.lane_width = dict_data['lane_width']
        lane_boundary.dash_interval_L1 = dict_data['dash_interval_L1']
        lane_boundary.dash_interval_L2 = dict_data['dash_interval_L2']
        lane_boundary.double_line_interval = dict_data['double_line_interval']
        if 'geometry' in dict_data:
            lane_boundary.geometry = dict_data['geometry']
        if 'pass_restr' in dict_data:
            lane_boundary.pass_restr = dict_data['pass_restr']
        elif 'passRestr' in dict_data:
            lane_boundary.pass_restr = dict_data['passRestr']

        lane_type_offset = []
        if 'lane_type_offset' in dict_data:
            lane_type_offset = dict_data['lane_type_offset']
        elif 'lane_type_start' in dict_data:
            lane_type_offset = dict_data['lane_type_start']
        if len(lane_type_offset) == 0:
            lane_type_offset.append(0)
        lane_boundary.lane_type_offset = lane_type_offset

        # lane_type_offset 갯수
        lane_type_count = len(lane_boundary.lane_type_offset)
        lane_shape = dict_data['lane_shape']
        shape_list = []
        if len(lane_shape) == lane_type_count:
            shape_list = lane_shape

        if 'lane_shape_list' in dict_data:
            lane_shape_list = dict_data['lane_shape_list']
            if len(lane_shape_list) == lane_type_count:
                shape_list = lane_shape_list
        if len(shape_list) == 0 and lane_shape is not None:
            lane_shape_str = ''
            for i in range(len(lane_shape)):
                if i == 0:
                    lane_shape_str += '{}'.format(lane_shape[i])
                else:
                    lane_shape_str += ' {}'.format(lane_shape[i])
            shape_list.append(lane_shape_str)

        lane_boundary.lane_shape = shape_list

        return lane_boundary

    
    @staticmethod
    def copy_attribute(src, dst):
        dst.lane_type_def = src.lane_type_def
        dst.lane_sub_type = src.lane_sub_type

        # dst.lane_type = src.lane_type
        # dst.lane_color = src.lane_color
        # dst.lane_shape = src.lane_shape

        # dash solid_line_interval
        dst.lane_width = src.lane_width
        dst.dash_interval_L1 = src.dash_interval_L1
        dst.dash_interval_L2 = src.dash_interval_L2
        dst.double_line_interval = src.double_line_interval

        dst.pass_restr = src.pass_restr


    def item_prop(self):
        item = self.to_dict()
        prop_data = OrderedDict()

        prop_data['idx'] = {'type' : 'string', 'value' : item['idx']}
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : item['points']}
        prop_data['from_node_idx'] = {'type' : 'string', 'value' : item['from_node_idx']}
        prop_data['to_node_idx'] = {'type' : 'string', 'value' : item['to_node_idx']}
        prop_data['lane_type'] = {'type' : 'list<int>', 'value' : item['lane_type']}
        prop_data['lane_sub_type'] = {'type' : 'int', 'value' : item['lane_sub_type']}
        prop_data['lane_type_def'] = {'type' : 'string', 'value' : item['lane_type_def']}
        prop_data['lane_color'] = {'type' : 'list<string>', 'value' : item['lane_color']}
        prop_data['lane_shape'] = {'type' : 'list<string>', 'value' : item['lane_shape']}
        prop_data['lane_width'] = {'type' : 'float', 'value' : item['lane_width']}
        prop_data['dash_interval_L1'] = {'type' : 'float', 'value' : item['dash_interval_L1']}
        prop_data['dash_interval_L2'] = {'type' : 'float', 'value' : item['dash_interval_L2']}
        prop_data['double_line_interval'] = {'type' : 'float', 'value' : item['double_line_interval']}
        prop_data['geometry'] = {'type' : 'list<dict>', 'value' : item['geometry']}
        prop_data['pass_restr'] = {'type' : 'string', 'value' : item['pass_restr']}
        prop_data['lane_type_offset'] = {'type' : 'list<float>', 'value' : item['lane_type_offset']}

        return prop_data

    def get_last_idx(self):
        return self.points.shape[0] - 1
