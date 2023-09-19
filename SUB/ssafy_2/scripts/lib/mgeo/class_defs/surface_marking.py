#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.base_plane import BasePlane

import numpy as np
from collections import OrderedDict

class SurfaceMarking(BasePlane):
    """
    노면표시를 나타내는 클래스. 두 가지 역할을 수행한다
    1) Mesh 생성 (예: Speedbump Mesh Guide 생성)
    2) PlannerMap에서 해당 표시를 인식 (현재 링크와 관련 있는 노면 표시를 조회 가능)
    """
    def __init__(self, points=None, idx=None):
        super(SurfaceMarking, self).__init__(points, idx)
        
        self.link_id_list = []
        self.road_id = ''

        # 참조를 저장하는 변수
        # NOTE: 참조를 위한 key가 있어도 참조할 인스턴스가 존재할 수 있다
        #       >> 이 경우 len(self.link_id_list) != len(self.link_list)이다
        self.link_list = list()

        # 기타 속성 정보
        self.type = None
        self.sub_type = None
        self.type_code_def = '' 

        """이하는 MPL에서의 draw를 위함"""
        # matplotlib에 의해 그려진 list of Line2D 객체에 대한 레퍼런스
        # plt.plot을 호출하면서 반환되는 값이며,
        # 이 레퍼런스를 통해 matplotlib에서 삭제 또는 스타일 변경 등이 가능
        self.plotted_obj = None

        # Visualization 모드
        self.reset_vis_mode_manual_appearance()


    def add_link_ref(self, link):
        if link not in self.link_list:
            self.link_list.append(link)
        
        if self not in link.surface_markings:
            link.surface_markings.append(self)


    def draw_plot(self, axes):
        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=1,
                marker='o')
            return
        
        else:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=1,
                marker='o',
                color='b')


    def erase_plot(self):
        if self.plotted_obj is not None:
            # list of matplotlib.lines.Line2D 이므로
            # iterate 하면서 remove를 호출해야 함
            for obj in self.plotted_obj:
                if obj.axes is not None:
                    obj.remove()


    def hide_plot(self):
        if self.plotted_obj is not None:
            for obj in self.plotted_obj:
                obj.set_visible(False)


    def unhide_plot(self):
        if self.plotted_obj is not None:
            for obj in self.plotted_obj:
                obj.set_visible(True)            


    def set_vis_mode_manual_appearance(self, width, color):
        self.vis_mode_line_width = width
        self.vis_mode_line_color = color


    def reset_vis_mode_manual_appearance(self):
        self.set_vis_mode_manual_appearance(None, None)      


    @staticmethod
    def to_dict(obj):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""

        dict_data = {
            'idx': obj.idx,
            'points': obj.points.tolist(),
            'link_id_list': obj.link_id_list,
            'road_id' : obj.road_id,
            'type': obj.type,
            'sub_type': obj.sub_type
        }
        return dict_data


    @staticmethod
    def from_dict(dict_data, link_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']
        points = np.array(dict_data['points'])

        # 연결된 객체 참조용 정보
        link_id_list = dict_data['link_id_list']
        road_id = dict_data['road_id']

        # 기타 속성 정보
        sm_type = dict_data['type'] # type은 지정된 함수명이므로 혼란을 피하기 위해 sign_type으로
        sm_subtype = dict_data['sub_type']

        """STEP #2 인스턴스 생성"""
        obj = SurfaceMarking(points=points, idx=idx)

        # 연결된 객체 참조용 정보
        obj.link_id_list = link_id_list
        obj.road_id = road_id

        # 기타 속성 정보
        obj.type = sm_type
        obj.sub_type = sm_subtype

        """STEP #3 인스턴스 참조 연결"""
        if link_set is not None:
            for link_id in link_id_list:
                if link_id in link_set.lines.keys():
                    link = link_set.lines[link_id]
                    obj.add_link_ref(link)

        return obj

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx}
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist()}
        prop_data['type'] = {'type' : 'string', 'value' : self.type}
        prop_data['sub_type'] = {'type' : 'string', 'value' : self.sub_type}
        prop_data['type_code_def'] = {'type' : 'string', 'value' : self.type_code_def}
        return prop_data
