#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.base_line import BaseLine

class Line(BaseLine):
    """FromNode, ToNode를 통해 연결하는 기능을 갖는 선"""
    def __init__(self, points=None, idx=None):
        super(Line, self).__init__(points, idx)
        self.from_node = None
        self.to_node = None

        # matplotlib에 의해 그려진 list of Line2D 객체에 대한 레퍼런스
        # plt.plot을 호출하면서 반환되는 값이며,
        # 이 레퍼런스를 통해 matplotlib에서 삭제 또는 스타일 변경 등이 가능
        self.plotted_obj = None

        # self.included_in_plane = None
        self.included_plane = list()

        # Visualization 모드
        self.set_vis_mode_all_different_color(False)
        self.reset_vis_mode_manual_appearance()

        # Curve Fitting을 지원하기 위한 기능
        self.geometry = [{'id':0, 'method':'poly3'}]


    def get_point_dict(self, point_idx):
        '''
        특정 점을 반환한다. 이 때 점은 자신이 속한 line에 대한 다양한 정보를 포함한다
        '''
        # 편의상 point_idx를 -1로 넣었으면, 이를 마지막 인덱스로 변경해준다
        if point_idx == -1:
            point_idx = self.get_last_idx()

        if point_idx < 0:
            raise BaseException('[ERROR] Line.get_point_dict: input argument point_idx must be >= 0. (-1 is exceptionally ok).')

        if point_idx == 0:
            type_str = 'start'
        elif point_idx == self.get_last_idx():
            type_str = 'end'
        else:
            type_str = 'mid'
        
        dict_obj = dict({
            'idx_line': self.idx,
            'idx_point': point_idx,
            'type': type_str,
            'coord': self.points[point_idx],
            'line_ref': self})
        return dict_obj 
    

    def get_last_idx(self):
        return self.points.shape[0] - 1
    

    def set_from_node(self, _from_node):
        # 기존의 from_node의 참조를 업데이트 (현재 링크 제거)
        if self.from_node is not None:
            self.from_node.remove_to_links(self)

        # 새로운 from_node로 참조를 업데이트
        self.from_node = _from_node

        # 새로운 from_node의 참조를 업데이트 (현재 링크 추가)
        # add_to_links
        if _from_node is not None and self not in _from_node.to_links:
            _from_node.to_links.append(self)


    def set_to_node(self, _to_node):
        # 기존의 to_node 참조를 업데이트 (현재 링크 제거)
        if self.to_node is not None:
            self.to_node.remove_from_links(self)

        # 새로운 to_node로 참조를 업데이트
        self.to_node = _to_node

        # 새로운 to_node의 참조를 업데이트 (현재 링크 추가)
        # add_from_links
        if _to_node is not None and self not in _to_node.from_links:
            _to_node.from_links.append(self)


    def remove_from_node(self):
        self.from_node.remove_to_links(self)
        self.from_node = None
    
    
    def remove_to_node(self):
        self.to_node.remove_from_links(self)
        self.to_node = None
        

    def get_from_node(self):
        return self.from_node 


    def get_to_node(self):
        return self.to_node


    def get_from_links(self):
        return self.from_node.get_from_links()
    

    def get_to_links(self):
        return self.to_node.get_to_links()


    def get_from_node_sharing_links(self):
        '''
        특정 노드에서 같이 출발하는 링크가 있을 수 있다.
        나를 제외한 해당 링크를 반환한다.
        '''
        links = self.from_node.get_to_links()
        ret = list()
        for each_link in links:
            if each_link is not self: # [중요] value가 같은게 아니라, instance가 동일여부인지를 체크해야 함 그래서 is not을 사용
                ret.append(each_link)
        return ret


    def get_to_node_sharing_links(self):
        '''
        특정 노드로 같이 들어가는 링크가 있을 수 있다.
        나를 제외한 해당 링크를 반환한다.
        '''
        links = self.to_node.get_from_links()
        ret = list()
        for each_link in links:
            if each_link is not self: # [중요] value가 같은게 아니라, instance가 동일여부인지를 체크해야 함 그래서 is not을 사용
                ret.append(each_link)
        return ret


    def is_source(self):
        return len(self.get_from_links()) == 0


    def is_sink(self):
        return len(self.get_to_links()) == 0


    def get_included_planes(self):
        return self.included_plane


    def add_included_plane(self, plane):
        self.included_plane.append(plane)


    def remove_included_plane(self, plane_to_remove):
        self.included_plane.remove(plane_to_remove)


    def add_geometry(self, point_id, method):
        if point_id == len(self.points) - 1:
            raise BaseException('adding geometry point in the last point is not supported.')

        # 만약 현재 point_id 가 이미 있으면 현재 입력된 method로 변경하고 리턴한다
        for geo_point in self.geometry:
            if geo_point['id'] == point_id:
                geo_point['method'] = method
                return

        # 이 경우는 현재 전달된 point_id가 self.geometry 내부에 없는 경우가 된다.
        # 이 point_id를 추가해주고, sort 해주면 된다.
        self.geometry.append({'id':point_id, 'method':method})

        # 추가할때마다 'id'를 기준으로 ascending-order로 sort해준다
        self.geometry = sorted(self.geometry, key=lambda element: element['id'])



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
        

        if self.get_vis_mode_all_different_color():
            # 모두 다르게 그리라고하면, 색을 명시하지 않으면 된다
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=1,
                marker='o')
        
        else:
            # 이 경우에는 선의 종류에 따라 정해진 색과 모양으로 그린다
            
            if not self.included_plane:
                # 이는 list of matplotlib.lines.Line2D 인스턴스를 반환
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    markersize=1,
                    marker='o',
                    color='k')
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


    def set_vis_mode_all_different_color(self, on_off):
        self.vis_mode_all_different_color = on_off


    def get_vis_mode_all_different_color(self):
        return self.vis_mode_all_different_color


    def set_vis_mode_manual_appearance(self, width, color):
        self.vis_mode_line_width = width
        self.vis_mode_line_color = color


    def reset_vis_mode_manual_appearance(self):
        self.set_vis_mode_manual_appearance(None, None)

                
                
