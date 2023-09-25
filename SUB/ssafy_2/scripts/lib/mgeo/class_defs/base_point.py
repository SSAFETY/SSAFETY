#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

class BasePoint(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self, _id=None):
        self.idx = _id # index can be initialized lazily, by a NodeSet object
        self.point = None


        """MPL Draw 관련 변수"""

        # matplotlib에 의해 그려진 list of Line2D객체에 대한 레퍼런스
        # plt.plot을 호출하면서 반환되는 값이며,
        # 이 레퍼런스를 통해 matplotlib에서 삭제 또는 스타일 변경 등이 가능
        self.plotted_objs_point = None 
        self.plotted_objs_text = None 

        # Visualization 모드
        self.reset_vis_mode_manual_appearance()


    def is_out_of_xy_range(self, xlim, ylim):
        """NOTE: XY 축에 대해서만 확인한다"""
        x_pos = self.point[0]
        y_pos = self.point[1]

        # x축에 대해
        if x_pos < xlim[0] or xlim[1] < x_pos:
            return True

        # y축에 대해
        if y_pos < ylim[0] or y_pos > ylim[1]:
            return True
        
        return False


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
        self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
            markersize=7,
            marker='D',
            color='g')

        # matplotlib.text.Text 인스턴스를 반환 
        self.plotted_objs_text = axes.text(self.point[0], self.point[1]+0.1,
            self.idx,
            fontsize=10)


    def erase_plot(self):
        if self.plotted_objs_point is not None:
            # list of matplotlib.lines.Line2D 이므로
            # iterate 하면서 remove를 호출해야 함
            for obj in self.plotted_objs_point:
                if obj.axes is not None:
                    obj.remove()

        self._erase_text()


    def _erase_text(self):
        if self.plotted_objs_text is not None:
            if self.plotted_objs_text.axes is not None:
                self.plotted_objs_text.remove()


    def hide_text(self):
        if self.plotted_objs_text is not None:
            self.plotted_objs_text.set_visible(False)


    def unhide_text(self):
        if self.plotted_objs_text is not None:
            self.plotted_objs_text.set_visible(True)


    def hide_plot(self):
        if self.plotted_objs_point is not None:
            for obj in self.plotted_objs_point:
                obj.set_visible(False)

        self.hide_text()


    def unhide_plot(self):
        if self.plotted_objs_point is not None:
            for obj in self.plotted_objs_point:
                obj.set_visible(True)

        self.unhide_text()


    def set_vis_mode_manual_appearance(self, size, color, no_text=False):
        self.vis_mode_size = size
        self.vis_mode_color = color
        self.vis_mode_no_text = no_text
    

    def reset_vis_mode_manual_appearance(self):
        self.set_vis_mode_manual_appearance(None, None, True)