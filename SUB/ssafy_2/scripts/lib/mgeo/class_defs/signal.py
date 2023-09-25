#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from class_defs.base_point import BasePoint
import numpy as np

from collections import OrderedDict

class Signal(BasePoint):
    def __init__(self, _id=None):
        super(Signal, self).__init__(_id)
   
        # 참조를 위한 key
        self.link_id_list = []
        self.road_id = ''

        # 참조를 저장하는 변수
        # NOTE: 참조를 위한 key가 있어도 참조할 인스턴스가 존재할 수 있다
        #       >> 이 경우 len(self.link_id_list) != len(self.link_list)이다
        self.link_list = list()
        
        # 기타 속성 정보
        self.type = ''
        self.sub_type = ''
        self.dynamic = None
        self.orientation = None # TODO(sglee): OpenDRIVE 일 때 MGeo 일 때 의미가 다름. OpenDRIVE 일 경우 출력하는 부분에서만 이슈이므로, MGEo에서만 의미하는 용도로 사용하도록 변경하거나 별도으 ㅣ필드 생성
        self.value = 0
        self.country = '' # 나라 이름, 필수 값은 아님
        self.z_offset = 0 # z offset from track level to bottom edge of the signal
        self.height = 0
        self.width = 0
        self.synced_signal_id = ''
        self.type_def = ''
        self.ref_crosswalk_id = '' # 연결되는 crosswalk id
        self.heading = 0


    def set_size(self):
        # 사이즈 단위 : m   
        # 
        # 코드값 1 : 주의표지
        # 설치높이(cm) : 100 ~ 210 
        if self.type == '1' :
            self.z_offset = 2.1
            self.height = 0.73
            self.width = 0.9
        # 코드값 2 : 규제표지
        # 설치높이(cm) : 100 ~ 210 cm
        elif self.type == '2' :
            self.z_offset = 2.1
            self.height = 0.6
            self.width = 0.6
        # 코드값 3 : 지시표지
        # 설치높이(cm) : 100 ~ 
        elif self.type == '3' :
            self.z_offset = 2.1
            self.height = 0.6
            self.width = 0.6
        # 코드값 4 : 보조표지
        # 설치높이(cm) : 100 ~ 
        elif self.type == '4' :
            self.z_offset = 2.1
            self.height = 0.4
            self.width = 0.4
        # 코드값 5 : 신호등
        # 측주식의 횡형, 현수식, 문형식 등은 신호등면의 하단이 차도의 노면으로부터 수직으로 450cm 이상의 높이에 위치하는 것을 원칙
        # 중앙주식, 측주식의 종형은 보도, 중앙섬 및 중앙분리대의 노면 혹은 상면에서 신호등 하단까지의 수직 높이가 250cm - 350cm 에 위치하는 것을 원칙
        elif self.type == '5' :
            # 코드값 502 : 횡형삼색등
            if self.sub_type == '502' :
                self.z_offset = 4.5
                self.height = 0.355
                self.width = 1.065
            # 코드값 505 : 횡형사색등A
            elif self.sub_type == '505' :
                self.z_offset = 4.5
                self.height = 0.355
                self.width = 1.42
            # 코드값 508 : 보행등
            elif self.sub_type == '508' :
                self.z_offset = 3.5
                self.height = 0.71
                self.width = 0.355
            # 코드값 510 : 종형삼색등
            elif self.sub_type == '510' :
                self.z_offset = 4.5
                self.height = 1.065
                self.width = 0.355
    
    def add_link_ref(self, link):
        if self.dynamic is None:
            raise BaseException('self.dynamic must be set first!')

        if link not in self.link_list:
            self.link_list.append(link)

        if self.dynamic:
            if self not in link.traffic_lights:
                link.traffic_lights.append(self)
        else:
            if self not in link.traffic_signs:
                link.traffic_signs.append(self)


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
                    fontsize=12)
            return

        """별도 style이 지정되어 있지 않을 경우, 아래의 디폴트 스타일로 그린다"""
        # 이는 list of matplotlib.lines.Line2D 인스턴스를 반환
        if self.dynamic:
            self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
                markersize=7,
                marker='o',
                color='orange')
        else:
            self.plotted_objs_point = axes.plot(self.point[0], self.point[1],
                markersize=7,
                marker='o',
                color='teal')

        # matplotlib.text.Text 인스턴스를 반환 
        self.plotted_objs_text = axes.text(self.point[0], self.point[1]+0.1,
            self.idx,
            fontsize=10)
    
    @staticmethod
    def to_dict(obj):
        """json 파일등으로 저장할 수 있는 dict 데이터로 변경한다"""

        dict_data = {
            'idx': obj.idx,
            'link_id_list': obj.link_id_list,
            'road_id' : obj.road_id,
            'type': obj.type,
            'sub_type': obj.sub_type,
            'dynamic': obj.dynamic,
            'orientation': obj.orientation,
            'point': obj.point.tolist(),
            'value' : obj.value,
            'country': obj.country,
            'z_offset': obj.z_offset,
            'height': obj.height,
            'width': obj.width,
            'type_def': obj.type_def,
            'ref_crosswalk_id':obj.ref_crosswalk_id,
            'heading': obj.heading
        }
        return dict_data

    @staticmethod
    def from_dict(dict_data, link_set=None):
        """json 파일등으로부터 읽은 dict 데이터에서 Signal 인스턴스를 생성한다"""

        """STEP #1 파일 내 정보 읽기"""
        # 필수 정보
        idx = dict_data['idx']
        point = dict_data['point']

        # 연결된 객체 참조용 정보
        link_id_list = dict_data['link_id_list']
        road_id = dict_data['road_id']
        
        # 기타 속성 정보
        sign_type = dict_data['type'] # type은 지정된 함수명이므로 혼란을 피하기 위해 sign_type으로
        sign_subtype = dict_data['sub_type']
        dynamic = dict_data['dynamic']
        orientation = dict_data['orientation']
        if 'type_def' in dict_data:
            type_def = dict_data['type_def']
        else:
            type_def = ''
        
        country = dict_data['country']
        
        if dict_data.__contains__('ref_crosswalk_id'):
            ref_crosswalk_id = dict_data['ref_crosswalk_id']
        else:
           ref_crosswalk_id = ''

        if dict_data['z_offset'] is None:
            z_offset = 0
        else:
            z_offset = dict_data['z_offset']

        if dict_data['height'] is None:
            height = 0
        else:
            height = dict_data['height']

        if dict_data['width'] is None:
            width = 0
        else:
            width = dict_data['width']

        if 'heading' in dict_data.keys():
            heading = dict_data['heading']
        else:
            heading = 0

        if 'value' in dict_data.keys():
            value = dict_data['value']
        else:
            value = 0

        """STEP #2 인스턴스 생성"""
        # 필수 정보
        obj = Signal(idx)
        obj.point = np.array(point)

        # 연결된 객체 참조용 정보
        # obj.link_id_list = link_id_list
        obj.road_id = road_id

        # 기타 속성 정보
        obj.type = sign_type
        obj.sub_type = sign_subtype
        obj.dynamic = dynamic
        obj.orientation = orientation
        obj.country = country
        obj.z_offset = z_offset
        obj.height = height
        obj.width = width
        obj.heading = heading
        obj.value = value

        obj.ref_crosswalk_id = ref_crosswalk_id
        obj.type_def = type_def

        """STEP #3 인스턴스 메소드 호출해서 설정할 값들 설정하기"""
        obj.link_id_list = []
        if link_set is not None:
            for link_id in link_id_list:
                if link_id in link_set.lines.keys():
                    link = link_set.lines[link_id]
                    obj.add_link_ref(link)
                    obj.link_id_list.append(link.idx)

        return obj
        

    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx }
        prop_data['link_id_list'] = {'type' : 'list<string>', 'value' : self.link_id_list}
        prop_data['road_id'] = {'type' : 'string', 'value' :  self.road_id}
        prop_data['type'] = {'type' : 'string', 'value' :  self.type}
        prop_data['sub_type'] = {'type' : 'string', 'value' : self.sub_type}
        prop_data['dynamic'] = {'type' : 'string', 'value' : self.dynamic}
        prop_data['orientation'] = {'type' : 'string', 'value' : self.orientation}
        prop_data['value'] = {'type' : 'int', 'value' : self.value }
        prop_data['point'] = {'type' : 'list<float>', 'value' : self.point.tolist()}
        prop_data['country'] = {'type' : 'string', 'value' : self.country}
        prop_data['z_offset'] = {'type' : 'float', 'value' : self.z_offset}
        prop_data['width'] = {'type' : 'float', 'value' : self.width}
        prop_data['height'] = {'type' : 'float', 'value' : self.height}
        prop_data['type_def'] = {'type' : 'string', 'value' : self.type_def}
        prop_data['ref_crosswalk_id'] = {'type' : 'string', 'value' : self.ref_crosswalk_id}
        prop_data['heading'] = {'type' : 'float', 'value' : self.heading}

        return prop_data
    
    def IsPedestrianSign(self):
        if self.type_def == 'mgeo' and self.type == 'pedestrian':
            return True
        elif self.type_def == 'ngii_model2' and self.type == '11':
            return True
        elif self.type_def == 'ngii_model1' and self.type == '5' and self.sub_type == '508':
            return True
        else:
            return False
                
    def remove_ref_crosswalk_id(self, id):
        if self.ref_crosswalk_id == id:
            self.ref_crosswalk_id = ''
                

    