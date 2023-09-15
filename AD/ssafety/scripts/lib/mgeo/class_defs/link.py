#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))


from class_defs.line import Line
import numpy as np 

from collections import OrderedDict

# Link Class
class Link(Line):
    '''
    내부의 points 필드를 처음부터 초기화하지 않고 나중에 만들 수 있는 클래스이다.
    
    lazy_point_init 필드가 True이면, point 변수 초기화를 나중에 할 수 있다.
    이는 차선 변경이 가능함을 표현하기 위한 클래스로, 아래 예시를 통해 정의를 이해할 수 있다.

    아래와 같이 편도 2차선인 도로를 가정하면 도로의 양끝에는 노드가 2개씩 있어,
    총 4개의 노드가 정의된다.
    
    예제)
    ======> 실제 도로 방향 =====>
    Node1A                Node2A
    Node1B                Node2B

    이 때 어느 쪽으로든 차선이 변경 가능하다고 하면, 총 4종류의 링크가 생성 가능한데,
    
    Node1A -> Node2A
    Node1B -> Node2B
    위 2가지 링크는 차선 변경을 하지 않는 링크로, 
      실제 차가 따라가야할 경로가 fix되어 있는 셈이다.
      이 경우 lazy_point_init = False로 정의하고, points 필드에 경로점이 정의되어 있다.
    
    Node1A -> Node2B
    Node1B -> Node2A
    위 2가지 링크는 차선 변경을 하는 링크로,
      실제 차가 따라가야할 경로는 고정되어 있지 않다 (차선 변경을 어느 시점에든 할 수 있으므로)
      이 경우 lazy_point_init = True로 정의하고, points 필드는 연결해야하는 양 끝점만 가지고 있다.

    '''
    def __init__(self, points=None, idx=None, lazy_point_init=False, link_type=None, road_type=None):        
        self.lazy_point_init = lazy_point_init
        super(Link, self).__init__(points, idx)

        # 차선 변경이 아닐 경우 이 값이 유효. 차선 변경 링크를 생성하기 위한 값들
        self.lane_ch_link_left = None # 좌측 차선 진입으로 들어갈 수 있는 링크
        self.lane_ch_link_right = None # 우측 차선 진입으로 들어갈 수 있는 링크

        # 같은 도로 소속인 차선의 관계성 확보용 변수
        self.lane_group = None

        # 차선 변경일 경우
        self.lane_change_pair_list = list()

        # 최대 속도 및 최저 속도
        self.max_speed = 0
        self.min_speed = 0
        self.speed_unit = ''
        # speed_start, speed_end -> speed_offset으로 변경
        self.speed_offset = []
        
        # TomTom 데이터 속도(list) 추가
        self.recommended_speed = 0
        self.speed_list = None

        self.link_type = link_type
        self.link_type_def = ''
        self.road_type = road_type

        # 해당 링크에 연결된 object들
        self.traffic_signs = list()
        self.traffic_lights = list()
        self.surface_markings = list()

        # 42dot 데이터
        self.road_id = ''
        self.ego_lane = None
        self.lane_change_dir = None
        self.hov = None

        # stryx 데이터
        self.related_signal = None        
        self.its_link_id = None
        self.can_move_left_lane = False
        self.can_move_right_lane = False
        # self.road_type = None

        # tomtom 데이터 - 21.04.03
        # 국토부 데이터 구조 때문에 link가 여러 개의 lane marking을 참조하도록 수정 - 21.06.08
        self.lane_mark_left = []
        self.lane_mark_right = []
     
        # OpenDRIVE 생성 관련 >> width 강제 설정
        fw, ws, fe, we = self.get_default_width_related_values()
        self.force_width_start = fw
        self.width_start = ws
        self.force_width_end = fe
        self.width_end = we

        # OpenDRIVE 생성 관련 >> sidewalk 설정
        self.enable_side_border = False
        # self.width_start = -1 # -1 이면 b.c.에 따라 자동 설정된다
        # self.width_end = -1 # -1 이면 b.c.에 따라 자동 설정된다

        # [210512] tomtom geoJSON 데이터 추가 속성 추출
        self.opp_traffic = False # oppTraffic -> opp_traffic 으로 변경
        self.is_entrance = False
        self.is_exit = False

        # OpenDRIVE 생성 관련 >> ref line 찾을 때 사용
        self.odr_lane = None
        self.max_succeeding_link_solution_calculated = False
        self.max_succeeding_link_solution = (1, [self])


    def set_points(self, points):
        super(Link, self).set_points(points)
        # NOTE: cost 계산 시 고려되어야 하는 부분이 너무 많아서, 이를 set_points에 묶어둘 수 없다.
        # self.calculate_cost()
    
    def is_it_for_lane_change(self):
        return self.lazy_point_init

    def get_traffic_signs(self):
        return self.traffic_signs

    def get_traffic_lights(self):
        return self.traffic_lights

    def get_surface_markings(self):
        return self.surface_markings
    

    ''' 차선 변경으로 진입 가능한 링크 설정 ''' 
    def set_left_lane_change_dst_link(self, link):
        if type(link).__name__ != 'Link':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(link)))    
        self.lane_ch_link_left = link

    def set_right_lane_change_dst_link(self, link):
        if type(link).__name__ != 'Link':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(link)))        
        self.lane_ch_link_right = link

    def get_left_lane_change_dst_link(self):
        if self.is_it_for_lane_change():
            raise BaseException('[ERROR] lane_change_dst_link is only defined when self.is_it_for_lane_change() == False')
        return self.lane_ch_link_left

    def get_right_lane_change_dst_link(self):
        if self.is_it_for_lane_change():
            raise BaseException('[ERROR] lane_change_dst_link is only defined when self.is_it_for_lane_change() == False')
        return self.lane_ch_link_right
    

    ''' 차선 변경 관련'''
    def get_lane_change_pair_list(self):
        return self.lane_change_pair_list
    
    def set_lane_change_pair_list(self, info):
        self.lane_change_pair_list = info

    def get_number_of_lane_change(self):
        if not self.is_it_for_lane_change():
            return 0
        else:
            return len(self.lane_change_pair_list)

    def get_all_left_links(self, check_road=True):
        """좌측 차선 변경으로 진입할 수 있는 모든 링크 리스트를 반환한다.
        check_road는 True이면, 현재 링크와 road가 같은 lane_ch_link_left 중에서 찾는다. (즉 road가 다른 link가 나타나면 중단)
        """
        ret_list = list()

        current_link = self
        left_link = current_link.lane_ch_link_left
        while left_link is not None:
            # link 오류로 인해, ret_list에서 다시 left_link가 검출되었다면 오류이다.
            if left_link in ret_list:
                raise BaseException('link: {} has a logical error. get_all_left_lanes detected an infinite-loop.'.format(current_link.idx))
            
            # road_id를 체크하는 경우라면,
            if check_road:
                # road_id가 다른 link가 발견되면 종료한다
                if left_link.road_id != current_link.road_id:
                    break

            ret_list.append(left_link)
            
            # 현재 링크를 다시 left_link로 업데이트하고, left_link 또한 업데이트
            current_link = left_link
            left_link = current_link.lane_ch_link_left

        return ret_list

    def get_all_right_links(self, check_road=True):
        """우측 차선 변경으로 진입할 수 있는 모든 링크 리스트를 반환한다.
        check_road는 True이면, 현재 링크와 road가 같은 lane_ch_link_right 중에서 찾는다. (즉 road가 다른 link가 나타나면 중단)
        """
        ret_list = list()

        current_link = self
        right_link = current_link.lane_ch_link_right
        while right_link is not None:
            # link 오류로 인해, ret_list에서 다시 left_link가 검출되었다면 오류이다.
            if right_link in ret_list:
                raise BaseException('link: {} has a logical error. get_all_right_links detected an infinite-loop.'.format(current_link.idx))
            
            # road_id를 체크하는 경우라면,
            if check_road:
                # road_id가 다른 link가 발견되면 종료한다
                if right_link.road_id != current_link.road_id:
                    break

            ret_list.append(right_link)
            
            # 현재 링크를 다시 right_link로 업데이트하고, right_link 또한 업데이트
            current_link = right_link
            right_link = current_link.lane_ch_link_right

        return ret_list

    def is_in_the_left_or_right_side(self, another_link):
        """현재 링크가 another_link의 왼쪽 또는 오른쪽에 있는지 찾아준다. 왼쪽/오른쪽 어디에도 없으면 False, ''가 반환된다"""
        if self in another_link.get_all_left_links():
            return True, 'left'

        elif self in another_link.get_all_right_links():
            return True, 'right'

        else:
            return False, ''

    """ 데이터 양 쪽 차선(lane_mark) 정보 관련 > id -> object로 """
    def set_lane_mark_left(self, lane_mark):
        if type(lane_mark).__name__ != 'LaneBoundary':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(lane_mark)))

        if type(self.lane_mark_left) is None or type(self.lane_mark_left) is str:
            self.lane_mark_left = []

        self.lane_mark_left.append(lane_mark)

    def set_lane_mark_right(self, lane_mark):
        if type(lane_mark).__name__ != 'LaneBoundary':
            raise BaseException('[ERROR] unexpected link type: {}'.format(type(lane_mark)))

        if type(self.lane_mark_right) is None or type(self.lane_mark_right) is str:
            self.lane_mark_right = []

        self.lane_mark_right.append(lane_mark)

    def get_lane_mark_left(self):
        return self.lane_mark_left

    def get_lane_mark_right(self):
        return self.lane_mark_right

    def set_link_type(self, link_type, type_def=''):
        self.link_type = link_type
        self.link_type_def = type_def

    def get_lane_marking_list_to_string(self, lane_boundary_list):
        if lane_boundary_list is None:
            return []
        else:
            lane_boundary_list_str = []
            for lane_boundary in lane_boundary_list:
                lane_boundary_list_str.append(lane_boundary.idx)
            
            return lane_boundary_list_str

    def set_values_for_lane_change_link(self, lane_change_path):
        '''
        본 링크가 차선 변경을 표현하고자하는 링크일 때, 
        lane_change_path = [A, B, C, D] 와 같은 식으로 넣어주면 된다. 
        - from_node는 A의 from_node,
          to_node  는 D의 to_node,
        - lane_change_pair_list는 [from A -> to B], [from B -> to C], [from C -> to D]
        '''
        if not self.lazy_point_init:
            raise BaseException('lazy_point_init is True => USE Line.set_points_using_node instead of this!! (cannot use set_points_using_node_lazy_init)')
        
        if len(lane_change_path) < 2:
            raise BaseException('len(lane_change_path) must be >= 2 !! length of the current input = {}'.format(len(lane_change_path)))

        # from node, to node 설정 
        from_node = lane_change_path[0].get_from_node()
        to_node = lane_change_path[-1].get_to_node()
        # NOTE: 이미 해당 링크가 먼저 파일등으로부터 로드되었다면, from_node, to_node 등은 미리 설정되어있을 것이다.
        # 전달된 데이터의 오류로 링크의 노드 정보가 없을 수 있기 때문에 예외처리 코드 추가
        if from_node is None or to_node is None:
            return

        # NOTE : https://morai.atlassian.net/browse/MS-62
        self.set_from_node(from_node)
        self.set_to_node(to_node)

        # points 설정
        p1 = from_node.point
        p2 = to_node.point
        points = p1
        points = np.vstack((points, p2))
        self.set_points(points)

        # 
        lane_change_pair_list = [] 
        for i in range(len(lane_change_path) - 1):
           lane_change_pair_list.append({'from': lane_change_path[i], 'to': lane_change_path[i+1]})
        self.set_lane_change_pair_list(lane_change_pair_list)

    def set_max_speed_kph(self, max_speed):
        self.max_speed = max_speed

    def set_min_speed_kph(self, min_speed):
        self.min_speed = min_speed

    def set_recommended_speed_kph(self, recommended_speed):
        self.recommended_speed = recommended_speed

    def set_speed_unit(self, unit):
        self.speed_unit = unit

    def set_speed_region(self, start, end=None):
        self.speed_offset.append(start)

    def get_max_speed_kph(self):
        return self.max_speed

    def get_min_speed_kph(self):
        return self.min_speed

    def get_recommended_speed_kph(self):
        return self.recommended_speed

    def set_width(self, width):
        if width is None:
            return
        self.width = width
        
    def set_width_related_values(self, force_width_start, width_start, force_width_end, width_end):
        self.force_width_start = force_width_start
        self.width_start = width_start
        self.force_width_end = force_width_end
        self.width_end = width_end

    def get_width(self):
        return self.width

    def get_offset(self):
        return self.offset

    def calculate_cost(self):
        '''
        points 필드를 바탕으로, cost를 계산한다.
        set_points가 초기화코드에서 호출되면서 point가 설정이 안 된 채로 호출될 수 있는데,
        이 때는 그냥 리턴한다. (TODO: 향후 코드 개선 필요.
        이건 사실 근본적으로 Line쪽의 문제임. ctor에서는 set_points를 호출하지 않든지 해야 함)
        '''
        if self.points is None:
            # Logger.log_warning('calculate_cost is called without points attribute initialized')
            return 
        

        # 거리 계산
        # TODO: 해당 차로의 속도를 생각해서, 시간을 기준으로 고려할 것
        if self.is_it_for_lane_change():
            # 변경해서 들어갈 마지막 차선의 distance로 계산한다
            # NOTE: 중요한 가정이 있음. 차선 변경 진입 전후의 링크 길이가 거의 같아야 한다
            # 차선 변경 진입 후 링크가 너무 길다거나 하면 차선 변경 링크 생성 이전에 편집이 필요

            lane_ch_pair_list = self.get_lane_change_pair_list()
            last_to_link = lane_ch_pair_list[-1]['to']
            distance = last_to_link.get_total_distance()
        else:
            distance = self.get_total_distance()


        # 차선 변경에 따른 cost 계산
        # TODO: 해당 차로의 속도를 생각해서, 차선 변경 가능 시간을 기준으로 고려할 것
        def calc_lane_change_cost(x):  
            # 기준이 되는 값
            x_org = [10, 50, 100, 500, 1000, 2000]
            y_org = [500, 300, 200,  50, 20, 10]
            return np.interp(x, x_org, y_org, left=float('inf'), right=y_org[-1])

        lane_ch_pair_list = self.get_lane_change_pair_list()
        if self.is_it_for_lane_change():
            # 차선 변경이 3번이면, 전체 링크 길이를 L이라 할 때
            # L/3 인 차선 변경 cost를 계산한 다음, 3을 곱하여 전체 차선 변경 penalty를 계산
            lc_num = self.get_number_of_lane_change()
            unit_distance = distance / lc_num
            lane_change_penalty = lc_num * calc_lane_change_cost(unit_distance)
        else:
            lane_change_penalty = 0            

        self.cost = distance + lane_change_penalty

    def draw_plot(self, axes):

        # 그려야하는 width와 color가 지정되어 있으면 해당 값으로만 그린다
        if self.vis_mode_line_width is not None and \
            self.vis_mode_line_color is not None:
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                linewidth=self.vis_mode_line_width,
                color=self.vis_mode_line_color,
                markersize=2,
                marker='o')
            return
            
        if self.get_vis_mode_all_different_color():
            # 모두 다르게 그리라고하면, 색을 명시하지 않으면 된다
            self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                markersize=2,
                marker='o')
                
        else:
            # 이 경우에는 선의 종류에 따라 정해진 색과 모양으로 그린다
            if not self.lazy_point_init:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    linewidth=1,
                    markersize=2,
                    marker='o',
                    color='k')
            else:
                self.plotted_obj = axes.plot(self.points[:,0], self.points[:,1],
                    linewidth=1,
                    markersize=2,
                    marker='o',
                    color='b')


    @staticmethod
    def copy_attributes(src, dst):
        dst.lane_group = src.lane_group
        dst.lane_change_pair_list = src.lane_change_pair_list
        
        
        dst.max_speed = src.max_speed
        dst.min_speed = src.min_speed
        dst.link_type = src.link_type
        
        
        dst.road_id = src.road_id
        dst.ego_lane = src.ego_lane
        dst.lane_change_dir = src.lane_change_dir
        dst.hov = src.hov

        dst.opp_traffic = src.opp_traffic
        dst.is_entrance = src.is_entrance
        dst.is_exit = src.is_exit



    def is_dangling_link(self):
        if self.from_node is None or self.to_node is None:
            return True
        else:
            return False


    def has_location_error_node(self):
        sp_distance = 0.0
        ep_distance = 0.0

        if self.from_node:
            pos_vect = self.points[0] - self.from_node.point
            sp_distance = np.linalg.norm(pos_vect)

        if self.to_node:
            pos_vect = self.points[len(self.points) - 1] - self.to_node.point
            ep_distance = np.linalg.norm(pos_vect)

        if sp_distance < 1.0 and ep_distance < 1.0:
            return False
        else:
            return True


    def to_dict(self):
        """json 파일 등으로 저장할 수 있는 dict 데이터로 변경한다"""
        
        # 차선 변경으로 진입 가능한 차선 정보    
        if not self.is_it_for_lane_change():
            # 일반 링크이면
            if self.get_left_lane_change_dst_link() is None:
                left_lane_change_dst_link_idx = None
            else:
                left_lane_change_dst_link_idx = self.get_left_lane_change_dst_link().idx
            
            if self.get_right_lane_change_dst_link() is None:
                right_lane_change_dst_link_idx = None
            else:
                right_lane_change_dst_link_idx = self.get_right_lane_change_dst_link().idx
        else:
            # 차선 변경 링크이면
            left_lane_change_dst_link_idx = None
            right_lane_change_dst_link_idx = None

        # 양 옆에 차선 정보
        lane_mark_left_idx_list = []
        if self.get_lane_mark_left() is []:
            pass
        elif self.get_lane_mark_left() is None:
            pass
        else:
            for lane_boundary in self.get_lane_mark_left():
                lane_mark_left_idx_list.append(lane_boundary.idx)
        
        lane_mark_right_idx_list = []
        if self.get_lane_mark_right() is []:
            pass
        elif self.get_lane_mark_right() is None:
            pass
        else:
            for lane_boundary in self.get_lane_mark_right():
                lane_mark_right_idx_list.append(lane_boundary.idx)

        # 차선 변경 링크인 경우, 차선 변경 Path 
        lane_ch_link_path = []
        pair_list = self.get_lane_change_pair_list()
        for i in range(len(pair_list)):
            pair = pair_list[i]

            lane_ch_link_path.append(pair['from'].idx)

            # 마지막이면, 
            if (i == len(pair_list) - 1):
                lane_ch_link_path.append(pair['to'].idx)

        dict_data = {
            'idx': self.idx,
            'from_node_idx': self.from_node.idx if self.from_node else None,
            'to_node_idx': self.to_node.idx if self.to_node else None,
            'points': self.points.tolist(),
            # 'points': [['%0.20f'% x for x in y] for y in self.points],
            'max_speed': self.max_speed,
            'min_speed': self.min_speed,
            'lazy_init': self.lazy_point_init,
            'can_move_left_lane': self.can_move_left_lane,
            'can_move_right_lane': self.can_move_right_lane,
            'left_lane_change_dst_link_idx': left_lane_change_dst_link_idx,
            'right_lane_change_dst_link_idx': right_lane_change_dst_link_idx,
            'lane_ch_link_path': lane_ch_link_path,
            'link_type': self.link_type,
            'link_type_def': self.link_type_def,
            'road_type': self.road_type,
            'road_id': self.road_id,
            'ego_lane': self.ego_lane,
            'lane_change_dir': self.lane_change_dir,
            'hov': self.hov,
            'geometry': self.geometry,         
            'related_signal': self.related_signal,
            'its_link_id': self.its_link_id,  
            'force_width_start': self.force_width_start,
            'width_start': self.width_start,
            'force_width_end': self.force_width_end,
            'width_end': self.width_end,
            'enable_side_border': self.enable_side_border,
            'lane_mark_left': lane_mark_left_idx_list,
            'lane_mark_right': lane_mark_right_idx_list,
            'opp_traffic': self.opp_traffic,
            'is_entrance': self.is_entrance,
            'is_exit': self.is_exit,
            'speed_unit': self.speed_unit,
            'speed_offset': self.speed_offset,
            'speed_list': self.speed_list,
            'recommended_speed': self.recommended_speed
        }
        
        return dict_data


    @staticmethod
    def from_dict(dict_data, link_set=None):
        pass


    @classmethod
    def get_id_list_string(cls, list_obj):
        ret_str = '['
        for obj in list_obj:
            ret_str += '{}, '.format(obj.idx)
        ret_str += ']'
        ret_str = ret_str.replace(', ]', ']')
        return ret_str


    @staticmethod
    def get_default_width_related_values():
        return False, 3.5, False, 3.5


    def item_prop(self):
        prop_data = OrderedDict()
        prop_data['idx'] = {'type' : 'string', 'value' : self.idx}
        prop_data['points'] = {'type' : 'list<list<float>>', 'value' : self.points.tolist()}
        prop_data['from_node'] = {'type' : 'string', 'value' : self.from_node.idx if self.from_node else None}
        prop_data['to_node'] = {'type' : 'string', 'value' : self.to_node.idx if self.to_node else None}
        prop_data['can_move_left_lane'] = {'type' : 'boolean', 'value' : self.can_move_left_lane}
        prop_data['can_move_right_lane'] = {'type' : 'boolean', 'value' : self.can_move_right_lane}
        prop_data['lane_ch_link_left'] = {'type' : 'string', 'value' : self.lane_ch_link_left.idx if self.lane_ch_link_left else None}
        prop_data['lane_ch_link_right'] = {'type' : 'string', 'value' : self.lane_ch_link_right.idx if self.lane_ch_link_right else None}
        prop_data['max_speed_kph'] = {'type' : 'int', 'value' : self.get_max_speed_kph()}
        prop_data['min_speed_kph'] = {'type' : 'int', 'value' : self.get_min_speed_kph()}
        prop_data['link_type_def'] = {'type' : 'string', 'value' : self.link_type_def}
        prop_data['link_type'] = {'type' : 'string', 'value' : self.link_type}
        prop_data['road_type'] = {'type' : 'string', 'value' : self.road_type}
        prop_data['road_id'] = {'type' : 'string', 'value' : self.road_id}
        prop_data['ego_lane'] = {'type' : 'int', 'value' : self.ego_lane}
        prop_data['hov'] = {'type' : 'boolean', 'value' : self.hov}
        prop_data['related_signal'] = {'type' : 'string', 'value' : self.related_signal}
        prop_data['its_link_id'] = {'type' : 'string', 'value' : self.its_link_id}
        prop_data['geometry'] = {'type' : 'list<dict>', 'value' : self.geometry}
        prop_data['force width (start)'] = {'type' : 'boolean', 'value' : self.force_width_start}
        prop_data['width_start'] = {'type' : 'float', 'value' : self.width_start}
        prop_data['force width (end)'] = {'type' : 'boolean', 'value' : self.force_width_end}
        prop_data['width_end'] = {'type' : 'float', 'value' : self.width_end}
        prop_data['side_border'] = {'type' : 'boolean', 'value' : self.enable_side_border}
        prop_data['lane_mark_left'] = {'type' : 'list<string>', 'value' : self.get_lane_marking_list_to_string(self.lane_mark_left)}
        prop_data['lane_mark_right'] = {'type' : 'list<string>', 'value' : self.get_lane_marking_list_to_string(self.lane_mark_right)}
        prop_data['opp_traffic'] = {'type' : 'boolean', 'value' : self.opp_traffic}
        prop_data['is_entrance'] = {'type' : 'boolean', 'value' : self.is_entrance}
        prop_data['is_exit'] = {'type' : 'boolean', 'value' : self.is_exit}
        prop_data['speed_unit'] = {'type' : 'string', 'value' : self.speed_unit}
        prop_data['speed_offset'] = {'type' : 'list<float>', 'value' : self.speed_offset}
        prop_data['speed_list'] = {'type' : 'dict', 'value' : self.speed_list}
        prop_data['recommended_speed'] = {'type' : 'int', 'value' : self.get_recommended_speed_kph()}

        return prop_data