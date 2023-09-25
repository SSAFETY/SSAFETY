#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

import numpy as np
import json
import datetime
import hashlib
import copy

# 이 support.py 만 릴리즈 시 변경
from class_defs.support import supported_class

from class_defs.node import Node
from class_defs.signal import Signal
from class_defs.lane_boundary import LaneBoundary
from class_defs.junction_set import JunctionSet
from class_defs.node_set import NodeSet
from class_defs.line_set import LineSet
from class_defs.lane_boundary_set import LaneBoundarySet
from class_defs.road_polygon import RoadPolygon

from class_defs.signal_set import SignalSet

if supported_class['synced_signal']:
    from class_defs.synced_signal import SyncedSignal
    from class_defs.synced_signal_set import SyncedSignalSet

if supported_class['intersection_controller']:
    from class_defs.intersection_controller import IntersectionController
    from class_defs.intersection_controller_set import IntersectionControllerSet

from class_defs.surface_marking import SurfaceMarking
from class_defs.surface_marking_set import SurfaceMarkingSet
from class_defs.crosswalk_set import CrossWalkSet
from class_defs.singlecrosswalk_set import SingleCrosswalkSet
from class_defs.crosswalk import Crosswalk
from class_defs.singlecrosswalk import SingleCrosswalk
from class_defs.road_polygon_set import RoadPolygonSet

from class_defs.parking_space import ParkingSpace
from class_defs.parking_space_set import ParkingSpaceSet

from pyproj import Proj

class MGeo():
    def __init__(self, 
        node_set=NodeSet(), link_set=LineSet(), lane_boundary_set = LaneBoundarySet(), lane_node_set=NodeSet(), junction_set=JunctionSet(),
        sign_set=SignalSet(), light_set=SignalSet(), synced_light_set=SyncedSignalSet(), intersection_controller_set=IntersectionControllerSet(), 
        sm_set=SurfaceMarkingSet(), scw_set = SingleCrosswalkSet(), cw_set = CrossWalkSet(), road_polygon_set=RoadPolygonSet(), parking_space_set=ParkingSpaceSet()):
        '''
        반드시 MGeoPlannerMap은 node_set, link_set을 가지고 있어야 함
        Ctor에 전달하면서 init한다

        ver2.1 -> ver2.2 update: link 출력에 max_speed 추가 
        ver2.2 -> ver2.3 update: junction 클래스 추가
        ver2.3 -> ver2.4 update: code42 지도 데이터 추가
        ver2.4 -> ver2.5 update: junction을 list 형태로 변경
        ver2.5 -> ver2.6 update: 선의 fitting 방식을 달리하도록 데이터 필드 추가
        ver2.6 -> ver2.7 update: surface marking set 추가
        ver2.7 -> ver2.8 update: link의 related signal 정의 변경
        ver2.8 -> ver2.9 update: traffic_dir, country, road_type, road_type_def, workspace_origin 추가
        '''
        # primitive 
        self.node_set = node_set
        self.link_set = link_set
        self.junction_set = junction_set
        
        self.sign_set = sign_set
        self.light_set = light_set
        self.synced_light_set = synced_light_set
        self.intersection_controller_set = intersection_controller_set
        self.intersection_state_list = None
        
        self.lane_boundary_set = lane_boundary_set
        self.lane_node_set = lane_node_set
        
        self.sm_set = sm_set
        self.scw_set = scw_set
        self.cw_set = cw_set
        self.road_polygon_set = road_polygon_set
        self.parking_space_set = parking_space_set

        # 교통 관련 정보
        self.traffic_dir = '' # 'RHT' or 'LHT'
        self.country = ''
        self.road_type = ''
        self.road_type_def = '' # i.e. OpenDRIVE

        # 차선 변경 링크 포함 정보
        self.lane_change_link_included = False

        # 좌표계 정보
        self.global_coordinate_system = ''
        self.local_origin_in_global = np.array([0, 0, 0])
        self.workspace_origin = np.array([0, 0, 0])

        # 기타 정보
        self.maj_ver = 2
        self.min_ver = 9
        
        # 파일 정보
        self.saved_utc_time = ''
        self.mgeo_file_hash = {}


    def set_coordinate_system_from_prj_file(self, prj_file):
        """SHP 파일 등에 포함되는 .prj 파일을 읽고 표준 proj4 string 포맷의 값으로 변환 & 저장한다.
        GDAL package를 필요로 한다. 
        """
        self.global_coordinate_system = MGeo.esri_prj_to_proj4_string(prj_file)


    def set_origin(self, origin):
        if isinstance(origin, np.ndarray):
            self.local_origin_in_global = origin
        else:
            self.local_origin_in_global = np.array(origin)


    def get_origin(self):
        return self.local_origin_in_global


    def convert_local_to_global(self, pointArray):
        if (self.local_origin_in_global == np.array([0, 0, 0])).all():
            return pointArray
        
        glob_pos = self.local_origin_in_global + pointArray

        return glob_pos
    
    
    def to_json(self, output_path):        
        MGeo.save_node(output_path, self.node_set)
        MGeo.save_link(output_path, self.link_set)

        if self.sign_set is not None:
            MGeo.save_traffic_sign(output_path, self.sign_set)
        
        if self.light_set is not None:
            MGeo.save_traffic_light(output_path, self.light_set)

        if self.synced_light_set is not None:
            MGeo.save_synced_traffic_light(output_path, self.synced_light_set)

        if self.intersection_controller_set is not None:
            MGeo.save_intersection_controller(output_path, self.intersection_controller_set)
        
        if self.intersection_state_list is not None:
            MGeo.save_intersection_state(output_path, self.intersection_state_list)

        if self.sm_set is not None:
            MGeo.save_surface_marking(output_path, self.sm_set)

        if self.cw_set is not None:
            MGeo.save_crosswalk(output_path, self.cw_set)
        
        if self.scw_set is not None:
            MGeo.save_single_crosswalk(output_path, self.scw_set)

        if self.lane_boundary_set is not None:
            MGeo.save_lane_boundary(output_path, self.lane_boundary_set)

        if self.lane_node_set is not None:
            MGeo.save_lane_node(output_path, self.lane_node_set)
        
        if self.road_polygon_set is not None :
            MGeo.save_road_polygon(output_path, self.road_polygon_set)
            
        if self.parking_space_set is not None :
            MGeo.save_parking_space(output_path, self.parking_space_set)

        # 모든 설정 파일들이 저장된 후 hash 를 생성하기 위해 global_info 를 가장 나중에 저장하도록 한다.
        MGeo.save_global_info(output_path, self)

    def get_file_hash_sha256(self, file_path):
        hash_string = ''
        if os.path.exists(file_path) and os.path.isfile(file_path):
            with open(file_path, 'rb') as f:
                hash_string = hashlib.sha256(f.read()).hexdigest()
        return hash_string


    def generate_mgeo_file_hash(self, data_dir):        
        hash_info = {}
        hash_files = os.listdir(data_dir)

        for file_name in hash_files:
            # 해시값을 계산하여 global_info.json 파일에 저장해야하기 때문에 global_info.json 파일은 해시값을 계산하지 않는다.
            if file_name == 'global_info.json':
                continue

            hash_string = self.get_file_hash_sha256(os.path.join(data_dir, file_name))
            if len(hash_string) > 0:
                hash_info[file_name] = hash_string

        return hash_info


    def remove_duplicate_data_from_both_dict(self, dict1, dict2):
        copy_dict1 = dict1.copy()
        copy_dict2 = dict2.copy()

        for key, value in dict1.items():
            if dict2.get(key) and dict2[key] == value:
                copy_dict1.pop(key)
                copy_dict2.pop(key)

        return copy_dict1, copy_dict2


    def check_mego_data(self, data_dir):
        """
        현재 폴더에 존재하는 파일로 계산한 해시 정보(file_hash)와 데이터를 저장할 때 계산했던 해시 정보(saved_hash)에서 
        같은 값을 제거하고 데이터가 남은 경우는 아래와 같다.
        1. file_hash (O), saved_hash(O) : 데이터가 변경됨
        2. file_hash (O), saved_hash(X) : 데이터가 추가됨
        3. file_hash (X), saved_hash(O) : 데이터가 삭제됨
        """
        file_hash, saved_hash = self.remove_duplicate_data_from_both_dict(self.generate_mgeo_file_hash(data_dir), self.mgeo_file_hash)        
        error_info = {}
        for file_name, hash_value in file_hash.items():
            if saved_hash.get(file_name):
                error_info.setdefault('Changed', [])
                error_info['Changed'].append(file_name)
            else:
                error_info.setdefault('Added', [])
                error_info['Added'].append(file_name)
        for file_name, hash_value in saved_hash.items():
            if not file_hash.get(file_name):
                error_info.setdefault('Removed', [])
                error_info['Removed'].append(file_name)

        return '' if len(error_info) == 0 else str(error_info)


    def get_country_name_iso3166_alpha2(self):
        import iso3166

        try:
            country_obj = iso3166.countries.get(self.country)
            return country_obj.alpha2

        except KeyError as e:
            return ''
            

    @staticmethod
    def esri_prj_to_proj4_string(prj_file):
        """SHP 파일 등에 포함되는 .prj 파일을 읽고 표준 proj4 string 포맷의 값으로 변환한다.
        GDAL package를 필요로 한다. 
        """
        from osgeo import osr

        prj_file = open(prj_file, 'r')
        prj_txt = prj_file.read()
        srs = osr.SpatialReference()
        srs.ImportFromESRI([prj_txt])
        
        Proj4 = srs.ExportToProj4()
        return Proj4


    @staticmethod
    def save_global_info(output_path, obj):
        # 해시 생성에 사용하기 위해 현재 시간을 먼저 저장함
        obj.saved_utc_time = str(datetime.datetime.utcnow())

        # 클래스 내 필드에서 global_info 설정
        global_info = {
            'saved_utc_time': obj.saved_utc_time,
            'mgeo_file_hash': str(obj.generate_mgeo_file_hash(output_path)),
            'maj_ver': obj.maj_ver,
            'min_ver': obj.min_ver,
            'global_coordinate_system': obj.global_coordinate_system, # 
            'local_origin_in_global': obj.local_origin_in_global.tolist(), # origin
            'workspace_origin': obj.workspace_origin.tolist(),
            'lane_change_link_included': obj.lane_change_link_included, 
            'traffic_dir': obj.traffic_dir,
            'country': obj.country,
            'road_type': obj.road_type,
            'road_type_def':  obj.road_type_def,
            'license':'MORAI Inc.'
        }
        filename = os.path.join(output_path, 'global_info.json')
        with open(filename, 'w') as f:
            json.dump(global_info, f, indent=2)
    
    
    @staticmethod
    def save_node(output_path, node_set):
        # 각 노드는 id와 point를 저장한다.
        save_info_list = []
        for var, node in node_set.nodes.items():
            dict_data = node.to_dict()
            save_info_list.append(dict_data)

        # 이를 저장한다.
        filename = os.path.join(output_path, 'node_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_link(output_path, link_set):
        save_info_list = []
        for idx, line in link_set.lines.items():
            dict_data = line.to_dict()
            save_info_list.append(dict_data)
            
        filename = os.path.join(output_path, 'link_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)        


    @staticmethod
    def save_traffic_light(output_path, light_set): 
        save_info_list = []
        for var, tl in light_set.signals.items():
            dict_data = Signal.to_dict(tl)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'traffic_light_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_synced_traffic_light(output_path, synced_light_set): 
        save_info_list = []
        for var, synced_tl in synced_light_set.synced_signals.items():
            dict_data = SyncedSignal.to_dict(synced_tl)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'synced_traffic_light_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_intersection_controller(output_path, intersection_controller_set): 
        save_info_list = []
        for var, ic in intersection_controller_set.intersection_controllers.items():
            dict_data = IntersectionController.to_dict(ic)
            save_info_list.append(dict_data)
      
        file_path = os.path.join(output_path, 'intersection_controller_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_traffic_sign(output_path, sign_set):
        save_info_list = []
        for var, ts in sign_set.signals.items():           
            dict_data = Signal.to_dict(ts)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'traffic_sign_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)


    @staticmethod
    def save_surface_marking(output_path, sm_set):
        save_info_list = []
        for key, sm in sm_set.data.items():
            dict_data = SurfaceMarking.to_dict(sm)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'surface_marking_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def save_road_polygon(output_path, rp_set) :
        save_info_list = []
        for key, rp in rp_set.data.items():
            dict_data = RoadPolygon.to_dict(rp)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'road_polygon_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)
    
    @staticmethod
    def save_parking_space(output_path, ps_set):
        save_info_list = []
        for key, scw in ps_set.data.items():
            dict_data = ParkingSpace.to_dict(scw)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'parking_space_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def save_crosswalk(output_path, cw_set):
        save_info_list = []
        for key, cw in cw_set.data.items():
            dict_data = cw.to_dict(cw)
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'crosswalk_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)
    
    @staticmethod
    def save_single_crosswalk(output_path, scw_set):
        save_info_list = []
        for key, scw in scw_set.data.items():
            dict_data = scw.to_dict()
            save_info_list.append(dict_data)

        # json 파일로 저장
        file_path = os.path.join(output_path, 'singlecrosswalk_set.json')
        with open(file_path, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def save_lane_boundary(output_path, lane_boundary_set):
        save_info_list = []
        for idx, lane in lane_boundary_set.lanes.items():
            dict_data = lane.to_dict()
            save_info_list.append(dict_data)
            
        filename = os.path.join(output_path, 'lane_boundary_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)     
        
    @staticmethod
    def save_lane_node(output_path, lane_node):
        # 각 노드는 id와 point를 저장한다.
        save_info_list = []
        for var, node in lane_node.nodes.items():
            dict_data = node.to_dict()
            save_info_list.append(dict_data)

        # 이를 저장한다.
        filename = os.path.join(output_path, 'lane_node_set.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)
    
    @staticmethod
    def save_intersection_state(output_path, intscn_states):
        # 각 Intersection Controller의 state를 저장한다.
        save_info_list = []
        for idx, state in intscn_states.items():
            save_info_list.append(state)
        
        # 이를 저장한다.
        filename = os.path.join(output_path, 'intersection_controller_data.json')
        with open(filename, 'w') as f:
            json.dump(save_info_list, f, indent=2)

    @staticmethod
    def load_node_and_link(folder_path):
        '''
        파일을 읽어 global_info, node_set, link_set을 생성하여 리턴한다
        MGeo ver2.1 까지 지원
        '''
        # node_set, link_set은 버전 정보와 관계없이 공통이다.
        filename = os.path.join(folder_path, 'node_set.json')
        with open(filename, 'r') as f:
            node_save_info_list = json.load(f)

        filename = os.path.join(folder_path, 'link_set.json')
        with open(filename, 'r') as f:
            line_save_info_list = json.load(f)


        # 버전 정보를 찾는다
        # 버전 파일이 없으면, ver1이다.
        # global_info.json 대신 global_info.mprj 파일이 있으면 mprj 파일로 읽어오기
        filename = os.path.join(folder_path, 'global_info.json')
        if os.path.exists(os.path.join(folder_path, 'global_info.mprj')):
            filename = os.path.join(folder_path, 'global_info.mprj')
        if not os.path.isfile(filename):

            from save_load import subproc_load_link_ver1
            node_set, link_set = subproc_load_link_ver1.load_node_and_link(node_save_info_list, line_save_info_list)

            # ver1에서는 global_info가 없으므로, 직접 생성해준다
            global_info = {
                'maj_ver': 1,
                'min_ver': 0,
                'global_coordinate_system': '+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs',
                'local_origin_in_global': [0, 0, 0]
            }

            return global_info, node_set, link_set

        # 읽을 버전 정보 파일이 있는 경우    
        with open(filename, 'r') as f:
            global_info = json.load(f)

        # 버전 정보에 맞게 node_set, link_set을 읽어온다.
        if global_info['maj_ver'] == 2:

            from save_load import subproc_load_link_ver2
            node_set, link_set, junction_set = subproc_load_link_ver2.load_node_and_link(
                node_save_info_list, line_save_info_list, global_info)

        return global_info, node_set, link_set, junction_set


    @staticmethod
    def load_traffic_sign(folder_path, link_set):
        """traffic_sign_set.json 파일을 읽고 표지판 셋 (ts_set)을 생성한다"""
        ts_set = SignalSet()
        filename = os.path.join(folder_path, 'traffic_sign_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return ts_set

        for each_info in saved_info:
            ts = Signal.from_dict(each_info, link_set)
            ts_set.append_signal(ts)

        return ts_set


    @staticmethod
    def load_traffic_light(folder_path, link_set):
        """traffic_light_set.json 파일을 읽고 표지판 셋 (tl_set)을 생성한다"""
        tl_set = SignalSet()
        filename = os.path.join(folder_path, 'traffic_light_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return tl_set

        for each_info in saved_info:
            tl = Signal.from_dict(each_info, link_set)
            tl_set.append_signal(tl)

        return tl_set


    @staticmethod
    def load_synced_traffic_light(folder_path, link_set, tl_set):
        """synced_traffic_light_set.json 파일을 읽고 synced_tl_set을 생성한다"""
        synced_tl_set = SyncedSignalSet()
        filename = os.path.join(folder_path, 'synced_traffic_light_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return synced_tl_set

        for each_info in saved_info:
            synced_tl = SyncedSignal.from_dict(each_info, link_set, tl_set)
            synced_tl_set.append_synced_signal(synced_tl)

        return synced_tl_set


    @staticmethod
    def load_intersection_controller(folder_path, light_set):
        """synced_traffic_light_set.json 파일을 읽고 synced_tl_set을 생성한다"""
        ic_set = IntersectionControllerSet()
        filename = os.path.join(folder_path, 'intersection_controller_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return ic_set

        for each_info in saved_info:
            ic = IntersectionController.from_dict(each_info, light_set)
            ic_set.append_controller(ic)

        return ic_set


    @staticmethod
    def load_intersection_controller_state(folder_path):
        intersection_state_list = None
        filename = os.path.join(folder_path, 'intersection_controller_data.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return intersection_state_list

        intersection_state_list = dict()
        for each_info in saved_info:
            intersection_state = dict()
            idx = each_info['idx']
            intersection_state['idx'] = each_info['idx']
            intersection_state['TLState'] = each_info['TLState']
            intersection_state['yelloduration'] = each_info['yelloduration']
            intersection_state['PSState'] = each_info['PSState']
            intersection_state_list[idx] = intersection_state

        return intersection_state_list


    @staticmethod
    def load_surface_marking(folder_path, link_set):
        """surface_marking_set.json 파일을 읽고 surface_marking셋 (sm_set)을 생성한다"""
        sm_set = SurfaceMarkingSet()
        filename = os.path.join(folder_path, 'surface_marking_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return sm_set

        for each_info in saved_info:
            sm = SurfaceMarking.from_dict(each_info, link_set)
            # 횡단보도 제외하고 load 하기
            if sm.type == 5 or sm.type == '5':
                continue
            sm_set.append_data(sm)

        return sm_set
        

    @staticmethod
    def load_lane_boundary(folder_path):
        """lane_marking_set.json 파일을 읽고 lane_marking셋 (lane_set)을 생성한다"""

        node_set = NodeSet()
        lane_set = LaneBoundarySet()

        filename = os.path.join(folder_path, 'lane_node_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                node_save_info_list = json.load(f)
        else:
            return lane_set, node_set

        for save_info in node_save_info_list:
            idx = save_info['idx']
            point = save_info['point']
        
            node = Node(idx)
            node.point = np.array(point)

            node_set.append_node(node, create_new_key=False)
    
        filename_old = os.path.join(folder_path, 'lane_marking_set.json')
        filename_new = os.path.join(folder_path, 'lane_boundary_set.json')
        
        if os.path.isfile(filename_old):
            with open(filename_old, 'r') as f:
                saved_info = json.load(f)
        elif os.path.isfile(filename_new):
            with open(filename_new, 'r') as f:
                saved_info = json.load(f)
        else:
            return lane_set, node_set

        for each_info in saved_info:
            lane = LaneBoundary.from_dict(each_info, node_set)
            lane_set.append_line(lane)

        return lane_set, node_set

    @staticmethod
    def load_single_crosswalk(folder_path):
        scw_set = SingleCrosswalkSet()
        filename = os.path.join(folder_path, 'singlecrosswalk_set.json')

        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return scw_set
        
        for each_info in saved_info:
            scw = SingleCrosswalk.from_dict(each_info)
            scw_set.append_data(scw)

        return scw_set

    
    @staticmethod
    def load_crosswalk(folder_path, scw_set ,tl_set):
        cw_set = CrossWalkSet()
        filename = os.path.join(folder_path, 'crosswalk_set.json')

        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return cw_set
        
        for each_info in saved_info:
            cw = Crosswalk.from_dict(each_info, scw_set, tl_set)
            cw_set.append_data(cw)

        return cw_set


    @staticmethod
    def load_road_polygon(folder_path):
        rp_set = RoadPolygonSet()
        filename = os.path.join(folder_path, 'road_polygon_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return rp_set

        for each_info in saved_info:
            rp = RoadPolygon.from_dict(each_info)
            rp_set.append_data(rp)

        return rp_set

    @staticmethod
    def load_parking_space(folder_path):
        parking_space_set = ParkingSpaceSet()
        filename = os.path.join(folder_path, 'parking_space_set.json')
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                saved_info = json.load(f)
        else:
            return parking_space_set

        for each_info in saved_info:
            ps = ParkingSpace.from_dict(each_info)
            parking_space_set.append_data(ps)

        return parking_space_set


    @staticmethod
    def connect_link_and_lane_mark(link_set, lane_set):
        links = link_set.lines
        lanes = lane_set.lanes

        for link in links:
            # lane_mark_left 값이 list로 되어있지 않은 이전 데이터인 경우 예외 처리
            if links[link].lane_mark_left is None:
                links[link].lane_mark_left = []
            elif links[link].lane_mark_left is not None and type(links[link].lane_mark_left) is not list:
                links[link].set_lane_mark_left(lanes[links[link].lane_mark_left])
            else:
                copy_list = copy.deepcopy(links[link].lane_mark_left)
                links[link].lane_mark_left = []
                for lane_mark_left_id in copy_list:
                    if lane_mark_left_id in lanes:
                        links[link].set_lane_mark_left(lanes[lane_mark_left_id])

            # lane_mark_right 값이 list로 되어있지 않은 이전 데이터인 경우 예외 처리
            if links[link].lane_mark_right is None:
                links[link].lane_mark_right = []
            elif links[link].lane_mark_right is not None and type(links[link].lane_mark_right) is not list:
                links[link].set_lane_mark_right(lanes[links[link].lane_mark_right])
            else:
                copy_list = copy.deepcopy(links[link].lane_mark_right)
                links[link].lane_mark_right = []
                for lane_mark_right_id in copy_list:
                    if lane_mark_right_id in lanes:
                        links[link].set_lane_mark_right(lanes[lane_mark_right_id])

        return link_set


    @staticmethod
    def create_instance_from_json(folder_path):
        '''
        파일을 읽어서 MGeo 인스턴스를 생성한다
        '''
        # global_info.mprj 파일 선택할 수 있게 설정
        if not os.path.isdir(folder_path):
            folder_path = os.path.dirname(folder_path)

        global_info, node_set, link_set, junction_set = MGeo.load_node_and_link(folder_path)

        lane_boundary_set, lane_node_set = MGeo.load_lane_boundary(folder_path)
        
        # lane_marking_set이 있으면 link_set에다가 연결
        if len(lane_boundary_set.lanes) > 0:
            link_set = MGeo.connect_link_and_lane_mark(link_set, lane_boundary_set)

        sign_set = MGeo.load_traffic_sign(folder_path, link_set)
        light_set = MGeo.load_traffic_light(folder_path, link_set)
        
        if supported_class['synced_signal']:
            synced_light_set = MGeo.load_synced_traffic_light(folder_path, link_set, light_set)
        else:
            synced_light_set = None

        if supported_class['intersection_controller']:
            intersection_controller_set = MGeo.load_intersection_controller(folder_path, light_set)
            intersection_state_list = MGeo.load_intersection_controller_state(folder_path)
        else:
            intersection_controller_set = None
            intersection_state_list = None

        sm_set = MGeo.load_surface_marking(folder_path, link_set)
        
        scw_set = MGeo.load_single_crosswalk(folder_path)

        cw_set = MGeo.load_crosswalk(folder_path, scw_set, light_set)

        rp_set = MGeo.load_road_polygon(folder_path)
        
        ps_set = MGeo.load_parking_space(folder_path)
        
        # MGeo 생성 
        mgeo_planner_map = MGeo(node_set, link_set, lane_boundary_set, lane_node_set, junction_set, 
            sign_set, light_set, synced_light_set, intersection_controller_set, sm_set, scw_set, cw_set, rp_set, ps_set)

        # Intersection Controller 정보
        mgeo_planner_map.intersection_state_list = intersection_state_list

        # 버전 정보
        mgeo_planner_map.maj_ver = global_info['maj_ver']
        mgeo_planner_map.min_ver = global_info['min_ver']

        # 좌표 정보        
        if global_info['global_coordinate_system'].upper() == 'UTM52N':
            mgeo_planner_map.global_coordinate_system = '+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs'
        else:
            mgeo_planner_map.global_coordinate_system = global_info['global_coordinate_system']

        result, proj4 = MGeo.validationProjectionFormat(mgeo_planner_map.global_coordinate_system)
        if result:
            mgeo_planner_map.global_coordinate_system = proj4
        else:
            # TODO: proj format에 맞지않으면 exception? 에러로그? 
            pass
        
        mgeo_planner_map.local_origin_in_global = np.array(global_info['local_origin_in_global'])
        if global_info.get('workspace_origin'):
            mgeo_planner_map.workspace_origin = np.array(global_info['workspace_origin'])
        
        # 기타 교통 관련 정보
        if 'traffic_dir' in global_info:
            mgeo_planner_map.traffic_dir = global_info['traffic_dir']
        
        if 'country' in global_info:
            mgeo_planner_map.country = global_info['country']
        
        if 'road_type' in global_info:
            mgeo_planner_map.road_type = global_info['road_type']
        
        if 'road_type_def' in global_info:
            mgeo_planner_map.road_type_def = global_info['road_type_def']

        # 파일 정보
        if 'saved_utc_time' in global_info:
            mgeo_planner_map.saved_utc_time = global_info['saved_utc_time']
        
        if 'mgeo_file_hash' in global_info:
            mgeo_planner_map.mgeo_file_hash = json.loads(global_info['mgeo_file_hash'].replace('\'', '"'))

        # 차선 변경 링크 포함 정보
        if 'lane_change_link_included' in global_info:
            lane_change_link_included = global_info['lane_change_link_included']
            if lane_change_link_included:
                if next((item for i, item in mgeo_planner_map.link_set.lines.items() if item.lazy_point_init is True), False) is False:
                    lane_change_link_included = False
            mgeo_planner_map.lane_change_link_included = lane_change_link_included

        return mgeo_planner_map

    
    @staticmethod
    def validationProjectionFormat(data):
        try:
            proj4 = Proj(data).srs
            return True, proj4
        except BaseException as e: 
            return False, None
