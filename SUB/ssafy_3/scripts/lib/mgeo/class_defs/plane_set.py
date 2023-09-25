#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from .plane import Plane


class PlaneSet(object): # super method의 argument로 전달되려면 object를 상속해야함 (Python2에서)
    def __init__(self):
        self.planes = list()

    def reorganize(self):
        for i, plane in enumerate(self.planes):
            plane.idx = i
        
    def add_plane(self, plane):
        self.planes.append(plane)

    def remove_plane(self, plane_to_delete):
        self.planes.remove(plane_to_delete)

    def create_a_new_empty_plane(self):
        new_id = len(self.planes)
        self.add_plane(Plane(new_id))

    def get_last_plane(self):
        return self.planes[-1]

    def save_as_json(self, filename):
        import json

        obj_to_save = []
        for plane in self.planes:
            # 완성된 plane만 저장한다 (closed 상태인)
            if not plane.is_closed():
                continue

            obj_to_save.append({
                'node_idx':plane.get_node_idx_list()
                })

        with open(filename, 'w') as f:
            json.dump(obj_to_save, f)

    def load_from_json(self, node_set_obj, filename):
        import json

        with open(filename, 'r') as f:
            list_of_info_for_each_plane = json.load(f)
        
        # NOTE: 여기서 이전에 존재하던 self.planes에 포함된 인스턴스에 대한 reference를 제거한다
        # 즉, 기존에 있던 plane들 정보를 날려버린다.
        # 그렇다면, 해당 인스턴스들은 자동으로 GC되겠지만,
        # 혹시 특정 라이브러리 등과의 연동으로 인해, reference가 계속 남아있게 된다면
        # GC되지 않을 것이므로, 메모리 증가가 일어나지는 않는지 유의할 것
        # 단, 실사용에서는 이 메소드를 계속 호출하진 않을 것이므로 문제를 일으킬 것 같지는 않다
        
        self.planes = list()
        for info in list_of_info_for_each_plane:
            self.create_a_new_empty_plane()
            self.get_last_plane().init_from_node_idx_list(node_set_obj, info['node_idx'])
        
        # 마지막에 load한 plane이 수정되지 않게 새로운 empty plane을 추가하면서 끝낸다
        self.create_a_new_empty_plane()
        
    def _print(self):
        for plane in self.planes:
            print(plane.to_string())