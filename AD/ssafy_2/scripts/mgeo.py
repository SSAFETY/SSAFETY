#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo 는 정밀도로지도 데이터 인 MGeo(MORAI Geometry) 데이터를 읽어오는 예제입니다.
# Json 파일 형식으로 되어 있는 MGeo 데이터를 dictionary 형태로 읽어옵니다.

# 노드 실행 순서 
# 1. Mgeo data 읽어온 후 데이터 확인

#TODO: (1) Mgeo data 읽어온 후 데이터 확인
'''
# Json 파일 형식으로 저장된 MGeo 데이터를 읽어오는 예제 입니다.
# VScode 의 debug 기능을 이용하여 MGeo 데이터를 확인 할 수 있습니다.
# MGeo 데이터는 인접 리스트 방식의 그래프 구조 입니다.
# 정밀도로지도의 도로 간의 연결 관계를 표현 합니다.
# MGeo 에는 도로의 형상을 나타내는 Node 와 Link 데이터가 있습니다.
# Node 와 Link 는 모두 Point 데이터 들의 집합입니다.
# Node 는 서로 다른 두개 이상의 Link 간의 연결 여부를 나타냅니다.
# Link 는 도로를 표현하며 도로 의 중심 선이 됩니다.
# Link 와 Node 정보가 모여 도로의 형상을 표현합니다.
# 각각의 Node Link 정보는 이름인 idx 정보를 가집니다 idx 는 중복 될 수 없습니다. 
# to_links , from_links , to_node , from_node ... 등 
# MGeo에 정의되어 있는 데이터를 활용해 각 Node 와 Link 간 연결 성을 나타낼 수 있습니다.
# 

'''
load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
mgeo_planner_map = MGeo.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines

print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))