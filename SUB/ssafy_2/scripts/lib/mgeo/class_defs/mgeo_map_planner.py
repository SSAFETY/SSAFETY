import os
import traceback

from lib.mgeo.class_defs.mgeo import MGeo

class MgeoMapPlanners:
    """
    Description : MGeo() Instance 는 이미 만들어진 상태에서, add_map 기능을 사용할수 있음
    그러므로, 이미 만들어진 또는 load 된 instance 를 parameter 에 넣어야함. 
    
    ex : {key1: {mgeo:MGeo1, path:path}, key2: Mgeo2....}
    """
    
    def __init__(self, map_list, instance=None): 
        # MGeo 의 객체의 고유 넘버 또는 객체 ID 및 path_name
        # self.loaded_path = loaded_path
        self.mgeo_maps_dict = map_list
        self.instance = None
        
        # For save_mgeo
        if instance != None:
            self.instance = instance
        
    def append_map(self, add_path):
        """
        Description : Appending the selected MGeo map data into existing the MGeo map data (Loaded MGeo Map Data)
        Variable    : path_name
        """ 
        
        # For secure purpose
        if not self.mgeo_maps_dict:
            return
        else:
            mgeo_planner_map = MGeo.create_instance_from_json(add_path)
            self.mgeo_maps_dict[str(add_path.split('/')[-1])] = mgeo_planner_map
         
    
    def clear_all_map(self):
        """
        Description : Remove all the map
        """
        self.mgeo_maps_dict.clear()
        