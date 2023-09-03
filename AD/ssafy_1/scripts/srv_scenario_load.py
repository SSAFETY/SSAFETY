#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from morai_msgs.msg import ScenarioLoad
from morai_msgs.srv import MoraiScenarioLoadSrv

# MoraiSL_client 는 Simulator의 시나리오를 제어하는 Client Node 작성 예제입니다.
# Simulatord의 시나리오 제어 메세지 송신 후 결과 값을 반환하는 Client Node 를 생성 합니다.

# 노드 실행 순서 
# 1. Service 가 생성 대기 함수 선언
# 2. 송신 될 메세지 변수 생성
# 3. Service 호출
# 4. Service 호출 결과 값 확인

def srv_client():
    rospy.init_node('MoraiSL_client', anonymous=True)
    #TODO: (1) Service 가 생성 대기 함수 선언
    rospy.wait_for_service('/Service_MoraiSL')

    #TODO: (2) 송신 될 메세지 변수 생성
    '''
    # 시뮬레이터로 송신 될 메세지 변수를 만든다.
    # 시뮬레이터에 저장 된 시나리오를 불러오는 통신 메세지이다.
    # 시뮬레이터에서 직접 시나리오를 만들어 저장한 뒤 저장된 파일 이름과 필요한 옵션 들을 설정하여 사용 가능하다.
    # file_name 은 String 형식이며, 나머지 옵션은 전부 Bool 형식으로 True/False 로 해당 옵션을 사용 여부를 선택 가능하다.
    '''
    scenario_setting = ScenarioLoad()
    scenario_setting.file_name                      = "test_scenario"
    scenario_setting.load_network_connection_data   = True
    scenario_setting.delete_all                     = False
    scenario_setting.load_ego_vehicle_data          = True
    scenario_setting.load_surrounding_vehicle_data  = True
    scenario_setting.load_pedestrian_data           = True
    scenario_setting.load_obstacle_data             = True
    scenario_setting.set_pause                      = False


    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            #TODO: (3) Service 호출
            '''
            # MoraiScenarioLoadSrv 라는 Morai ROS 서비스 형식을 사용하여 Service 호출 함수를 만든다.
            # Service 호출 이름은 시뮬레이터 Network 연결시 확인 가능하다.
                        
            '''
            ros_srv = rospy.ServiceProxy( '/Service_MoraiSL' ,MoraiScenarioLoadSrv )
            result = ros_srv(scenario_setting)


            #TODO: (4) Service 호출 결과 값 확인
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logwarn('no respone')

        rate.sleep()

if __name__ == '__main__':
    try:
        srv_client()
    except rospy.ROSInterruptException:
        pass
