#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from morai_msgs.msg import EventInfo,Lamps
from morai_msgs.srv import MoraiEventCmdSrv

# MoraiEventCmd_client 는 Ego 차량의 상태를 제어하는 Client Node 작성 예제입니다.
# Ego 차량의 상태(전조등, 방향 지시등, 차량 Gear, 차량 제어 Mode)제어 메세지 송신 후 결과 값을 반환하는 Client Node 를 생성 합니다.

# 노드 실행 순서 
# 1. Service 가 생성 대기 함수 선언
# 2. 송신 될 메세지 변수 생성
# 3. Service 호출
# 4. Service 호출 결과 값 확인

def srv_client():
    rospy.init_node('MoraiEventCmd_client', anonymous=True)
    #TODO: (1) Service 가 생성 대기 함수 선언
    rospy.wait_for_service('/Service_MoraiEventCmd')

    #TODO: (2) 송신 될 메세지 변수 생성
    '''
    # 시뮬레이터로 송신 될 메세지 변수를 만든다.
    # 시뮬레이터에서 차량의 상태를 제어하는 옵션으로 차량의 control mode, 차량의 기어, 방향 지시 제어가 가능하다.
    # option 은 이벤틑 제어를 요청하는 필드 옵션으로 1 : ctrl_mode, 2 : gear, 4 : lamps, 8 : set_pause 로 구성되어 있습니다.
    # option 에 각 값을 입력하여 원하는 제어 값만을 사용 가능합니다. 두개 이상의 제어 입력을 같이 사용하기 위해서는 각 옵션 값의 숫자를 더하면 됩니다.
    # (ctrl_mode + gear --> 1 + 2 = 3 입력)
    # ctrl_mode = 1: Keyboard / 3: automode / 4 : cruisemode
    # gear = -1 : 이전 상태 유지 1 : P / 2 : R / 3 : N / 4 : D
    # Lamps() turnSignal = 0 : No Signal / 1 : Left Signal / 2 : Right Signal / 3 : 이전 상태 유지
    # Lamps() emergencySignal = 0 : No Signal / 1 : Emergency Signal
    # set_pause = True : 시뮬레이터 Pause 상태로 유지 / False : 시뮬레이터 Play상태로 전환
    lamp_cmd = Lamps()
    lamp_cmd.turnSignal = 1
    lamp_cmd.emergencySignal = 0
        
    set_Event_control = EventInfo()
    set_Event_control.option = 7
    set_Event_control.ctrl_mode = 3
    set_Event_control.gear = 4
    set_Event_control.lamps = lamp_cmd
    '''

    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        try:
            #TODO: (3) Service 호출
            '''
            # MoraiEventCmdSrv 라는 Morai ROS 서비스 형식을 사용하여 Service 호출 함수를 만든다.
            # Service 호출 이름은 시뮬레이터 Network 연결시 확인 가능하다.
            ros_srv = rospy.ServiceProxy( 변수 1 , 변수 2 )
            result = ros_srv( 변수 3 )
            
            '''

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
