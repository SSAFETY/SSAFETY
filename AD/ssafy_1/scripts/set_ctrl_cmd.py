#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

# Ego_Control_Command 는 Simulator 에서 Ego 차량을 움직임을 제어하는 메세지 송신의 예제입니다.
# /ctrl_cmd 라는 메세지를 Publish 하여 Ego 차량을 제어 합니다.

# 노드 실행 순서 
# 1. publisher 생성
# 2. 송신 될 메세지 변수 생성
# 3. /ctrl_cmd 메세지 Publish

def talker():
    #TODO: (1) publisher 생성
    '''
    # CtrlCmd 라는 Morai ROS 메세지 형식을 사용하여 Topic Publisher 를 완성한다.
    # Topic 이름은 시뮬레이터 Network 연결시 확인 가능하다.
    publisher = rospy.Publisher( 변수 1 , 변수 2 , queue_size=10)

    '''

    rospy.init_node('Ego_Control_Command', anonymous=True)
    publisher = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size = 10)
    

    #TODO: (2) 송신 될 메세지 변수 생성
    
    # 시뮬레이터로 송신 될 메세지 변수를 만든다.
    # CtrlCmd 메세지는 차량을 제어하는 메세지이다. Accel/Brake/Steering 세팅이 가능하다.
    # longlCmdType 은 차량에 제어 모드를 선택하는 값이다 (1: Throttle control, 2: Velocity control, 3: Acceleration control)
    # 가속, 브레이크 패달은 0 ~ 1 범위를 가지며 1은 가장 강하게 패달을 밟은 값이다.
    # Steering 은 차량의 앞 바퀴 각도를 의미하며 Rad 단위이다.
    # velocity, acceleration 은 각각 longlCmdType 값이 2,3 일때만 동작하며 차량의 속도 또는 가속도를 제어 입력 값으로 넣는다.
    # 원하는 제어 입력값을 넣은 뒤 시뮬레이터에서 차량의 변화를 관찰한다.
    ctrl_cmd = CtrlCmd()
    ctrl_cmd.longlCmdType = 1
    ctrl_cmd.accel = 5 
    ctrl_cmd.brake = 0
    ctrl_cmd.steering = 0 
    ctrl_cmd.velocity = 30 
    ctrl_cmd.acceleration = 5 

    
    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        rospy.loginfo(ctrl_cmd)
        #TODO: (3) /ctrl_cmd 메세지 Publish
        '''
        # ctrl_cmd 를 전송하는 publisher 를 만든다.
        publisher.
        
        '''
        publisher.publish(ctrl_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
