#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os

from morai_msgs.msg import GetTrafficLightStatus





def get_traffic_callback(msg):
    # self.is_get_traffic = True
    # traffic_data = msg
    
    os.system('clear')
    print('-------------------------------------')
    print('traffic data : {}'.format(msg))
    print('traffic data list : {}'.format(msg.trafficLightStatus))
    # rospy.loginfo("rosinfo traffic : ")
    # rospy.loginfo(msg)
    print('-------------------------------------')


def listener():
    rospy.init_node('get_traffic', anonymous=True)

    rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, get_traffic_callback)

    # self.is_get_traffic = False

    rospy.spin()



if __name__ == '__main__':
    Traffic = listener()
