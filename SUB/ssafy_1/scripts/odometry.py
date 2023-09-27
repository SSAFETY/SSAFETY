#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry



class Odometry_publisher:
    def __init__(self):
        rospy.init_node('Odometry_publisher', anonymous=True)
        self.utm_sub = rospy.Subscriber("/utm", Float32MultiArray, self.utm_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        
        self.is_utm=False
        self.is_imu=False
        self.utm_x = None
        self.utm_y = None
        
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_utm == True and self.is_imu==True:
                self.odom_pub.publish(self.odom_msg)
                print("data published at '/odom' topic !")
            else:
                print("waiting for data...")
            rate.sleep()


    def utm_callback(self, utm_msg):
        self.is_utm=True
        self.odom_msg.pose.pose.position.x = utm_msg.data[0]
        self.odom_msg.pose.pose.position.y = utm_msg.data[1]
        self.odom_msg.pose.pose.position.z = 0


    def imu_callback(self, imu_msg):
        self.is_imu=True
        self.odom_msg.pose.pose.orientation.x = imu_msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = imu_msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = imu_msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = imu_msg.orientation.w

        

if __name__ == '__main__':
    try:
        Odometry_publisher()
    except rospy.ROSInterruptException:
        pass
