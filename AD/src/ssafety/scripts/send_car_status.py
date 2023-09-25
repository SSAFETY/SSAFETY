#!/usr/bin/env python

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

import rospy
import rospkg
import time

from morai_msgs.msg import EgoVehicleStatus

import os

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from math import pi

# Firebase db 연동
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('sub1')
cred = credentials.Certificate(pkg_path + '/key/' + 'db_key.json')
app = firebase_admin.initialize_app(cred)
db = firestore.client()
doc_ref = db.collection(u'car').document(u'car_status')
seongam_doc_ref = db.collection('path').document('seongam')

class SendCarInfo:
    def __init__(self):
        rospy.init_node("send_car_status_info", anonymous=True)

        # 차량 위치 데이터
        self.now_gps_x = 0
        self.now_gps_y = 0
        self.map_gps_x = 0
        self.map_gps_y = 0
		
        self.now_imu_x = 0
        self.now_imu_y = 0
        self.now_imu_z = 0
        self.now_imu_w = 0

        # 차량 상태 데이터
        self.now_vel_x = 0
        self.now_vel_y = 0
        self.now_vel_z = 0
        
        self.now_tm = rospy.Time.from_sec(time.time())
        self.now_stm = self.now_tm.to_sec()

        # 센서 데이터 수신 확인
        self.get_imu=False
        self.get_gps=False
        self.get_ego_status = False

        # utm 좌표계 변환
        self.proj_utm = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        # 차량 위치,자세 데이터 저장
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'


        # 차량 센서 정보 구독
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.Ego_status_callback = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_callback)
        
        # 차량 정보 발행
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        self.path_pub = rospy.Publisher('/patrol_pub', String, queue_size=1)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.get_gps == True and self.get_imu == True and self.get_ego_status == True:
                self.convertLL2UTM()

                # 차량 정보 publish
                self.odom_pub.publish(self.odom_msg)
                seongam_doc = seongam_doc_ref.get()
				
                if seongam_doc.exists:
                    s_path = 'seongam'
                    seongam_p_dict = seongam_doc.to_dict()
                    s_path = s_path + '/t' + seongam_p_dict['local_path']

                    self.path_pub.publish(s_path)
				
                self.now_tm = rospy.Time.from_sec(time.time())
                self.now_stm = self.now_tm.to_sec()

                # 차량 데이터 Firebase Cloud Store 전송 
                doc_ref.set({
                    u'gps_x': f'{self.now_gps_x :.4f}',
                    u'gps_y': f'{self.now_gps_y :.4f}',
                    u'vel_x': f'{self.now_vel_x :.4f}',
                    u'vel_y': f'{self.now_vel_y}',
                    u'vel_z': f'{self.now_vel_z}',
                    u'now_time': f'{self.now_stm :8.0f}'
                })
				
                rospy.loginfo(seongam_p_dict['local_path'])
                os.system('clear')
            else:
                rospy.loginfo("cannot receive ego data")
                
            rate.sleep()

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset
		
        self.now_gps_x = gps_msg.longitude
        self.now_gps_y = gps_msg.latitude

        self.get_gps=True

    def imu_callback(self, data):

        # imu 데이터 저장
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
        
        self.get_imu=True    

    def convertLL2UTM(self):
        # 현재 한국의 utm 좌표계를 사용한다.
        # 해당 좌표계를 사용해 2d값의 위치로 정사영한다.
        xy_zone = self.proj_utm(self.lon, self.lat)
        
        # GPS 센서에서 출력되는 Offset 값은 시뮬레이터에 맵 좌표계로 변경을 위한 값이다.
        # UTM 좌표로 변환 된 x, y 값에 offset 값을 빼주면 된다.
        if self.lon == 0 and self.lat == 0:
            self.map_gps_x = 0.0
            self.map_gps_y = 0.0
        else:
            self.map_gps_x = xy_zone[0] - self.e_o
            self.map_gps_y = xy_zone[1] - self.n_o

        # 차량 위치정보를 odom_msg에 저장
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.map_gps_x
        self.odom_msg.pose.pose.position.y = self.map_gps_y
        self.odom_msg.pose.pose.position.z = 0

    # 차량 상태 데이터 입력
    def Ego_callback(self, Ego_data):
        self.get_ego_status = True

        self.now_vel_x = Ego_data.velocity.x
        self.now_vel_y = Ego_data.velocity.y
        self.now_vel_z = Ego_data.velocity.z

if __name__ == "__main__":
    try:
        SendCarInfo()
    except rospy.ROSInterruptException:
        pass
