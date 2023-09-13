#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser hsv Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# 출력시 bgr 이미지가 아닌 hsv 형식으로 변환 된 이미지를 출력 합니다.

# 노드 실행 순서 
# 1. bgr 이미지를 hsv로 변환 
# 2. 서로다른 이미지 배열 합
# 3. 이미지 출력

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            '''

            np_arr = np.fromstring(             )
            img_bgr = cv2.imdecode(             )

            '''
            np_car_im = np.fromstring(msg.data, np.uint8)
            car_im_bgr = cv2.imdecode(np_car_im, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        #TODO: (1)
        '''
        # cv2.cvtColor 함수를 이용하여 bgr 이미지를 hsv 이미지로 변환합니다.
        # cv2.cvtColor 함수는 색상 공간 변환 함수로 이미지를 원하는 색상 코드로 변환합니다.
        # 이번 예제에서는 bgr이미지를 HSV 이미지로 변환합니다.

        img_hsv = cv2.cvtColor(                 )

        '''
        
        car_im_hsv = cv2.cvtColor(car_im_bgr, cv2.COLOR_BGR2HSV)
        #TODO: (2)
        '''
        # np.concatenate 함수는 는 서로 다른 두 배열을 합치는 예제입니다.
        # bgr 이미지와 hsv 이미지를 동시에 출력하기 위해 np.concatenate 함수를 사용해 이미지를 합칩니다.

        img_concat = np.concatenate(            )

        '''
        # horizontal 
        car_im_concat = np.concatenate((car_im_bgr,car_im_hsv), axis=1)
        #TODO: (3)
        '''
        # 이미지를 출력 합니다.

        cv2.imshow(         )
        cv2.waitKey(        ) 

        '''
        cv2.imshow("Image window",car_im_concat)
        cv2.waitKey(10)


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 