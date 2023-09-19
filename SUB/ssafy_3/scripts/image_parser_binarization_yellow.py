#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser binarization Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# 출력시 hsv 특정 영역의 색상 범위를 지정하여 원하는 색상의 영역만 특정하여 출력합니다.

# 노드 실행 순서 
# 1. HSV 색상 영역 지정
# 2. 특정 영역의 색상 검출
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
        except CvBridgeError as e:
            print(e)
        '''
        img_hsv = cv2.cvtColor(                 )

        '''
        #TODO: (1)
        '''
        # 특정 색상 영역을 검출하기 위해 범위를 지정합니다.
        # 하한 값 행렬과 상한 값 행렬을 정해 그 사이의 값 만을 출력 하도록 합니다.
        # 이번 예제에서는 노란색 영역을 검출합니다.
        
        lower_ylane = np.array([    ,       ,       ])
        upper_ylane = np.array([    ,       ,       ])

        '''
        
        #TODO: (2)
        '''
        # cv2.inRange 함수는 특정 색상 영역을 추출할 수 있습니다. 
        # cv2.inRange 함수를 이용하여 HSV 이미지에서 색상 범위를 지정합니다.
        # 함수의 첫번째 변수에는 이미지 정보를 두번째는 하한 값 세번째는 상한 값 행렬식을 넣습니다.

        img_ylane = cv2.inRange(                    )

        img_ylane = cv2.cvtColor(                   )
        
        img_concat = np.concatenate(                )


        '''

        #TODO: (3)
        '''
        # 이미지를 출력 합니다.

        cv2.imshow(         )
        cv2.waitKey(        ) 

        '''


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 