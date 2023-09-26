#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser Node 는 시뮬레이터에서 송신하는 Camera 센서 정보를 받아 실시간으로 출력하는 예제입니다.
# Camera 센서 정보인 /image_jpeg/compressed 라는 메세지를 Subscribe 합니다.
# Subscribe 한 데이터를 OpenCV 를 이용하여 Image 로 출력합니다.

# 노드 실행 순서 
# 1. 문자열 Type 데이터를 정수형으로 변환 
# 2. 읽을 수 있는 bgr 이미지로 변환 
# 3. 이미지 출력

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            #TODO: (1)
            '''
            # Open CV 를 이용하여 Camera 데이터를 시각화 하기 위해서는 np.array 형태가 필요하다. 
            # Byte 단위로 저장 된 이미지 데이터를 np.array 형태로 읽는 부분이다.
            # np.fromstring 함수를 이용하여 읽어오는 data 정보를 uint8 형태로 변환한다.

            np_arr = np.fromstring(         )

            '''
            #TODO: (2)
            '''
            # 1차원 배열 형태로 되어있는 np_arr 변수를 3차원 배열로 만든뒤 컬러 이미지로 변환합니다.
            # cv2.imdecode 함수를 이용하여 np_arr 변수를 3차원 배열로 만듭니다.
            # cv2.IMREAD_COLOR 함수를 통해 이미지 파일을 bgr 형태의 Color로 읽어들입니다.

            img_bgr = cv2.imdecode(         )

            '''
        except CvBridgeError as e:
            print(e)

        #TODO: (3)
        '''
        # 이미지를 출력 합니다.
        # cv2.imshow 함수를 이용하여 이미지를 실시간으로 출력합니다.
        # cv2.waitKey 함수를 이용하여 이미지 업데이트 시간을 지연 시킬 수 있습니다.
        # cv2.waitKey 함수의 단위는 ms 입니다.

        cv2.imshow(             )
        cv2.waitKey(            ) 

        '''


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 