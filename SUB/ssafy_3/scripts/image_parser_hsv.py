#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser hsv Node - bgr 이미지가 아닌 hsv 형식으로 변환된 이미지를 출력

# 노드 실행 순서 
# 1. bgr 이미지를 hsv로 변환 
# 2. 서로다른 이미지 배열 합
# 3. 이미지 출력

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            # 카메라 데이터 받아서 데이터 변환 - Byte 단위 이미지 -> np.array
            # np.fromstring 함수를 이용하여 데이터를 uint8 형태로 변환
            np_arr = np.frombuffer(msg.data, dtype='uint8')

            # 1차원 배열 형태 np_arr를 3차원 배열의 bgr 형태로 변환
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        # cv2.cvtColor() - bgr 이미지 -> HSV 이미지 
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # np.concatenate() - 서로 다른 두 배열 합치는 함수
        img_concat = np.concatenate((img_bgr, img_hsv), axis=1)

        # 출력
        # cv2.imshow("Ego-0 Live Cam", img_bgr)
        cv2.imshow("Ego-0 HSV Cam", img_concat)
        cv2.waitKey(1)      # 단위: ms


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 