#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image parser binarization Node - HSV 영역의 특정 색상 범위만 출력합니다.

# 노드 실행 순서 
# 1. HSV 색상 영역 지정
# 2. 특정 영역의 색상 검출
# 3. 비트 연산을 통핸 두 이미지의 합
# 4. 이미지 출력

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

        # white, yellow 색상 영역 검출하기 위해 범위를 지정
        lower_wlane = np.array([0,0,205])
        upper_wlane = np.array([30,60,255])

        lower_ylane = np.array([0,70,120])# ([0,60,100])
        upper_ylane = np.array([40,195,230])# ([40,175,255])

        # cv2.inRange("이미지 정보", 하한 값, 상한 값) - 특정 색상 영역을 추출, 범위 지정
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        img_ylane = cv2.cvtColor(img_ylane, cv2.COLOR_GRAY2BGR)


        # 비트 연산을 통해 두 이미지 합치기
        img_lane = cv2.bitwise_or(img_wlane, img_ylane)
        # np.concatenate() - 서로 다른 두 배열 합치는 함수
        img_concat = np.concatenate((img_bgr, img_lane), axis=1)


        # 출력
        cv2.imshow("Ego-0 Bin Cam", img_concat)
        cv2.waitKey(1)      # 단위: ms


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 