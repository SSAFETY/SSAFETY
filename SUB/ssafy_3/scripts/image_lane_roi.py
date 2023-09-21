#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_roi - 이미지의 관심있는 부분(차선)만 남기고 나머지 부분 마스킹하는 이미지 처리
# 관심 영역 지정, 마스크 생성, 마스크를 이미지에 합치는 과정
# 이거 그냥 단순히 이미지 잘라주는 거고 이미지 처리는 다른 부분에서 하는 거네!

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        
        # image_size
        x = 640
        y = 480

        #TODO: (1) 관심있는 영역만 지정 - 4개의 포인트 - 이거 조정 필요
        self.crop_pts = np.array([[
            [5, 365],           # 좌측 하단
            [233, 265],         # 좌측 상단
            [395, 265],         # 우측 상단
            [650, 365],         # 우측 하단
        ]])


    def callback(self, msg):
        # uint8 : unsined integer 0~255 로 만들기 위함입니다.
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as err:
            print(err)

        self.mask = self.mask_roi(img_bgr)
        # 이미지를 가로로
        # 이 때 들어오는 이미지에 따라 Color(RGB)와 grayscale 를 구분한다.
        if len(self.mask.shape) == 3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Ego-0 RoI Cam", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):

        # img.shape 는 [h, w, Color] 3차원이고, Color 는 RGB 를 갖는 3채널이다.
        h = img.shape[0]
        w = img.shape[1]
        
        # 3channel color image일 경우,
        if len(img.shape) == 3:
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)

        # grayscale image일 경우.
        # (참고 - 시뮬레이터에서 주는 이미지는 항상 3차원이기 때문에 예외를 처리를 위해서만 )
        else:
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = (255)
        
        # TODO 관심 영역: 원본 이미지 / 나머지 부분: 검은색(255)로 반환
        #마스킹 영역을 만들기 위해서 다양한 방법을 사용할 수 있습니다만, 코드에서 이미 까만 이미지를 생성했습니다.
        #이를 이용하는 방법을 찾아야 합니다.
        
        #TODO: (2) cv2 이용하여 polygon 그려주기
        cv2.fillPoly(mask, self.crop_pts, mask_value)

        #TODO : (3) RGB 이미지를 마스킹하는 함수 이용
        mask = cv2.bitwise_and(mask, img)
        
        return mask


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
