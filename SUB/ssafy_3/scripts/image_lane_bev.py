#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# image_lane_bev - perspective view 형태의 이미지를 BEV(bird-eye-view) 형태로 전환
# IPM (Inverse Perspective Mapping) - need to Warping; 이미지 왜곡시키는 작업 필수, 곡률 계산 위해 사용

# 노드 실행 순서 
# 1. 이미지 warping 영역 지정
# 2. 원본 이미지 내 Warping 영역과 매칭 될 목적지 지점 정의
# 3. 원근 맵 행렬 생성 함수를 이용하여 매핑 좌표에 대한 원근 맵 행렬을 생성
# 4. 원근 맵 행렬에 대한 기하학적 변환

def warp_image(img, source_prop):
    image_size = (img.shape[1], img.shape[0])

    x = image_size[0]
    y = image_size[1]
    # print('image_size(x, y): ',(x, y))

    #TODO: (2) 원본 이미지 내 Warping 영역과 매칭될 목적지 지점 정의 - 전체네 ?
    destination_points = np.array([
        [0, y],             # 좌측 하단
        [0, 0],             # 좌측 상단
        [x, 0],             # 우측 상단
        [x, y]              # 우측 하단
    ], dtype=np.float32)
    
    source_points = source_prop * np.float32([[x, y]]* 4)

    # 이미지를 기하학적 변환(확대, 축소, 위치 변경, 회전, 왜곡) - 원근을 없애는 작업 과정
    # 기하학적 변환은 아핀변환과(Affine transformation) 원근변환(Perspective transformation)이 있습니다.
    # 우리는 원근 변환이라는 OpenCV를 사용; 원근 맵 행렬 생성 함수 이용 - 매핑 좌표에 대한 원근 맵 행렬을 생성

    #TODO: (3) 원근 맵 행렬 생성
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    # print(perspective_transform)

    #TODO: (4) 이후 원근 맵 행렬에 대한 기하학적 변환을 진행
    warped_img = cv2.warpPerspective(img, perspective_transform, (x, y), flags=cv2.INTER_LINEAR)

    return warped_img


class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        #TODO: (1) BEV 위한 이미지 warping 영역 지정 - 비율로
        # 
        self.source_prop = np.array([
            [0.01, 0.72],               # 좌측 하단
            [0.50-0.14, 0.51],          # 좌측 상단
            [0.50+0.14, 0.51],          # 우측 상단
            [1.00-0.01, 0.72]           # 우측 하단
        ], dtype=np.float32)


    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as err:
            print(err)

        #TODO: (2-0) 이미지 Warping 시작 - 함수 리턴
        img_warp = warp_image(img_bgr, self.source_prop)

        img_concat = np.concatenate([img_bgr, img_warp], axis=1)

        cv2.imshow("Ego-0 BEV Cam", img_concat)
        cv2.waitKey(1)


def main():

    rospy.init_node('lane_birdview', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()

if __name__ == '__main__':
    
    main()
