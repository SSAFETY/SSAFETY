#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

# camera_pedes_detector: 카메라로 보행자 인식하는 Object-detection 
# HoG feature 추출 후 사전 학습된 SVM 알고리즘을 통해 보행자 영역이 분류됨
# 여기서 적절한 영역 추출 - 이론 배경 노션 정리 참고(or 명세서)

# 노드 실행 순서 
# 1. 이미지 subscriber 선언
# 2. HOG descitpor 생성
# 3. 기학습된 SVM 분류기 설정 (for peds)
# 4. 이미지 불러오기
# 5. 검출 (paramter 바꿔가면서 설정)
# 6. NMS 후처리 통과
# 7. NMS 내부 (IOU 계산)
# 8. 추출된 좌표 Detection 용 박스 화

def non_maximum_supression(bboxes, threshold=0.3):
    bboxes = sorted(bboxes, key=lambda detections: detections[3], reverse=True)

    new_bboxes = []
    new_bboxes.append(bboxes[0])
    
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]

            #TODO: (7)  IOU 계산
            # 이미지의 IoU를 계산하는 영역입니다. 위에 입력된 파라미터를 이용하여 IoU를 계산.
            x_overlap = max(0, min(x1_br, x2_br) - max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br) - max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap

            area_1 = bbox[2] * bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2

            overlap_area = overlap_area / (total_area - overlap_area)

            if overlap_area < threshold:
                new_bboxes.append(bbox)

    return new_bboxes


class PEDESDetector:
    def __init__(self):

        #TODO: (1) subscriber 선언
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.rate = rospy.Rate(20)

        #TODO: (2) HOG descitpor 생성
        self.pedes_detector = cv2.HOGDescriptor()

        #TODO: (3) 사전 학습된 SVM 분류기 설정 (for peds)
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def callback(self, msg):

        self.rate.sleep()

        try:
            #TODO: (4)  이미지 불러오기
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as err:
            print(err)

        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        #TODO: (5)  검출 (paramter 바꿔가면서 설정)
        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_gray, winStride=(4, 4), padding=(2, 2), scale=32)
        
        if len(rects_temp) != 0:            
            #TODO: (6)  NMS 후처리 통과
            rects = non_maximum_supression(rects_temp)

            for (x,y,w,h) in rects:
                #TODO: (8)  추출된 좌표로 박스 생성
                cv2.rectangle(img_bgr, (x, y), (x+w, y+h), (0,255,255), 2)

        cv2.imshow("Pedes detection Cam", img_bgr)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('pedes_detector', anonymous=True)

    pedes_detector = PEDESDetector()

    rospy.spin() 
