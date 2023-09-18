#!/usr/bin/env python
# coding: utf-8

# In[1]:


import json
import os
import torch
import torchvision
import mmcv
import cv2
import re
import mmdet
import time
import subprocess
import requests

os.environ["CUDA_VISIBLE_DEVICES"] = "3"

import PIL.ImageDraw as ImageDraw
import PIL.Image as Image
import numpy as np
import matplotlib.pyplot as plt
import torch.nn as nn

from collections import OrderedDict
from PIL import ImageFont
from glob import glob
from tqdm import notebook
from mmdet.apis import init_detector, inference_detector
from torchvision import transforms, models

from constants import ImageInfo, ObjectInfo, VEHICLE_LIST, \
    LANE_LABEL_MAP_PREV, LANE_COLOR_MAP_MODEL, VEHICLE_COLOR_MAP, NEW_SIZE, \
    VIOLATION_MAP, VLT_COLOR, DANGER_COLOR, NORMAL_COLOR
from lane_detection.model import LaneSegModel
from utils import viz_inference_result

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')


# In[2]:


print(f"pytorch => {torch.__version__}")
print(f"torchvision => {torchvision.__version__}")
print(f"mmdet => {mmdet.__version__}")
print(f"mmcv => {mmcv.__version__}")
print(f"opencv2 => {cv2.__version__}")


# # Useful Functions

# In[3]:


SCORE_TH = 0.5

def inference(img_path, vehicle_model, lane_model, device):
    if torch.cuda.is_available():
        device = torch.device("cuda:0")  # GPU 0을 사용하도록 선택합니다.
    else:
        device = torch.device("cpu")
        
    image_info = ImageInfo(img_path, None)
    obj_id = 0

    # >>> inference cars
    #input_img = Image.open(img_path)
    input_img = Image.fromarray(img_path);
    
    '''
    permute(2, 0, 1)을 사용하여 텐서의 차원 순서를 변경합니다. 
    OpenCV에서 읽어온 이미지의 채널 순서는 (높이, 너비, 채널)이므로 PyTorch에서 사용하는 (채널, 높이, 너비) 순서로 변경합니다.
    '''
    img_tensor = transforms.functional.to_tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(device)
    
    vehicle_model.to(device)
    
    bbox_result, segm_result = inference_detector(vehicle_model, img_path)

    #print(bbox_result)
    #print(segm_result)


    bboxes = np.vstack(bbox_result)
    labels = [
        np.full(bbox.shape[0], i, dtype=np.int32)
        for i, bbox in enumerate(bbox_result)
    ]

    #print(labels)

    labels = np.concatenate(labels)
    if isinstance(segm_result, tuple):
        segm_result = segm_result[0]
    segms = None

    #print(segm_result)


    if segm_result is not None and len(labels) > 0:  # non empty
        segms = mmcv.concat_list(segm_result)
        if isinstance(segms[0], torch.Tensor):
            segms = torch.stack(segms, dim=0).detach().cpu().numpy()
        else:
            segms = np.stack(segms, axis=0)

    object_list = []
    for i in range(len(labels)):
        bbox = bboxes[i][:4]
        score = bboxes[i][-1]
        if score < SCORE_TH:
            continue
        category = VEHICLE_LIST[labels[i]]
        segm = segms[i]
        # remove small size objects
        if category == "vehicle_bike" and bbox[3] - bbox[1] < 100:
            continue
        elif category != "vehicle_bike" and np.sum(segm) < 10000:
            continue
        # cv2.CHAIN_APPROX_TC89_L1, cv2.CHAIN_APPROX_TC89_KCOS, cv2.CHAIN_APPROX_SIMPLE
        c, _ = cv2.findContours(segm.astype(np.uint8), cv2.RETR_LIST, 
                                cv2.CHAIN_APPROX_SIMPLE)
        approx_poly = cv2.approxPolyDP(
            c[0], 0.005 * cv2.arcLength(c[0], True), True).astype(float)
        points = approx_poly.squeeze(1).reshape(-1)
        object_list.append(ObjectInfo(obj_id=obj_id,
                                      obj_type="vehicle", 
                                      bbox=bbox,
                                      poly=points,
                                      score=score,
                                      category=category,
                                      segm=segm))
        obj_id += 1


    image_info.objects = object_list
    result_img = vehicle_model.show_result(img_path, (bbox_result, segm_result))
    #print(result_img)

    # for debug

    # <<< inference cars end.
    
    # >>> inference lanes
    #input_img = Image.open(img_path)
    #w, h = input_img.size
    w, h = input_img.size

    img_tensor = transforms.functional.to_tensor(
            transforms.functional.resized_crop(
                input_img, h - w // 2, 0, w // 2, w, (800, 1333)
            )
        )
    

    img_tensor = img_tensor.to(device)
    
    lane_model.to(device)
    
    lane_out = torch.sigmoid(lane_model(img_tensor.unsqueeze(0))['out'])
    final_out = torch.argmax(lane_out[0],0).view(1,1,800,1333).to(torch.float)
    final_out = torch.nn.functional.interpolate(final_out, (1080,1920)).squeeze()
    lane_mask = np.array(final_out.to("cpu"))
    lane_mask = lane_mask.astype(np.uint8)
    lane_mask = Image.fromarray(lane_mask, mode="L")
    
    # post processing
    kernel = np.ones((5, 5), np.uint8)
    lane_mask = cv2.dilate(np.array(lane_mask), kernel, iterations=3)

    for label in np.unique(lane_mask):
        if label == 0:
            continue
        segm = np.where(np.array(lane_mask) == label, True, False)
        if np.sum(segm) < 1500:
            continue


        category = LANE_LABEL_MAP_PREV[label]
        # cv2.CHAIN_APPROX_TC89_L1, cv2.CHAIN_APPROX_TC89_KCOS, cv2.CHAIN_APPROX_SIMPLE
        c, _ = cv2.findContours(segm.astype(np.uint8), cv2.RETR_LIST, 
                                cv2.CHAIN_APPROX_SIMPLE)
        # TODO: deal with more than one contour
        bbox = cv2.boundingRect(c[0])
        bbox = (bbox[0], bbox[1], bbox[0] + bbox[2], bbox[1] + bbox[3])
        approx_poly = cv2.approxPolyDP(c[0], 0.005 * cv2.arcLength(c[0], True), 
                                       True)
        points = approx_poly.squeeze(1).reshape(-1).astype(float)
        obj_info = ObjectInfo(obj_id, "lane", category, 
                              np.array(bbox), points, segm, 1.)
        obj_id += 1
        image_info.objects.append(obj_info)
    # <<< end.

    return image_info


def detect_violation(object_info, violation_model, device):
    def _get_arr_mask(pil_img, new_size):
        result_arr = np.array(pil_img.copy().resize(new_size, Image.NEAREST))
        result_arr = np.mean(result_arr, axis=2)
        result_arr = np.where(result_arr > 0, True, False)
        
        return result_arr
    
    lane_list = []
    car_list = []
    result_list = []
    object_info = sorted(object_info, key=lambda k: np.sum(k.segm))
    for obj in object_info:
        if obj.obj_type == "vehicle":
            car_list.append(obj)
        else:
            lane_list.append(obj)
            result_list.append(obj)
    
    img_size = (1920, 1080)
    
    car_arr_list = []
    for car in car_list:
        # get array intersection is filled
        car_ctg = car.category
        result_img = Image.new("RGB", img_size)
        result_img_arr = np.array(result_img)
        result_img_arr[car.segm, :] = VEHICLE_COLOR_MAP[car_ctg]
        car_arr_mask = _get_arr_mask(Image.fromarray(result_img_arr), NEW_SIZE)
        intersections = []
        match = []
        for lane in lane_list:
            lane_ctg = lane.category
            result_img_arr[lane.segm] = LANE_COLOR_MAP_MODEL[lane.category]
            result_img = Image.fromarray(result_img_arr)
            blank_img = Image.new("RGB", img_size)
            blank_img_arr = np.array(blank_img)
            blank_img_arr[lane.segm] = LANE_COLOR_MAP_MODEL[lane.category]
            lane_arr_mask = _get_arr_mask(Image.fromarray(blank_img_arr), 
                                         NEW_SIZE)
            if np.sum(car_arr_mask & lane_arr_mask) > 10:
                match = [car.obj_id, lane.obj_id]
            
            if lane_ctg in VIOLATION_MAP[car_ctg]:
                color = VLT_COLOR
            elif lane_ctg in VIOLATION_MAP["danger"]:
                color = DANGER_COLOR
            else:
                color = NORMAL_COLOR
            intersections.append((color, 
                                  car_arr_mask & lane_arr_mask, 
                                  match))
        result_img_arr = np.array(result_img.resize(NEW_SIZE, Image.NEAREST))
        for inter in intersections:
            result_img_arr[inter[1], :] = inter[0]
        result_img_arr = result_img_arr.astype(np.float32)
        result_img_arr /= 255.
        car_arr_list.append((result_img_arr, match))
        #plt.imshow(result_img_arr)
        #plt.show()
    
    # Inference model
    if len(car_arr_list) == 0:
        return result_list
    car_arr_list = np.array(car_arr_list, dtype=object)
    input_image = np.stack(
        car_arr_list[:, 0]).transpose(0, 3, 1, 2)
    input_image = torch.from_numpy(input_image).type(torch.float32)
    data_transforms = torch.nn.Sequential(
            transforms.Resize(224),
            transforms.Normalize(
                        [0.0181, 0.0304, 0.0147], [0.1199, 0.1648, 0.1065]
                    ),
        )
    img_tensor = data_transforms(input_image)
    img_tensor = img_tensor.to(device)
    outputs = violation_model(img_tensor)
    _, preds = torch.max(outputs, 1)
    preds = np.array(preds.to("cpu"))
    
    if 2 in preds:
        car_idx = np.where(preds == 2)[0][0]
        vlt_car = car_list[car_idx]
        vlt_car.label = "violation"
        match = car_arr_list[car_idx][-1]
        if len(match) > 0: 
            lane_obj_id = match[-1]
            new_list = [vlt_car]
            for lane in result_list:
                if lane.obj_id == lane_obj_id:
                    new_list.append(lane)
            result_list = new_list
        else:
            result_list.append(vlt_car)
    elif 0 in preds:
        car_idx = np.where(preds == 0)[0][0]
        vlt_car = car_list[car_idx]
        vlt_car.label = "danger"
        match = car_arr_list[car_idx][-1]
        if len(match) > 0: 
            lane_obj_id = match[-1]
            new_list = [vlt_car]
            for lane in result_list:
                if lane.obj_id == lane_obj_id:
                    new_list.append(lane)
            result_list = new_list
        else:
            result_list.append(vlt_car)
    else:
        for car in car_list[:3]:
            result_list.append(car)
    
    return result_list


# # Config 파일 지정 및 학습된 모델 경로 지정

# In[4]:

path = '/home/ssafy/nia-82-134-main'


vehicle_cfg_path = f'{path}/configs/vehicle_detection_config.py'
vehicle_ckpt_path = f"{path}/best_models/vehicle_detection_model.pth"

lane_ckpt_path = f"{path}/best_models/lane_all.pth"

violation_ckpt_path = f"{path}/best_models/vlt_cls_model.pth"


# # 모델들 로드

# In[5]:


vehicle_model = init_detector(vehicle_cfg_path, vehicle_ckpt_path, device=device)

lane_model = LaneSegModel(num_classes=13)
lane_model_ckpt = torch.load(lane_ckpt_path, map_location=device)
new_state_dict = OrderedDict()
for n, v in lane_model_ckpt["state_dict"].items():
    new_name = n.replace("module.", "")
    new_state_dict[new_name] = v
lane_model.load_state_dict(new_state_dict)
lane_model.to(device)
lane_model.eval()

violation_model = models.resnet18(pretrained=False)
num_ftrs = violation_model.fc.in_features
violation_model.fc = nn.Linear(num_ftrs, 3)
violation_model_ckpt = torch.load(violation_ckpt_path, map_location=device)
violation_model.load_state_dict(violation_model_ckpt["state_dict"])
violation_model.to(device)
violation_model.eval()

# In[6]:


image_list = glob(f"{path}/sample_images/BLUE/*.jpg")

print(f"The number of images => {len(image_list)}")


# # 예시 이미지

# In[7]:

img_path = image_list[2]
print(img_path)
curr_img = Image.open(img_path)
result_img = None

# 새로운 FPS 값을 설정
new_fps = 10

# 웹캠 열기
webcam = cv2.VideoCapture(0)


if not webcam.isOpened():
    print("Could not open webcam")
    exit()

webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
webcam.set(cv2.CAP_PROP_FPS, new_fps)

frame_interval = 30
frame_count = 0

while webcam.isOpened():
    status, frame = webcam.read()

    if not status:
        break

    if frame_count % frame_interval == 0:
        resized_frame = cv2.resize(frame, (1920, 1080))

        image_info = inference(resized_frame, vehicle_model, lane_model, device)

        result_list = detect_violation(image_info.objects, violation_model, device)
        if not result_list:
            print("empty")
        else:
            labels = [obj.label for obj in result_list]
            print(labels)
            if "violation" in labels:
                data = {"fileName": "/home/ssafy01/Downloads/2023-09-18-172106.mp4"}
                headers = {'Content-Type': 'application/json'}
                #response = requests.post("http://localhost:8080/api/hi", data=json.dumps(data), headers=headers)
        image_info.objects = result_list
        result_img = viz_inference_result(resized_frame, image_info)        

    cv2.imshow("test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    frame_count += 1

webcam.release()
cv2.destroyAllWindows()
