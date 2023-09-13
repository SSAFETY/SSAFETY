#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv


# lidar_ex_calib_velodyne 은 MORAI SIM에서 송신하는 LiDAR PointCloud2 Data를 Camera Image Data에 정합하는 예제입니다.
# 정합을 위해서는 LiDAR와 Camera 간의 위치 자세 관계성을 나타내는 Transformation Matrix(Extrinsic)와 Image Plane에 대한 정보인 Camera Matrix(Intrinsic)가 필요합니다.
# Transformation Matrix, Camera Matrix는 Camera, LiDAR 센서의 설치 위치 및 스펙을 이용해 계산합니다.
# 계산된 Matrix 들을 활용하여 LiDAR PointCloud2 Data를 Image Data에 정합한 결과를 시각화 합니다.
# 정합 시에는 불필요한 Point들을 제거해야 하며 각 Sensor Data의 좌표계가 어떤 형태인지 명심해야 합니다. 
# LiDAR는 VLP-16을 사용합니다.

#노드 실행 순서
# 1. Camera, LiDAR의 설치 좌표, Camera Parameter 입력
# 2. Extrinsic : Transformation Matrix(LiDAR to Camera Frame) 계산
#   2.1 Sensor to Vehicle Tranformation Matrix 계산 함수 구현
#       2.1.1 Rotation Matrix 계산 함수 구현
#       2.1.2 Transformation Matrix 계산 함수 구현
#   2.2 LiDAR to Camera Transformation Matrix 계산       
# 3. Intrinsic : Camera Matrix (Camera to Image Plane) 계산
# 4. LiDAR의 PointCloud2, Camera의 Image data 수신
# 6. 수신된 PointCloud2 data를 2D Image Plane으로 정합
# 5. PointCloud가 Image에 투영된 Processed Image 시각화

#TODO: (1) Camera, LiDAR의 설치 좌표, Camera Parameter 입력
'''
# 시뮬레이터에서 설치한 Camera와 LiDAR의 정보를 입력하는 영역입니다.
# 차량 뒷축 중심을 기준으로 설치된 센서들의 위치를 입력해줍니다. (X,Y,Z,Roll,Pitch,Yaw)
# Roll, Pitch, Yaw 입력 시에는 Radian 값으로 변환하여 입력해주어야 합니다.
# Camera는 추가로 Width, Height, Horizontal FOV 값을 입력합니다.

parameters_cam = {
    "WIDTH": , # image width
    "HEIGHT": , # image height
    "FOV": , # Field of view
    "X": , # meter
    "Y": ,
    "Z": ,
    "YAW": , # radian    
    "PITCH": ,
    "ROLL": 
}

parameters_lidar = {
    "X": , # meter
    "Y": ,
    "Z": ,
    "YAW": , # radian
    "PITCH": ,
    "ROLL": 
}
'''

def getRotMat(RPY):        
    #TODO: (2.1.1) Rotation Matrix 계산 함수 구현
    '''    
    # Rotation Matrix를 계산하는 영역입니다.
    # 각 회전에 대한 Rotation Matrix를 계산하면 됩니다.
    # Input
        # RPY : sensor orientation w.r.t vehicle. (Roll, Pitch, Yaw)
    # Oputput
        # rotMat : 3x3 Rotation Matrix of sensor w.r.t vehicle.
    # Tip : math, numpy
    
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])
    
    rotRoll = 
    rotPitch = 
    rotYaw = 

    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))    
    return rotMat
    '''

def getSensorToVehicleMat(sensorRPY, sensorPosition):
    #TODO: (2.1.2) Transformation Matrix 계산 함수 구현
    '''
    # Sensor To Vehicle Transformation Matrix를 계산하는 영역입니다.
    # 위에서 구현한 getRotMat 함수를 이용해 Rotation Matrix를 구하고
    # sensorPosition 정보를 활용하여 Translation Matrix를 구해
    # 최종적으로 Transformation Matrix를 생성하면 됩니다.
    # Input
        # sensorRPY : sensor orientation w.r.t vehicle. (Roll, Pitch, Yaw)
        # sensorPosition : sensor position w.r.t vehicle. (X, Y, Z)
    # Output
        # Tr_sensor_to_vehicle : 4x4 Transformation Matrix of sensor w.r.t vehicle.
    # Tip : numpy

    sensorRotationMat = 
    sensorTranslationMat = 
    Tr_sensor_to_vehicle = 
    
    return Tr_sensor_to_vehicle
    '''
    
def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):
    #TODO: (2.2) LiDAR to Camera Transformation Matrix 계산
    '''
    # LiDAR to Camera Transformation Matrix를 계산하는 영역입니다.
    # 아래 순서대로 계산하면 됩니다.
        # 1. LiDAR to Vehicle Transformation Matrix 계산        
        # 2. Camera to Vehicle Transformation Matrix 계산
        # 3. Vehicle to Camera Transformation Matrix 계산
        # 3. LiDAR to Camera Transformation Matrix 계산
    # Input
        # camRPY : Orientation
        # camPosition
        # lidarRPY
        # lidarPosition
    # Output
        # Tr_lidar_to_cam
    # Tip : getSensorToVehicleMat, inv 사용 필요

    Tr_lidar_to_vehicle = 
    Tr_cam_to_vehicle =     
    Tr_vehicle_to_cam = 
    Tr_lidar_to_cam = 
    
    print(Tr_lidar_to_cam)
    return Tr_lidar_to_cam
    '''

def getTransformMat(params_cam, params_lidar):
    #With Respect to Vehicle ISO Coordinate    
    lidarPositionOffset = np.array([0, 0, -0.25]) # VLP16 사용해야 함
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset  

    camPosition = np.array([params_cam.get(i) for i in (["X","Y","Z"])]) + camPositionOffset    
    camRPY = np.array([params_cam.get(i) for i in (["ROLL","PITCH","YAW"])]) + np.array([-90*math.pi/180,0,-90*math.pi/180])
    lidarPosition = np.array([params_lidar.get(i) for i in (["X","Y","Z"])]) + lidarPositionOffset
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL","PITCH","YAW"])])    
    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    #TODO: (3) Intrinsic : Camera Matrix (Camera to Image Plane) 계산
    '''
    # Camera의 Intrinsic parameter로 이루어진 Camera Matrix를 계산하는 영역입니다.
    # Camera의 width, height, fov 값을 활용하여 focal length, principal point를 계산한 뒤,
    # 이를 조합하여 Camera Matrix를 생성합니다.
    # Camera Model은 Lens 왜곡이 없는 Pinhole Model임을 참고하시기 바랍니다.
    # Input
        # params_cam : camera parameters
    # Output
        # CameraMat : 3x3 Intrinsic Matrix(a.k.a. Camera Matrix)    

    focalLength = 
    principalX = 
    principalY = 
    CameraMat = 
    
    print(CameraMat)
    return CameraMat
    '''

# Variables
    # scan_sub : ROS Subscriber of lidar data
    # image_sub : ROS Subscriber of image data
    # pc_np : pointcloud data of current frame 
    # img : iamge data of current frame 
    # width : image width
    # height : image height
    # TransformMat : lidar to camera Transform Matrix
    # CameraMat : Intrinsic Matrix(Camera Matrix)
class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat = getCameraMat(params_cam)

    #TODO : (4) LiDAR의 PointCloud2, Camera의 Image data 수신
    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)

    #TODO : (5.1) LiDAR Pointcloud to Camera Frame
    # Input
        # pc_lidar : pointcloud data w.r.t. lidar frame
    # Output
        # pc_wrt_cam : pointcloud data w.r.t. camera frame
    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    #TODO : (5.2) Camera Frame PointCloud to Image Plane with Filtering
    # Input
        # pc_camera : pointcloud data w.r.t. camera frame
    # Output
        # pc_proj_to_img : projection lidar data to image plane
    # Tip : for clear data use filtering
    def transformCameraToImage(self, pc_camera):
        pc_proj_to_img = self.CameraMat.dot(pc_camera)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[2,:]<0),axis=1)
        pc_proj_to_img /= pc_proj_to_img[2,:]
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[0,:]>self.width),axis=1)
        pc_proj_to_img = np.delete(pc_proj_to_img,np.where(pc_proj_to_img[1,:]>self.height),axis=1)
        return pc_proj_to_img

def draw_pts_img(img, xi, yi):
    point_np = img

    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (0,255,0),-1)
    return point_np

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    time.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        xyz_p = Transformer.pc_np[:, 0:3]
        xyz_p = np.insert(xyz_p,3,1,axis=1).T
        xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]<0),axis=1)
        #xyz_p = np.delete(xyz_p,np.where(xyz_p[0,:]>10),axis=1)
        #xyz_p = np.delete(xyz_p,np.where(xyz_p[2,:]<-1.2),axis=1) #Ground Filter
        #print(xyz_p[0])

        xyz_c = Transformer.transformLiDARToCamera(xyz_p)
        #print(np.size(xyz_c[0]))

        xy_i = Transformer.transformCameraToImage(xyz_c)
        #print(np.size(xy_i[0]))

        #TODO: (6) PointCloud가 Image에 투영된 Processed Image 시각화
        xy_i = xy_i.astype(np.int32)
        projectionImage = draw_pts_img(Transformer.img, xy_i[0,:], xy_i[1,:])        
        cv2.imshow("LidartoCameraProjection", projectionImage)
        cv2.waitKey(1)