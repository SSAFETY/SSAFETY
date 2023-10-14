# SSAFETY
### 교통 법규 위반 감지 자율 주행 버스 프로젝트

![싸피로고1](/uploads/93d86f044df33b845f2ccfb888b4ec07/싸피로고1.png)       

<br><br><br><br>

## 프로젝트 개요
교통 법규 위반 신고는 3년간 3배 증가하였으나 기존 신고 방식은 복잡하고 다소 번거롭다는 문제가 있습니다.
<br>
SSAFETY는 교통 법규 위반 감지 자율 주행 버스로, 위반 감지 시 자동으로 영상을 저장하고 관제 사이트로 신고하여 신고 방식을 개선하였습니다.
<br><br><br><br>

## 프로젝트 소개
- 자율 주행 버스가 AI 기반 영상 분석을 통해 교통 법규 위반 차량을 감지합니다.
- 위반 영상을 관련 정보와 함께 관제 시스템에 자동으로 등록합니다.
- 등록된 신고 데이터는 날짜, 지역, 위반 종류 등으로 필터링하여 확인할 수 있습니다.
- 통계 파트에서는 지역별, 시간별, 종류별 위반 통계를 확인할 수 있습니다.
- 자율 주행 버스의 현재 위치를 확인할 수 있으며 경로를 지정할 수 있습니다.
<br><br><br><br>

## 개발 기간
2023.08.21 ~ 2023.10.06(7주)
<br><br><br><br>

## 팀 소개

### 팀 아우토반
- 이충혁(팀장/AD/Planning): Lidar 클러스터링, global path planning, 상암 hd맵 반영
- 김민재(AD/Perception & Control): Pure Pursuit, PID제어, ACC(Adaptive Cruise Control), Lattice planning 등의 자율 주행 제어, 카메라 센서 처리
- 김현명(AD/Perception): morai simulator 환경에서 gps/imu/차량 정보 데이터 이용하여 pub-sub 등록, 차량 정보 데이터 firebase - cloud store간 송수신 연동
- 정유준(WEB): Spring Jpa specification을 이용한 조건별 검색 구현, React D3를 이용한 실제 지형과 같은 지도 구현(GPS 연동 가능), React와 Spring을 이용한 리스트 페이지네이션 구현, Firebase에서 자율주행과 연동된 좌표를 기반으로 실시간 웹에 Pin을 통한 현재 차량 위치 및 속도 동기화, Nivo를 이용한 통계 차트와그래프 구현
- 김용균(AI/INFRA): Resnet 기반 교통 위반 감지 AI 모델 학습 및 적용, CI/CD 인프라 구축 및 관리
<br><br><br><br>

## 기술스택
<img src="https://img.shields.io/badge/java-007396?style=for-the-badge&logo=java&logoColor=white">
<img src="https://img.shields.io/badge/springboot-6DB33F?style=for-the-badge&logo=springboot&logoColor=white">
<img src="https://img.shields.io/badge/springdatajpa-6DB33F?style=for-the-badge&logo=springboot&logoColor=white">
<img src="https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white">
<img src="https://img.shields.io/badge/Firebase-FFCA28?style=for-the-badge&logo=Firebase&logoColor=white">

<br>
<img src="https://img.shields.io/badge/javascript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black">
<img src="https://img.shields.io/badge/react-61DAFB?style=for-the-badge&logo=react&logoColor=black">
<img src="https://img.shields.io/badge/html-E34F26?style=for-the-badge&logo=html5&logoColor=white">
<img src="https://img.shields.io/badge/css-1572B6?style=for-the-badge&logo=css3&logoColor=white">
<br>
<img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white">
<img src="https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white">
<img src="https://img.shields.io/badge/pytorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white">
<br>
<br>
<img src="https://img.shields.io/badge/docker-2496ED?style=for-the-badge&logo=docker&logoColor=white">
<img src="https://img.shields.io/badge/jenkins-D24939?style=for-the-badge&logo=jenkins&logoColor=white">
<img src="https://img.shields.io/badge/nginx-009639?style=for-the-badge&logo=nginx&logoColor=white">
<img src="https://img.shields.io/badge/gitlab-FC6D26?style=for-the-badge&logo=gitlab&logoColor=white">
<br><br><br><br>

## 개발환경

- BackEnd       
  - Java : 11.0.19 <br>
  - JVM : 18.9 <br>
  - SERVER : AWS EC2 Ubuntu 20.04.3 LTS <br>

- Front     
  - VS Code : 1.80.2 <br>
  - Node.js : 18.17.1 <br>
  - D3 : 7.6.1 <br>
  - WebPack : 5.74.0 <br>

- DB        
  - MySQL : 8.1 <br>
  - StmCubeIDE : 1.13.1 <br>
  - Firebase Cloud Firestore <br>

- AD        
  - Simulator : MORAI SIM ver22.R2.1
  - Python : 3.8.10     
    - scikit-learn : 1.3.1
    - scipy : 1.10.1
  - Linux os : Ubuntu 20.04.6 LTS      
  - ROS : noetic        
  - NVDIA Driver : 470.199.02       
  - CUDA Version: 11.4      

- AI        
  - cudatoolkit : 11.0      
  - cudnn : 8       
  - mmcv-full : 1.3.9       
  - mmdet : 2.18.1      
  - pandas : 1.3.4      
  - scikit-learn : 1.0.1        
  - tensorboard : 2.7.0     
  - torch : 1.7.0+cu110     

## 배포 주소
https://j9a102.p.ssafy.io
<br><br><br><br>

## 웹 - 자율주행 환경간 데이터 송수신

### 프로토콜 규격 및 정보

`car` / `car1, car2, car3` / `gps_x, gps_y, vel_x, vel_y` <br>

`car`: 자율주행 정보 colleciton <br>
`car1, car2, car3`: 자율주행 버스 종류 documentation <br>
`gps_x, gps_y, vel_x, vel_y`: 자율주행 버스 정보(차량위치, 속도) field <br>

환경: Firebase Cloud firestore (NoSql)<br>

## 웹 서비스 화면


### 1. Web

#### 1-1. 메인 페이지
![temp_1697100464736.286979516](/uploads/c07241ebc917c30cd865c05986c72de8/temp_1697100464736.286979516.png)

<br><br>

<br><br>

#### 1-2. 교통 위반 데이터베이스

![temp_1697100464734.286979516](/uploads/c48dc5a97557d903a48bd3914ccc8e0f/temp_1697100464734.286979516.png)

<br><br>

#### 1-3. 교통 위반사항 상세보기
![temp_1697100464727.286979516](/uploads/b0ff5cb1e323c79335d6fc9db8d0633c/temp_1697100464727.286979516.png)

<br><br>

#### 1-4. 자율주행 버스 관제, 통제

![temp_1697100464730.286979516](/uploads/0520a7f29ced7165fcf2794fb4e444f3/temp_1697100464730.286979516.png)
<br><br>

### 2. Autonomous driving

#### 2-1. 아키텍처
![image](/uploads/c00d8ac06330985b676f5d35f5b3548d/image.png)

<br><br>

#### 2-2. 인지 - 카메라
![image](/uploads/c35eb81049c13ff2c0353da1206eda47/image.png)

<br><br>

#### 2-3. 인지 - 라이다
![image](/uploads/140293026e9c294b09e7c2608f7df824/image.png)

<br><br>

#### 2-4. 판단 - 교통 신호
![image](/uploads/a407b1ae258fffae09678ef78ed3078f/image.png)

<br><br>

#### 2-5. 판단 - 충돌 회피
![image](/uploads/c1da832148a054cbe8f6aa609acb5941/image.png)

<br><br>

#### 2-6. 제어 - Pure Pursuit, PID, ACC
![image](/uploads/f163a0af977eb9cb19bb498df9c2fc8e/image.png)

<br><br>
