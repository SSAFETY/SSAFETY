# SSAFETY
### 교통 법규 위반 감지 자율 주행 버스 프로젝트
<img src="https://github.com/leehk77789/BEP/assets/96775737/9ef93474-9b36-4b83-807c-14198a10aee6">

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
2023.08.28 ~ 2023.10.06
<br><br><br><br>

## 팀 소개

### 팀 아우토반
- 이충혁(AD):
- 김민재(AD):
- 김현명(AD): Ubuntu 20.04.3, Morai Simulator, Firebase CloudStore
- 정유준(WEB): Spring, React, Mysql, D3
- 김용균(AI/INFRA):
<br><br><br><br>

## 기술스택
<img src="https://img.shields.io/badge/java-007396?style=for-the-badge&logo=java&logoColor=white">
<img src="https://img.shields.io/badge/springboot-6DB33F?style=for-the-badge&logo=springboot&logoColor=white">
<img src="https://img.shields.io/badge/springdatajpa-6DB33F?style=for-the-badge&logo=springboot&logoColor=white">
<img src="https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white">

<br>
<img src="https://img.shields.io/badge/javascript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black">
<img src="https://img.shields.io/badge/react-61DAFB?style=for-the-badge&logo=react&logoColor=black">
<img src="https://img.shields.io/badge/html-E34F26?style=for-the-badge&logo=html5&logoColor=white">
<img src="https://img.shields.io/badge/css-1572B6?style=for-the-badge&logo=css3&logoColor=white">
<br>
<br>
<img src="https://img.shields.io/badge/docker-2496ED?style=for-the-badge&logo=docker&logoColor=white">
<img src="https://img.shields.io/badge/jenkins-D24939?style=for-the-badge&logo=jenkins&logoColor=white">
<img src="https://img.shields.io/badge/nginx-009639?style=for-the-badge&logo=nginx&logoColor=white">
<img src="https://img.shields.io/badge/gitlab-FC6D26?style=for-the-badge&logo=gitlab&logoColor=white">
<br><br><br><br>

## 개발환경

수정해야 됨
> BackEnd
Java : 11.0.19 <br>
JVM : 18.9 <br>
SERVER : AWS EC2 Ubuntu 20.04.3 LTS <br>

> Front
VS Code : 1.80.2 <br>
Node.js : 18.17.1 <br>

> DB
MySQL : 8.1 <br>
StmCubeIDE : 1.13.1 <br>
ArduinoIDE : 2.1.1 <br>

- AD
Python : 3.8.10
Simulation os : Ubuntu 20.04.6 LTS
ROS : noetic, 1.16.0
NVDIA Driver : 470.199.02   
CUDA Version: 11.4 

- AI
cudatoolkit : 11.0
cudnn : 8
colorlog : 6.6.0
easydict : 1.9
future : 0.18.2
ipywidgets : 7.6.5
jupyter : 1.0.0
matplotlib : 3.5.0
mmcv-full : 1.3.9
mmdet : 2.18.1
omegaconf : 2.1.1
pandas : 1.3.4
scikit-learn : 1.0.1
tensorboard : 2.7.0
torch : 1.7.0+cu110
torchaudio : 0.7.0
torchvision : 0.8.1+cu110
tqdm : 4.62.3

## 인프라
<br><br><br><br>

## ERD
<br><br><br><br>

## 배포 주소
https://j9a102.p.ssafy.io
<br><br><br><br>

## 주요 기능 소개

### 0. 초기 설정

#### 사장님은 키오스크에서 최초 실행 시에 해당 테이블이 몇 번 테이블인지 매칭해주어야 합니다.
<br>
![image-8](/uploads/5e4492703141fc7ae86fc779f331a20c/image-8.png)
<br><br>

### 1. Web

#### 1-1. 메인 페이지
<img src="https://github.com/leehk77789/BEP/assets/96775737/080e1c1a-8c79-4401-96b0-4b9e580dab94">
<br><br>

#### 1-2. 교통 위반 통계
<img src="https://github.com/leehk77789/BEP/assets/96775737/f59abff2-d4a8-4420-8e32-bd3f349c6e4a">
<br><br>

#### 1-3. 교통 위반 데이터베이스
<img src="https://github.com/leehk77789/BEP/assets/96775737/5bc4a67e-6a1f-4b03-9680-7f69d3fd94c0">
<br><br>

#### 1-4. 교통 위반사항 상세보기
<img src="https://github.com/leehk77789/BEP/assets/96775737/fb3396fe-c387-49c3-94f4-524798a76164">
<br><br>

#### 1-5. 자율주행 버스 관제, 통제
<img src="https://github.com/leehk77789/BEP/assets/96775737/c875cefb-cc5c-4f52-8367-d7f7d33a2e65">
<br><br>

