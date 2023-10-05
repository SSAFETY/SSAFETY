# SSAFETY

교통 법규 위반 신고 서비스
Team 아우토반

## 개요

1. 프로젝트 개요
2. 프로젝트 사용 도구
3. 개발환경
4. 외부 서비스

## 빌드

1. 환경변수 형태
2. 빌드하기
3. 배포하기
4. 서비스 이용방법
5. 기타


## 개요

- 프로젝트 개요
  현재 신고 시스템은 개개인이 블랙박스에서 교통 법규 위반 차량 영상을 찾아 업로드 하는 방식으로 번거로운 점이 있습니다.
- SSAFETY는  AI기반의 영상분석을 통해 교통 법규 위반 차량 발견시 영상 저장 및 신고 웹사이트와 연동해 신고 시스템을 개선합니다.


## 프로젝트 사용 도구

이슈 관리 : JIRA
형상 관리 : Gitlab
커뮤니케이션 : Notion, Mattermost
UCC : Microsoft Clipchamp
CI/CD : Jenkins
Simulator : MORAI SIM 22.R2.1

## 개발환경

VS Code : 1.83.0
Spring Boot : 3.1.3
Gradle : 8.2.1
JVM : 18.9
Java : 11.0.19
Node.js : 18.17.1
SERVER : AWS EC2 Ubuntu 20.04.3 LTS
MySQL : 8.1 

#### AD
Python : 3.8.10
Simulation os : Ubuntu 20.04.6 LTS
ROS : noetic, 1.16.0
NVDIA Driver : 470.199.02   
CUDA Version: 11.4 

#### AI
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

## 빌드

### 환경변수 형태

.application.yml

```
driver-class-name: com.mysql.cj.jdbc.Driver
url: url
username: username
password: password
```

## 빌드하기

1. Front  
    터미널 상 `~/front` 내에서  
   `npm i`  
   `npm run build`
2. Back-spring
   Gradle 실행
   Bootjar 실행
3. Back-flask

## 배포하기

Nginx 설정

```
server {
        listen                          443 ssl http2;
        server_name                     i9a201.p.ssafy.io;
        charset                         utf-8;

        ssl_certificate                 /etc/ssl/ssiosk/nginx_ssl.crt;
        ssl_certificate_key             /etc/ssl/ssiosk/private.key;

        ssl_session_timeout             5m;
        ssl_prefer_server_ciphers       on;

        add_header                      'Access-Control-Allow-Origin' '*';

        location / {
                proxy_pass http://localhost:3000;
                proxy_read_timeout 604800s;
                proxy_send_timeout 604800s;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection 'upgrade';
                proxy_set_header Host $host;
                proxy_set_header Origin "";
                proxy_cache_bypass $http_upgrade;
        }

        location /api {
                proxy_pass http://localhost:5500;
                proxy_read_timeout 604800s;
                proxy_send_timeout 604800s;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection 'upgrade';
                proxy_set_header Host $host;
                proxy_set_header Origin "";
                proxy_cache_bypass $http_upgrade;

        }

        location /ws {
                proxy_pass http://localhost:5500;
                proxy_read_timeout 604800s;
                proxy_send_timeout 604800s;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection 'upgrade';
                proxy_set_header Host $host;
                proxy_set_header Origin "";
                proxy_cache_bypass $http_upgrade;
        }

       location /device {
                proxy_pass http://127.0.0.1:5000;
                proxy_set_header Host $host;
                proxy_set_header X-Forwarded-Server $host;
                proxy_set_header X-Real-IP $remote_addr;
                proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
                proxy_set_header X-Forwarded-Proto $scheme;
       }
}

server {
        listen                          80 default;
        server_name                     i9a201.p.ssafy.io;

        location / {
                return 301 https://i9a201.p.ssafy.io$request_uri;
                expires epoch;
        }
}
```

이후 sudo service nginx start


## 서비스 이용 방법

#### AD
```
roslaunch rosbridg_server rosbridge_websocket.launch
roslaunch ssafety traffic_light.launch
```

#### AI
```
conda activate [venv name]
python3 lane_violation_detect.py
```

## 기타

#### 외부 서비스 사용 방법

Amazone S3 Storage : 교통 법규 위반 영상 저장용
Google Firebase : 차량 데이터 웹과 송수신용


**Amazone S3**

1. Aws S3 접속 후 버킷 만들기
2. AWS console > IAM > 엑세스 관리 > 사용자 > 사용자 추가
3. AWS Console > IAM > 엑세스 관리자 > 사용자 > 생성한 사용자 이름 클릭 > 보안 자격 증명 > 엑세스 키 만들기
4. 액세스 키와 비밀 액세스 키를 application.yml 파일에 입력

**Google Firebase**

1. Firebase Console에서 프로젝트 추가
2. Cloud Firestore 섹션으로 이동, 워크플로 따르기
3. Cloud Firestore 보안 규칙의 시작 모드를 선택
4. 데이터베이스의 위치를 선택 (기본 Google Cloud Platform(GCP))
