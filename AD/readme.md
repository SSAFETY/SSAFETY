# AD(Autonomous Driving)  
: 자율주행 자동차는 사람과 같이 인지된 각종 교통상황 정보를 종합하고 상황을 고려해 빠른 판단을 내릴 수 있는 기능이 필요하다. 이를 **인지 - 판단 - 제어**의 3가지 과정으로 일반적으로 정의한다.  
: 자율주행 자동차는 차량에 장착된 각종 센서로부터 수집된 데이터를 종합하여 상황을 **‘인지’**하고, 인지된 상황에 근거하여 차량을 어떻게 제어하고 주행해야 할지 **‘판단’**하며, 이러한 주행제어 측면의 판단에 근거하여 차량을 **‘제어’**한다.

![아키텍처](https://github.com/GGamangCoder/GGamangCoder/assets/94775103/e174ce0e-d9c1-4a8c-b418-092610bfc609)  


## 인지(Perception)
- GPS, IMU : 주행기록계(odometry) 측정
- Lidar : DBMSCAN 으로 객체 인지


## 판단(Decision & Planning)
- 


## 제어(Control)
- Pure pursuit: 조향각
- PID 제어: 속도 제어
- 경로기반 속도 계획(velocity_planning): 직선-곡선 속도 계획
  - 최소자승법: 곡률 반지름(r) 계산
- ACC(Adaptive Cruise Control): 앞차와의 속도 유지  

- 현재 최종 코드: `velocity_planning.py`


#### 출처
- [자율주행 알고리즘](http://weekly.tta.or.kr/weekly/files/20215519055532_weekly.pdf)