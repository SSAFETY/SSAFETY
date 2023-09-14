#!/usr/bin/env python3
#-- coding:utf-8 --

import rospy
import subprocess

def call_conda_environment():
    # Conda 환경 이름 설정
    conda_env_name = "test"

    try:
        # Conda 환경 활성화 명령어 설정
        #activate_cmd = f"conda activate {conda_env_name}"

        # Conda 환경을 활성화
        #subprocess.run('conda activate test && python /nia-82-134-main/service-example.py', shell=True)
        subprocess.call('conda activate test', shell=True)
        rospy.loginfo(f"Conda 환경 {conda_env_name}이(가) 활성화되었습니다.")

        # 필요한 스크립트 실행
        #subprocess.check_call(["python", "/nia-82-134-main/service-example.py"])
        #subprocess.run(['python', '/nia-82-134-main/service-example.py'], 
        #stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        
        # Conda 환경 비활성화
        #deactivate_cmd = "conda deactivate"
        #subprocess.check_call(["bash", "-c", deactivate_cmd], shell=True)
        #rospy.loginfo(f"Conda 환경 {conda_env_name}이(가) 비활성화되었습니다.")
    
    except subprocess.CalledProcessError as e:
        print("hello")
        rospy.logerr("Conda 환경을 관리하거나 스크립트를 실행하는 중에 오류가 발생했습니다.")
        rospy.logerr(str(e))

if __name__ == '__main__':
    rospy.init_node('conda_environment_example_node')

    try:
        call_conda_environment()
    except rospy.ROSInterruptException:
        pass

