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
        ENV_NAME = "test"

        # Conda 환경을 활성화
        #subprocess.run('conda activate test && python /nia-82-134-main/service-example.py', shell=True)
        #subprocess.run("source /anaconda3/etc/profile.d/conda.sh", shell=True)
        #subprocess.run(f"""conda init bash
        #conda activate {ENV_NAME}
        #python -c 'import pandas; print(pandas.__version__)'""", shell=True, executable='/bin/bash', check=True)
        anaconda_path = "/home/ssafy01/anaconda3"
        conda_script_path = f"{anaconda_path}/etc/profile.d/conda.sh"
        # Anaconda를 실행하는 셸 스크립트 작성
        conda_command = f"source {conda_script_path} && conda activate {ENV_NAME}"
        subprocess.run(conda_command, shell=True, executable='/bin/bash', check=True)
        python_executable = f"{anaconda_path}/envs/test/bin/python3"
        python_script_path = "/home/ssafy01/nia-82-134-main/service-example.py"
        subprocess.run(f"{python_executable} {python_script_path}", shell=True, executable='/bin/bash', check=True)
        rospy.loginfo(f"Conda 환경 {conda_env_name}이(가) 활성화되었습니다.")

        # 필요한 스크립트 실행
        #subprocess.check_call(["python", "/nia-82-134-main/service-example.py"])

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

