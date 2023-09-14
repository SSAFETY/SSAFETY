#!/usr/bin/env python3

#-- coding:utf-8 --

import subprocess

# Conda 환경 이름 설정
conda_env_name = "test"

# Conda 환경 활성화 명령어 설정
activate_cmd = f"conda activate {conda_env_name}"

print("?")

# Conda 환경을 활성화하고 명령어 실행
result = subprocess.run(["bash", "-c", activate_cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

print("!")

# 결과 확인
if result.returncode == 0:
    print(f"Conda 환경 {conda_env_name}이(가) 활성화되었습니다.")
else:
    print("Conda 환경 활성화 중 오류가 발생했습니다.")
    print("표준 출력:", result.stdout.decode())

subprocess.run(["python", "/nia-82-134-main/service-example.py"])

