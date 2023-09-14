#!/usr/bin/env python3

#-- coding:utf-8 --

import rospy
import subprocess


def call_virtualenv():


    virtualenv_activate = "/anaconda3/envs/test/bin/activate"


    try:
        subprocess.check_call(["source", virtualenv_activate], shell=True)
        rospy.loginfo("Virtual environment activated successfully.")
    
        subprocess.check_call(["python", "/nia-82-134-main/service-example.py"])
        #subprocess.check_call(["deactivate"], shell=True)
        rospy.loginfo("Virtual environment deactivated successfully.")

    except subprocess.CalledProcessError:
        print("what")
        rospy.logerr("Error while activating or deactivating virtual environment.")

if __name__ == '__main__':
    rospy.init_node('virtualenv_example_node')

    try:
        call_virtualenv()
    except rospy.ROSInterruptException:
        pass


