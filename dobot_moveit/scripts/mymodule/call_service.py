#!/usr/bin/env python3

import rospy
from dobot_v4_bringup.srv import ClearError, EnableRobot, DisableRobot

def call_ClearError():
    rospy.wait_for_service("/dobot_v4_bringup/srv/ClearError")
    try:
        service = rospy.ServiceProxy('/dobot_v4_bringup/srv/ClearError', ClearError)
        response = service()
    except rospy.ServiceException:
        print("Service call failed")

def call_EnableRobot():
    rospy.wait_for_service("/dobot_v4_bringup/srv/EnableRobot")
    try:
        service = rospy.ServiceProxy('/dobot_v4_bringup/srv/EnableRobot', EnableRobot)
        response = service()
    except rospy.ServiceException:
        print("Service call failed")

def call_DisableRobot():
    rospy.wait_for_service("/dobot_v4_bringup/srv/DisableRobot")
    try:
        service = rospy.ServiceProxy('/dobot_v4_bringup/srv/DisableRobot', DisableRobot)
        response = service()
    except rospy.ServiceException:
        print("Service call failed")
