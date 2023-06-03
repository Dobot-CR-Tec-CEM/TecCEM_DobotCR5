#!/usr/bin/env python

import rospy
from dobot_bringup.srv import *

def disable_client():
    rospy.wait_for_service('/dobot_bringup/srv/DisableRobot')
    
    try:
        disable = rospy.ServiceProxy('/dobot_bringup/srv/DisableRobot', DisableRobot)
        resp = disable()
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def enable_client():
    rospy.wait_for_service('/dobot_bringup/srv/EnableRobot')
    
    try:
        enable = rospy.ServiceProxy('/dobot_bringup/srv/EnableRobot', EnableRobot)
        resp = enable()
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def go_to_point(x, y, z, rx, ry, rz):
    rospy.wait_for_service('/dobot_bringup/srv/ServoP')
    
    try:
        move = rospy.ServiceProxy('/dobot_bringup/srv/ServoP', ServoP)
        resp = move(x, y, z, rx, ry, rz)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def speedFactor():
    rospy.wait_for_service("/dobot_bringup/srv/SpeedFactor")
    
    try:
        speed = rospy.ServiceProxy("/dobot_bringup/srv/SpeedFactor", SpeedFactor)
        resp = speed(10)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def clearError():
    rospy.wait_for_service("/dobot_bringup/srv/ClearError")
    
    try:
        clear = rospy.ServiceProxy("/dobot_bringup/srv/ClearError", ClearError)
        resp = clear()
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def moveJoints(j1, j2, j3, j4, j5, j6):
    rospy.wait_for_service("/dobot_bringup/srv/JointMovJ")
    
    try:
        move = rospy.ServiceProxy("/dobot_bringup/srv/JointMovJ", JointMovJ)
        resp = move(j1, j2, j3, j4, j5, j6)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)

def sync():
    rospy.wait_for_service("/dobot_bringup/srv/Sync")
    
    try:
        sync = rospy.ServiceProxy("/dobot_bringup/srv/Sync", Sync)
        resp = sync()
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)




if __name__ == "__main__":
    #sync()
    #clearError()
    #enable_client()
    speedFactor()
    print(moveJoints(90, 0, 0, 0,0,0))
    #print(moveJoints(160, 50, 30, 10, -90, -90))
    #print("Ya se movio")
    #print(go_to_point(500, 500, 500, -80, 0, -100))
    
    #

    