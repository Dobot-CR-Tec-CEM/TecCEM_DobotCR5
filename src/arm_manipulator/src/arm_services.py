#!/usr/bin/env python

"""
Nodo para enviar instrucciones directas a los servicios
que se dan de alta al lanzar dobot_bringup.
"""


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


def moveL(x, y, z, rx, ry, rz):
    rospy.wait_for_service('/dobot_bringup/srv/MoveL')
    
    try:
        move = rospy.ServiceProxy('/dobot_bringup/srv/MoveL', MoveL)
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


def moveJ(x, y, z, rx, ry, rz):
    rospy.wait_for_service("/dobot_bringup/srv/MovJ")
    
    try:
        move = rospy.ServiceProxy("/dobot_bringup/srv/MovJ", MovJ)
        resp = move(x, y, z, rx, ry, rz)
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

def armOrientation():
    rospy.wait_for_service("/dobot_bringup/srv/SetArmOrientation")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/SetArmOrientation", SetArmOrientation)
        resp = arm(1, 1, 1, 1)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)


def toolDOEx():
    rospy.wait_for_service("/dobot_bringup/srv/ToolDOExecute")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/ToolDOExecute", ToolDOExecute)
        resp = arm(2, 0)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)

def dO(id, val):
    rospy.wait_for_service("/dobot_bringup/srv/DO")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/DO", DO)
        resp = arm(id, val)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)

def doExecute():
    rospy.wait_for_service("/dobot_bringup/srv/DOExecute")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/DOExecute", DOExecute)
        #resp = arm(1, 1)
        return arm
    except rospy.ServiceException as e:
        print("Service Failed:", e)

def payLoad():
    rospy.wait_for_service("/dobot_bringup/srv/PayLoad")
    
    try:
        arm = rospy.ServiceProxy("/dobot_bringup/srv/PayLoad", PayLoad)
        resp = arm(1, 20)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e) #RunScript

def runScript(name):
    rospy.wait_for_service("/dobot_bringup/srv/RunScript")
    
    try:
        f = rospy.ServiceProxy("/dobot_bringup/srv/RunScript", RunScript)
        resp = f(name)
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e) #RunScript  PowerOn

def power():
    rospy.wait_for_service("/dobot_bringup/srv/PowerOn")
    
    try:
        f = rospy.ServiceProxy("/dobot_bringup/srv/PowerOn", PowerOn)
        resp = f()
        return resp
    except rospy.ServiceException as e:
        print("Service Failed:", e)

if __name__ == "__main__":
    #clearError()
    #power()
    #enable_client()
    #speedFactor()
    #print(runScript("initGripper"))
    sync()
    for i in range(3):
        runScript("closeGripper")
        rospy.sleep(5)
        runScript("openGripper")
        rospy.sleep(5)
    #armOrientation()
    #toolDOEx()
    #payLoad()
    #doExecute()()
    #print(moveJ(-552, -470, 553, -178, -0.27, 119.28))
    #print(moveL(-552, -470, 253, -178, -0.27, 119.28))
    #print(moveL(-552, -470, 553, -178, -0.27, 119.28))
    #print("Ya se movio")
    #for i in range(1, 25):
    #    dO(i, 1)
    #    dO(i, 0)

    #print(moveJ(552, -470, 553, -178, -0.27, 119.28))
    #print(moveL(552, -470, 253, -178, -0.27, 119.28))
    #print(moveL(552, -470, 553, -178, -0.27, 119.28))

    
    #

    