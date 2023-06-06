#!/usr/bin/env python
import rospy 
import cv2 
from cv2 import aruco
import numpy as np    
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import Int32, Float64MultiArray
from geometry_msgs.msg import Pose2D
import tf
import sys
from coords_srv.srv import Coordenadas
from coords_srv.srv import *
medicion_ArUco = Pose2D()

def aruco_coords_client(x, y):
	rospy.wait_for_service('coordenadas_x_y')
	try:
		aruco_coords = rospy.ServiceProxy("coordenadas_x_y", Coordenadas)
		response = aruco_coords(x, y)
		return response.coord_x_y
	except rospy.ServiceException as e:
	    print ("Service call failed: %s"%e)
	    
def coords_callback(msg):
    global medicion_ArUco
    #print(type(msg))
    if msg is not None:
        medicion_ArUco.x = msg.x
        medicion_ArUco.y = msg.y
        medicion_ArUco.theta = msg.theta
  

if __name__ == "__main__":
	#if len(sys.argv) == 3:
	#	x = int(sys.argv[1])
	#	y = int(sys.argv[2])
	#else:
	#	print("%s [x y]"%sys.argv[0])
	#	sys.exit(1) 
	#print ("Requesting %s + %s"%(x, y))
	#print("%s + %s = %s"%(x, y, aruco_coords_client(x, y)))
	
    rospy.init_node('aruco_coords_client')


    pose = rospy.Subscriber("/pose_aruco", Pose2D, coords_callback)
 #   print('............................', type(medicion_ArUco.x))
    
    rospy.sleep(1)
    print ("Requesting %s, %s"%(medicion_ArUco.x, medicion_ArUco.y))
    aruco_coords_client(medicion_ArUco.x, medicion_ArUco.y)
    #   print("%s + %s = %s"%(medicion_ArUco.x, medicion_ArUco.y, aruco_coords_client(medicion_ArUco.x, medicion_ArUco.y)))
      

    #print ("Requesting %s + %s"%(medicion_ArUco.x, medicion_ArUco.y))