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
from coords_srv.srv import Coordenadas, CoordenadasResponse 
from coords_srv.srv import *




def envio_coord(req):
    print("Returning [%s + %s = %s]"%(req.x, req.y, [req.x, req.y]))
    return CoordenadasResponse([req.x, req.y])
  

def suma_coords():
    rospy.init_node('aruco_coords_server')
    serv = rospy.Service('coordenadas_x_y', Coordenadas, envio_coord)
    print('ready to add two coords')
    rospy.spin()

if __name__ == '__main__':
    try:
      suma_coords()

    except rospy.ROSInterruptException:
        print("Deteniendo")
        exit()
    
