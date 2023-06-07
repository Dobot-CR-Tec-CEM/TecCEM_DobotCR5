#!/usr/bin/env python
import rospy 
import cv2 
from cv2 import aruco
import numpy as np    
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import Int32 
from geometry_msgs.msg import Pose2D
import tf

bridge = CvBridge()
cv_image = None
dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
lista_arucos =[15,17,20]

parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 9
parameters.adaptiveThreshWinSizeStep = 1
parameters.errorCorrectionRate = 0.1
parameters.minMarkerDistanceRate = 30  
parameters.polygonalApproxAccuracyRate = 0.04
#parameters.minOtsuStdDev = 2
#parameters.minMarkerPerimeterRate = 0.035
#parameters.minCornerDistanceRate = 0.035
mtx = np.array([[1052.826663, 0.000000, 761.706500],
                [0.000000, 994.620836, 317.701036],
                [0.000000, 0.000000, 1.000000]])

dist = np.array([[0.115192, -0.095804, -0.035235, 0.031588, 0.000000]])

pose_aruco = Pose2D()

def norm_coord(idx_x, idx_y):
    #x e y estan con respecto a los ejes del robot
    #print(idx_x, idx_y)
    if idx_y < 111:
        #print()
        idx_y = -1 * (idx_y - 111)
        incremento_y = 0.19724770642201836
    else:
        idx_y = -1 * (idx_y - 111)
        incremento_y = 0.11375460122699387 

    if idx_x < 575:
        idx_x = -1 * (idx_x - 575)
        incremento_x =  -0.133913043478261
    else:
        idx_x = (idx_x - 560)

        incremento_x = 0.130357142857143
  
    coord_x = incremento_x * idx_x
    coord_y = incremento_y * idx_y
    return coord_x, coord_y


def image_callback(img):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='rgb8') #passthrough
    cv_image = cv2.resize(cv_image, (1280, 720))

    #cv2.circle(cv_image, (1230,40), 7, (0,255,0), -1)
    #cv2.circle(cv_image, (1230,640), 7, (0,255,0), -1)
    #cv2.circle(cv_image, (95,640), 7, (0,255,0), -1)
    #cv2.circle(cv_image, (567+95,300+40), 7, (0,255,0), -1)
    #cv2.circle(cv_image, (1020+95,310+40), 7, (0,255,0), -1) 


    cv_image = cv_image[40:640, 95:1230]
  

def aruco_identify(img):
    #gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    gray = img
    corners, maker_ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, maker_ids)
    #print(corners)
    x_sum = 0
    y_sum = 0
    x_centerPixel = 0
    y_centerPixel = 0
    x_norm, y_norm = 0, 0
    if corners != []:
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        
        x_centerPixel = x_sum*.25
        y_centerPixel = y_sum*.25

        x_norm, y_norm = norm_coord(x_centerPixel, y_centerPixel)
        dir = 1
        error_x = 4.5
        error_y = 1.3
        x_norm_e, y_norm_e = x_norm, y_norm
         
        if x_norm > 0:
            print('1')
            x_norm_e = x_norm - error_x
        elif x_norm < 0:
            x_norm_e = x_norm

        
        if y_norm < 0: 
            print(2)
            y_norm_e = y_norm + error_y
        elif y_norm > 0:
            print(3 )
            y_norm_e = y_norm - error_y


        print('cc', x_norm_e, y_norm_e)



        print('sc ', x_norm, y_norm)

        #print(x_centerPixel, y_centerPixel)

        #print(x_norm, y_norm)
        #print(frame_markers, corners, maker_ids, x_norm_e, y_norm_e)
        return frame_markers, corners, maker_ids, x_norm_e, y_norm_e
  

def main():
    marker_size = 0.33
    while not rospy.is_shutdown():
        rate.sleep()
        if cv_image is not None:
            try:
                frame_markers, maker_corners, maker_ids, x_coord, y_coord = aruco_identify(cv_image)
            except:
                continue
            if np.all(maker_ids != None) and maker_ids[0][0] in lista_arucos:
                marcador = maker_ids[0][0]
                rvec, tvec , _= aruco.estimatePoseSingleMarkers(maker_corners, marker_size, mtx, dist)
                #rvec, tvec,_ = cv2.solvePnP(marker_points, maker_corners, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
                #print(rvec)
                pose_aruco.x, pose_aruco.y, pose_aruco.theta = x_coord, y_coord, 0 #######

                for i in range(0, maker_ids.size):
                    frame_markers = aruco.drawAxis(frame_markers, mtx, dist, rvec[i], tvec[i], 0.3)
                    #cv2.drawFrameAxes(frame_markers, mtx, dist, rvec[i], tvec[i], 0.3)
                    output_image = bridge.cv2_to_imgmsg(frame_markers, encoding = 'rgb8') #passthrough
            else:
                marcador = -1 
                output_image = bridge.cv2_to_imgmsg(cv_image, encoding = 'rgb8')

            pose_aruco_pub.publish(pose_aruco)
            aruco_detected_pub.publish(marcador)
            output_image = bridge.cv2_to_imgmsg(frame_markers, encoding = 'rgb8') #passthrough	
            image_process_pub.publish(output_image)

if __name__ == '__main__':
    try:
        rospy.init_node("aruco_detection")
        rate = rospy.Rate(20)
        image_process_pub = rospy.Publisher("/usb_cam/image_raw/aruco_detected_img", Image, queue_size=1)
        image_raw_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
        aruco_detected_pub = rospy.Publisher("/aruco_detected", Int32, queue_size=1) 
        pose_aruco_pub = rospy.Publisher("/pose_aruco", Pose2D, queue_size=1)

        main()

    except rospy.ROSInterruptException:
        print("Deteniendo")
        exit()
    