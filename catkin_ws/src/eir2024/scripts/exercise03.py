#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 03 - DETECTOR DE BORDES DE CANNY
"""
import math
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_trackbar_lower(val):
    global lower_threshold
    lower_threshold = val

def callback_trackbar_upper(val):
    global upper_threshold
    upper_threshold = val

def callback_rgb_image(msg):
    global img, edges
    global lower_threshold, upper_threshold
    bridge = CvBridge()
    img  = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #
    # Agregue un filtro Gaussiano para la imagen en escala de grises y
    # aplique el detector de bordes de Canny, usando los respectivos
    # umbrales 'lower_threshold' y 'upper_threshold'
    #
    

def main():
    global img, edges
    global lower_threshold, upper_threshold
    print("INITIALIZING EXERCISE 03...")
    rospy.init_node("exercise03")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)
    rate = rospy.Rate(10)

    lower_threshold = 10
    upper_threshold = 50
    img   = numpy.zeros((480,640,3), numpy.uint8)
    edges = numpy.zeros((480,640,3), numpy.uint8)
    cv2.namedWindow("Canny")
    cv2.createTrackbar("Lower", "Canny", lower_threshold, 255, callback_trackbar_lower)
    cv2.createTrackbar("Upper", "Canny", upper_threshold, 255, callback_trackbar_upper)
    while not rospy.is_shutdown() and cv2.waitKey(10) != 'q':
        cv2.imshow("BGR", img)
        cv2.imshow("Canny", edges)
        rate.sleep()
    

if __name__ == "__main__":
    main()
    
    

