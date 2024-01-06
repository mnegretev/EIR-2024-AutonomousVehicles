#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 04 - TRANSFORMADA HOUGH
"""
import math
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_trackbar_threshold(val):
    global hough_threshold
    hough_threshold = val

def callback_trackbar_length(val):
    global min_length
    min_length = val

def callback_trackbar_gap(val):
    global max_gap
    max_gap = val

def draw_lines(lines, img):
    if lines is None:
        return
    lines = lines[:,0]
    for x1,y1,x2,y2 in lines:
        cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 3, cv2.LINE_AA)

def callback_rgb_image(msg):
    global img, hough_threshold, min_length, max_gap
    bridge = CvBridge()
    img  = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #
    # Aplique un filtro Gaussiano a la imagen en escala de grises
    # Aplique el detector de bordes de Canny a la imagen filtrada
    # Detecte lineas en los bordes utilizando la Transformada Hough
    # Dibuje las lineas encontradas llamando a la funcion 'draw_lines'
    #
    
    
def main():
    global img, hough_threshold, min_length, max_gap
    print("INITIALIZING EXERCISE 04...")
    rospy.init_node("exercise04")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)
    rate = rospy.Rate(10)

    
    img = numpy.zeros((480,640,3), numpy.uint8)
    hough_threshold = 80
    min_length = 80
    max_gap = 100
    cv2.namedWindow("Hough")
    cv2.createTrackbar("Thr:", "Hough", hough_threshold, 255, callback_trackbar_threshold)
    cv2.createTrackbar("Len:", "Hough", min_length     , 400, callback_trackbar_length   )
    cv2.createTrackbar("Gap:", "Hough", max_gap        , 200, callback_trackbar_gap      )
    while not rospy.is_shutdown() and cv2.waitKey(10) != 'q':
        cv2.imshow("Hough", img)
        rate.sleep()
    

if __name__ == "__main__":
    main()
    
    

