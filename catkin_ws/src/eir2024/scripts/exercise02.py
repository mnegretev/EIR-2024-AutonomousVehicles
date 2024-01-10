#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 02 - GRADIENTE DE UNA IMAGEN
"""
import math
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_rgb_image(msg):
    bridge = CvBridge()
    img  = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #
    # Agregue un filtro Gaussiano para disminuir el ruido
    #
    
    #
    # Aplique un filtro de Sobel y calcule la magnitud del gradiente con la norma 1
    #
    grad_x = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3)
    abs_grad_x = cv2.convertScaleAbs(grad_x)
    abs_grad_y = cv2.convertScaleAbs(grad_y)
    grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
    cv2.imshow("Gradient Magnitude", grad)
    cv2.imshow("Image BGR", img)
    cv2.waitKey(10)

def main():
    print("INITIALIZING EXERCISE 02...")
    rospy.init_node("exercise02")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)
    rate = rospy.Rate(10)
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

