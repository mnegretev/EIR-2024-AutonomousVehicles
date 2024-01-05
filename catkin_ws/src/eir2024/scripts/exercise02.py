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

    

