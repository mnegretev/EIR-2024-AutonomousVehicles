#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 05 - DETECCIÓN DE CARRILES
"""
import math
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def detect_lines(img):
    lines = []
    #
    # Aplique un filtro Gaussiano a la imagen en escala de grises
    # Aplique el detector de bordes de Canny a la imagen filtrada
    # Utilice la transformada Hough para detectar líneas
    # Devuelva el conjunto de líneas detectadas como una matriz de Nx4 donde
    # N es el número de líneas detectadas y cada línea está dada por la
    # tupla (x1,y1,x2,y2)
    #
    
    return lines

def to_normal_form(x1, y1, x2, y2):
    rho = 0
    theta = 0
    #
    # Transforme la línea dada por los puntos (x1,y1) y  (x2,y2)
    # a la forma normal rho,theta
    #
   
    return numpy.asarray([rho, theta])

def translate_lines_to_bottom_center(lines, x_center, y_center):
    if lines is None:
        return None
    new_lines = []
    #
    # Cambie cada línea a coordenadas con respecto al centro inferior de la imagen
    # Recuerde que el centro original de la imagen está en la esquina superior
    # izquierda. El nuevo origen está dado por las variables x_center, y_center
    #
    
    return new_lines

def filter_lines(lines):
    left_lines  = []
    right_lines = []
    min_angle_right = 0.1
    max_angle_right = 1.3
    min_angle_left  = 1.9
    max_angle_left  = 2.8
    #
    # Transforme las líneas a la forma normal y elimine aquellas
    # cuyo ángulo esté fuera de los límites para poder ser el
    # borde de un carril.
    # Sintonice los ángulos correspondientes.
    # Devuelva el conjunto de líneas filtradas como dos matrices de Nx4 donde
    # N es el número de líneas detectadas y cada línea está dada por la
    # tupla (x1,y1,x2,y2). Devuelva un conjunto para las líneas izquierdas
    # y otra para las derechas. 
    #
    
    return left_lines, right_lines

def weighted_average(lines):
    if lines is None or len(lines) == 0:
        return 0, 0
    weighted_average_rho   = 0
    weighted_average_theta = 0
    #
    # Calcule un promedio ponderado de las líneas tomando una ponderación
    # proporcional a su longitud. Devuelva el promedio ponderado de los
    # parámetros en forma normal rho, theta. 
    #
    
    return weighted_average_rho, weighted_average_theta

def draw_normal_line(rho, theta, length, img,color):
    if rho == 0 or theta == 0:
        return
    a  = math.cos(theta)
    b  = math.sin(theta)
    x1 = int(a*rho - b*length + img.shape[1]/2)
    y1 = int(img.shape[0] - (b*rho + a*length))
    x2 = int(a*rho + b*length + img.shape[1]/2)
    y2 = int(img.shape[0] - (b*rho - a*length))
    cv2.line(img, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)

def callback_rgb_image(msg):
    global img, pub_left_lane, pub_right_lane
    bridge = CvBridge()
    img  = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    lines = detect_lines(gray)
    lines = translate_lines_to_bottom_center(lines, img.shape[1]/2, img.shape[0])
    left_lines, right_lines = filter_lines(lines)
    mean_rho_l, mean_theta_l = weighted_average(left_lines)
    mean_rho_r, mean_theta_r = weighted_average(right_lines)
    msg_left_lane  = Float64MultiArray()
    msg_right_lane = Float64MultiArray()
    msg_left_lane.data  = [mean_rho_l, mean_theta_l]
    msg_right_lane.data = [mean_rho_r, mean_theta_r]
    pub_left_lane.publish(msg_left_lane)
    pub_right_lane.publish(msg_right_lane)
    draw_normal_line(mean_rho_l, mean_theta_l, img.shape[0], img, (255,0,0))
    draw_normal_line(mean_rho_r, mean_theta_r, img.shape[0], img, (0,0,255))
    
def main():
    global img, pub_left_lane, pub_right_lane
    print("INITIALIZING EXERCISE 05...")
    rospy.init_node("exercise05")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)
    pub_left_lane  = rospy.Publisher("/demo/left_lane" , Float64MultiArray, queue_size=10)
    pub_right_lane = rospy.Publisher("/demo/right_lane", Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    
    img = numpy.zeros((480,640,3), numpy.uint8)
    while not rospy.is_shutdown() and cv2.waitKey(10) != 'q':
        cv2.imshow("Hough", img)
        rate.sleep()
    

if __name__ == "__main__":
    main()
    
    

