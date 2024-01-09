#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 06 - SEGUIMIENTO DE CARRILES
"""
import math
import cv2
import numpy
import rospy
from std_msgs.msg import Float64, Float64MultiArray

goal_rho_l   = 185.0
goal_theta_l = 2.4
goal_rho_r   = 215.0
goal_theta_r = 0.895
k_delta = 1.5

def calculate_control(rho_l, theta_l, rho_r, theta_r):
    global max_speed, k_rho, k_theta
    speed = 0
    steering = 0
    #
    # Implemente las leyes de control para calcular la velocidad lineal 'speed' y
    # el ángulo del volante 'delta' en función de los parámetros de las líneas
    # observadas.
    #
    
    return speed, steering

def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data

def main():
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r
    global max_speed, k_rho, k_theta
    print('INITIALIZING EXERCISE 06...')
    rospy.init_node('exercise06')
    max_speed = rospy.get_param('~max_speed', 10)
    k_rho = rospy.get_param('~k_rho', 0.002)
    k_theta = rospy.get_param('~k_theta', 0.02)
    lane_rho_l   = 0
    lane_theta_l = 0
    lane_rho_r   = 0
    lane_theta_r = 0    
    rate = rospy.Rate(30)
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=10)

    print("Waiting for lane detection...")
    msg_left_lane  = rospy.wait_for_message('/demo/left_lane' , Float64MultiArray, timeout=100)
    msg_right_lane = rospy.wait_for_message('/demo/right_lane', Float64MultiArray, timeout=100)
    print("Using:")
    print("Max speed: " + str(max_speed))
    print("K_rho: " + str(k_rho))
    print("K_theta: " + str(k_theta))
    while not rospy.is_shutdown():
        speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r)
        pub_speed.publish(speed)
        pub_angle.publish(steering)
        rate.sleep()
    

if __name__ == "__main__":
    main()
    
    

