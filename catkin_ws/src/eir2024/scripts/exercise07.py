#!/usr/bin/env python3
"""
ESCUELA DE INVIERNO DE ROBÓTICA 2024
EJERCICIO 07 - DETECCIÓN DE OBSTÁCULOS
"""

import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

lower_x  = 2.5
upper_x  = 25
lower_y  = -1.0
upper_y  = 1.0
min_points = 150

def pub_obstacle_position(p):
    global pub_obs_point
    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "car_link"
    msg.point.x = p[0]
    msg.point.y = p[1]
    msg.point.z = p[2]
    pub_obs_point.publish(msg)
    

def callback_point_cloud(msg):
    global pub_obstacle
    P = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    # 
    # Detecte obstáculos en la región 'norte' indicada contando el número de puntos
    # presentes en la región. Si dicho número supera un umbral, se detecta obstáculo
    # ----------------|---------|--------------|
    #    NORTE        | <-Auto  |              |
    # ----------------|---------|--------------|
    #   NOROESTE      | OESTE   |              |
    # ----------------|---------|--------------|
    
    pub_obstacle.publish(P.shape[0] > min_points)
    if P.shape[0] > min_points:
        pub_obstacle_position(numpy.mean(P, axis=0))

def main():
    global pub_obstacle,pub_obs_point
    print("INITIALIZING EXERCISE 07...")
    rospy.init_node("exercise07")
    rate = rospy.Rate(10)        
    rospy.Subscriber('/point_cloud', PointCloud2, callback_point_cloud)
    pub_obstacle  = rospy.Publisher("/obstacle/north"     , Bool, queue_size=1)
    pub_obs_point = rospy.Publisher("/obstacle/north_point", PointStamped, queue_size=1)
    while not rospy.is_shutdown():  
        rate.sleep()  

if __name__ == "__main__":
    main()
    
