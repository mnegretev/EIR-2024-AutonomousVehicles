#!/usr/bin/env python3
"""
This node determines if the car is at the right or left lane of the road. 
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

def callback_curr_pose(msg):
    global pub_curr_lane, curr_lane, x, y, theta # , prev_curr_lane   

    x = msg.x
    y = msg.y
    z = msg.theta
 
    if y > 0:
       curr_lane =  False # False indicates the car is in the left lane
    else:
       curr_lane = True  # True indicates the car is in the right lane     

    #if prev_curr_lane != curr_lane: 
       #print("Position in y ", y, "curr_lane", curr_lane, "prev_curr_lane", prev_curr_lane,  flush=True)
       #prev_curr_lane = curr_lane

    pub_curr_lane.publish(curr_lane)


def main():
    global pub_curr_lane, curr_lane, x, y, theta
    
    x = 0.0
    y = 0.0
    theta = 0.0
    curr_lane = True
    
    print("INITIALIZING LANE NODE...", flush=True)
    rospy.init_node("lane")
    rate = rospy.Rate(10)

    rospy.Subscriber("/current_pose", Pose2D, callback_curr_pose)
    
    pub_curr_lane  = rospy.Publisher("/current_lane", Bool, queue_size=10)

    while not rospy.is_shutdown():
        rate.sleep()
   

if __name__ == "__main__":
    main()

'''
if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
'''    

