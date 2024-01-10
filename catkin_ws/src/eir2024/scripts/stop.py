#!/usr/bin/env python3
"""
This node implements a proportional control and is intended to be used
together with the lane_detector node. It is assumed both lane borders 
are given by two straight lines in rho-theta form. Given a desired rho-theta
for each lane border, an error is calculated.
Steering is calculated proportional to this error and linear speed is
set as a constant. 
"""
import rospy
from std_msgs.msg import Float64, Empty, Bool
   
def callback_stop_motion(msg):
    global stop_motion
    stop_motion = msg.data    

def main():
    global stop_motion
    global curr_time
    
    curr_time = 0.0
        
    print('INITIALIZING STOP NODE...', flush=True)
    rospy.init_node('stop')
    rate = rospy.Rate(10)

    rospy.Subscriber("/stop", Bool, callback_stop_motion) 
    
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=1)

    stop_motion = False
    while not rospy.is_shutdown():
        if stop_motion:
            speed = 0.0  
        else:
            continue
        
        pub_speed.publish(speed)

        rate.sleep()
    

if __name__ == "__main__":
    main()

    

