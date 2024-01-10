#!/usr/bin/env python3
"""
This node is intended to be used to code the resulting policy
after modeling and training the MDPs using ProbLog.
This node provides several functions to check if there are
other vehicles around the car and to execute the three
different behaviors: steady motion, follow and pass. 
"""
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool, String
from rosgraph_msgs.msg import Clock 

def mysleep(secs):
    global curr_time

    init_time = curr_time        
    diff = 0.0
    while diff <= secs: # and not rospy.is_shutdown():
       diff  = curr_time - init_time


def callback_sim_time(msg):
    global curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs
    curr_time = sim_secs + sim_nsecs / (10**9) 

def callback_free_N(msg):
    global free_N
    free_N = msg.data

def callback_free_NW(msg):
    global free_NW
    free_NW = msg.data

def callback_free_W(msg):
    global free_W
    free_W = msg.data
    
def callback_free_SW(msg):
    global free_SW
    free_SW = msg.data
    
def callback_free_NE(msg):
    global free_NE
    free_NE = msg.data    

def callback_free_E(msg):
    global free_E
    free_E = msg.data    

def callback_free_SE(msg):
    global free_SE
    free_SE = msg.data    
    
def callback_success(msg):
    global success
    success = msg.data
    
def callback_curr_lane(msg):
    global curr_lane
    curr_lane = msg.data

def cruise():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane
    pub_keep_distance.publish(False)
    pub_stop.publish(False)
    pub_change_lane.publish(False)
    pub_cruise.publish(True)    

def keep_distance():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, curr_lane
    pub_cruise.publish(False)
    pub_stop.publish(False)
    pub_change_lane.publish(False)
    pub_keep_distance.publish(True)    

def change_lane():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, curr_lane
    global curr_time

    pub_keep_distance.publish(False)
    pub_stop.publish(False)
    pub_cruise.publish(False)    

    #if curr_lane:
    #   print("Inicia giro hacia carril izquierdo ", curr_time, flush = True)
    #else: 
    #   print("Inicia giro hacia carril derecho ", curr_time, flush = True)        

    pub_change_lane.publish(True)
    msg_finished = rospy.wait_for_message('/change_lane/finished', Empty, timeout=600.0)
    
    #if curr_lane:
    #   print("Terminó giro en carril derecho", curr_time, curr_lane, flush = True)
    #else: 
    #   print("Terminó giro en carril izquierdo", curr_time, curr_lane,  flush = True)    
        
def stop():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, curr_lane
    pub_cruise.publish(False)
    pub_keep_distance.publish(False)
    pub_change_lane.publish(False)
    pub_stop.publish(True)    
        
def main():
    global free_N, free_NW, free_W, free_SW, free_NE, free_E, free_SE,  success, curr_lane
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop
    global curr_time
    
    curr_time = 0.0    
    
    print("INITIALIZING POLICY...", flush=True)
    rospy.init_node("policy")
    rate = rospy.Rate(10)
        
    rospy.Subscriber("/clock", Clock, callback_sim_time)    
    rospy.Subscriber("/free/north"     , Bool, callback_free_N)
    rospy.Subscriber("/free/north_west", Bool, callback_free_NW)
    rospy.Subscriber("/free/west"      , Bool, callback_free_W)
    rospy.Subscriber("/free/south_west", Bool, callback_free_SW)
    rospy.Subscriber("/free/north_east"      , Bool, callback_free_NE)        
    rospy.Subscriber("/free/east"      , Bool, callback_free_E)    
    rospy.Subscriber("/free/south_east"      , Bool, callback_free_SE)
    rospy.Subscriber("/success", Bool, callback_success) 
    rospy.Subscriber("/current_lane", Bool, callback_curr_lane)
       
    pub_policy_started  = rospy.Publisher("/policy_started", Empty, queue_size=1)
    pub_cruise = rospy.Publisher("/cruise/enable", Bool, queue_size=1)
    pub_keep_distance    = rospy.Publisher("/follow/enable", Bool, queue_size=1)
    pub_change_lane = rospy.Publisher("/change_lane/start", Bool, queue_size=1)
    pub_action = rospy.Publisher("/action", String, queue_size=1)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=1)

    free_N  = True
    free_NW = True
    free_W  = True
    free_SW = True
    free_NE = True
    free_E  = True                
    free_SE = True
    success = True
    curr_lane = True

    action = "NA"
    action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_policy_started.publish()
        
        if success:
           # right lane
           if curr_lane:
           
              if not free_N and not free_NW and not free_W and not free_SW:
                 action = "Right Keep distance 1"
                 pub_action.publish(action)                 
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and not free_SW:
                 action = "Right Cruise 2"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and not free_SW:
                 action = "Right Keep distance 3"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and not free_W and not free_SW:
                 action = "Right Cruise 4"
                 pub_action.publish(action)                  
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and free_W and not free_SW:
                 action = "Right Keep distance 5"
                 pub_action.publish(action) 
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and not free_SW:
                 action = "Right Cruise 6"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and free_W and not free_SW:
                 action = "Right Keep distance 7"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and free_W and not free_SW:
                 action = "Right Cruise 8"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and not free_W and free_SW:
                 action = "Right Keep distance 9"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and free_SW:
                 action = "Right Cruise 10"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and free_SW:
                 action = "Right Change lane 11"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action 
                 #print ("free_N", free_N, "free_NW", free_NW, "free_W", free_W, "free_SW", free_SW, flush = True)                                   
                 change_lane()

              elif free_N and free_NW and not free_W and free_SW:
                 action = "Right Cruise 12"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and free_W and free_SW:
                 action = "Right Keep distance 13"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and free_SW:
                 action = "Right Cruise 14"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and free_W and free_SW:
                 action = "Right Change lane 15"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action
                 #print ("free_N", free_N, "free_NW", free_NW, "free_W", free_W, "free_SW", free_SW, flush = True)
                 change_lane()

              elif free_N and free_NW and free_W and free_SW:
                 action = "Right Cruise 16"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()
                 
           # left lane      
           elif not curr_lane:
              if not free_E and not free_N and not free_NE and not free_SE:
                 action = "Left Keep distance 1"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and not free_SE:
                 action = "Left Keep distance 2"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and not free_SE:
                 action = "Left Cruise 3"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif free_E and free_N and not free_NE and not free_SE:
                 action = "Left Cruise 4"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and not free_SE:
                 action = "Left Keep distance 5"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and not free_SE:
                 action = "Left Change lane 6"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action      
                 #print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane()

              elif not free_E and free_N and free_NE and not free_SE:
                 action = "Left Cruise 7"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif free_E and free_N and free_NE and not free_SE:
                 action = "Left Change lane 8"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 #print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane()

              elif not free_E and not free_N and not free_NE and free_SE:
                 action = "Left Keep distance 9"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and free_SE:
                 action = "Left Keep distance 10"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and free_SE:
                 action = "Left Cruise 11"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif free_E and free_N and not free_NE and free_SE:
                 action = "Left Cruise 12"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and free_SE:
                 action = "Left Keep distance 13"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and free_SE:
                 action = "Left Change lane 14"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 #print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)                 
                 change_lane()

              elif not free_E and free_N and free_NE and free_SE:
                 action = "Left Cruise 15"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 cruise()

              elif free_E and free_N and free_NE and free_SE:
                 action = "Left Change lane 16"
                 pub_action.publish(action)
                 if action_prev != action:
                    print(action, curr_time, flush = True)
                    action_prev = action                
                 #print ("free_E", free_E, "free_N", free_N, "free_NE", free_NE, "free_SE", free_SE, flush = True)       
                 change_lane()

        else: # not success       
           action = "stop"
           stop() 
                                              
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

