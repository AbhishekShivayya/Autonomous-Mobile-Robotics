#! /usr/bin/env python
import rospy
import time
import math
import pdb
from math import atan2
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose,Point 
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from goal_publisher.msg import PointArray
from tf.transformations import euler_from_quaternion


x=0
y=0
theta=0
get_goals = []
#rotation_direction = 1.0
#angle_to_goal = 0.0


def model(msg):     # Model state function which is continously providing x,y and theta value of robot while moving
    global x
    global y
    global theta
    x = msg.pose[1].position.x
    y = msg.pose[1].position.y
    rot_q=msg.pose[1].orientation
    (roll,pitch,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w]) #used the formula from the construct 
    
def callback(msg):
    global get_goals	
    get_goals = msg.goals
    #print (get_goals)
    

def clbk_laser(msg):
    global front,fleft,left,right,fright
    front = min(min(msg.ranges[0:20]),min(msg.ranges[341:359]),10) 
    fleft =min(min(msg.ranges[21:60]),10)
    left = min(min(msg.ranges[61:90]),10)
    right = min(min(msg.ranges[270:310]),10)
    fright = min(min(msg.ranges[311:340]),10)

#def turn_goal_direction():
 #   velocity_message.linear.x = 0.0
  #  velocity_message.angular.z = rotation_direction*(0.1+((0.2/3.14)*abs(angle_to_goal - theta)))

def stop():
    velocity_message.linear.x = 0.0
    velocity_message.angular.z =0.0   

def main():
    global velocity_message, x, y, theta,x_goal,y_goal,front
    print "moving robot"
    rospy.init_node('goal_to_goal')
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pose_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,model)
    goal_sub = rospy.Subscriber('/goals',PointArray,callback)
    scan_sub = rospy.Subscriber('/scan',LaserScan,clbk_laser)
    rospy.sleep(2) 
    i = 0
    inc_x=[0.0]*20
    inc_y=[0.0]*20
    velocity_message = Twist() 
    #rotation_direction = 1.0
   
   
    while not rospy.is_shutdown():
	    
              inc_x[i] = get_goals[i].x-x 
	      inc_y[i] = get_goals[i].y-y 
	      #distance = abs(math.sqrt(((x_goal - x)**2) + ((y_goal - y)**2)))
	      angle_to_goal = math.atan2(inc_y[i],inc_x[i])
	      #rotation_direction = np.sign(angle_to_goal - theta)
		
	      if abs(inc_x[i])>0.1 or abs(inc_y[i])>0.1:
		 
		 if abs(angle_to_goal - theta) > 0.2:
		    velocity_message.angular.z = 0.1
		    velocity_message.linear.x = 0.0
		    pub.publish(velocity_message)
	         else:
		    
		    #velocity_message.linear.x = 0.3 
		    #pub.publish(velocity_message)
                 		
		    if front > 1:
		       velocity_message.linear.x = 0.2
                       velocity_message.angular.z =0.0
		       pub.publish(velocity_message)
		       #print ("entered if ")
		    elif front < 1 and fright > 1 and fleft > 1:
			#turn_goal_direction()
			 velocity_message.linear.x = 0.0
			 velocity_message.angular.z = 0.3
		         pub.publish(velocity_message)
 	                 rospy.sleep(2)
		         velocity_message.linear.x = 0.3
		         pub.publish(velocity_message)
			 rospy.sleep(2)
			 #print ("entered elif1")
		    elif front > 1 and fright < 1 and fleft > 1:
			 velocity_message.linear.x = 0.0
			 velocity_message.angular.z = 0.1
		         pub.publish(velocity_message)
			   
		    elif front > 1 and fright > 1 and fleft < 1:
			 velocity_message.linear.x = 0.0
			 velocity_message.angular.z = -0.1
		         pub.publish(velocity_message)
			   


	      elif abs(inc_x[i])<0.1 or abs(inc_y[i])<0.1:
		   if i == 19:
                      stop()
		      pub.publish(velocity_message)
		      print ('--------------------------------')
		      print ('All 20 goals reached ')
		      print ('--------------------------------')			
		   pub.publish(velocity_message)
		   print ('-----------------------------------')
		   print ('goal',i+1,'reached',get_goals[i].x,get_goals[i].y)
		   print ('-----------------------------------')
		   i += 1				
    rospy.sleep(2)
    rospy.spin()		   

if __name__ == '__main__':
    try:
  	main()
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #rospy.spin()
    except rospy.ROSInterruptException:pass
