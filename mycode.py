#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from math import sqrt
import time
import actionlib
from goal_publisher.msg import PointArray
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class move():
	#----------# Init module initialises all the member variables and subscribes to topics---------------#
	def __init__(self):
		self.sub = rospy.Subscriber('/goals', PointArray, self.goals)
		self.pos_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.robot_position)
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.vel = Twist()
		time.sleep(2)
		self.rate = rospy.Rate(100)
		self.x = 0
		self.y = 0
		self.r =0
		self.f =0
		self.j = 0
		self.x_pos = 0
		self.y_pos = 0
		self.distance = 0
		#self.final_points=[]
		self.get_goals=[]
		self.rem_goals=[]
		self.reward_gained=[]
		self.reached_goals=[]
		self.cancelled_goals=[]

        #---------Goals subscribed from Goal Pulisher and assigning to list for further sorting function--------#

	def goals(self,msg):
		self.raw_goals = msg.goals

	#----------Function for sorting the goals according to the cost function calculated---------------#
	
	def sorting_goals(self):
		cost_list = []	
		final_points = []	
		for i in range(len(self.rem_goals)):
			#print(len(self.rem_goals))
			dist = self.Ecd(self.x_pos,self.y_pos,self.rem_goals[i][0], self.rem_goals[i][1])
			final_points.append([self.rem_goals[i][0], self.rem_goals[i][1], self.rem_goals[i][2], dist])
			#print(len(final_points))		
		total_sum = np.sum(final_points,0)
		norm_dist = [pt[3]/total_sum[3] for pt in final_points]
		norm_reward = [pt[2]/total_sum[2] for pt in final_points]
		cost = np.true_divide(norm_dist, norm_reward)
		for j in range(len(final_points)):
			cost_list.append([final_points[j][0], final_points[j][1], cost[j], final_points[j][2], final_points[j][3]])
		cost_list.sort(key=lambda item: item[2])
		#print(cost_list)		
		return cost_list

        #------core is the main function of this class which calls other member functions to perform robot movement to goals-----#

	def core(self):		
		goals = []	
		for i in range(len(self.raw_goals)):
			goals.append([self.raw_goals[i].x, self.raw_goals[i].y, self.raw_goals[i].reward])
		self.rem_goals = goals
		self.one_rotation()
		while len(self.rem_goals)>0:
			r_goals = []
			cost_list = self.sorting_goals()
			goal_point = self.attempt_goal(cost_list)			
			goal_pose = self.pose(goal_point)
			client.send_goal(goal_pose)
			rospy.loginfo("Attempting to Goal: ({},{})".format(cost_list[0][0], cost_list[0][1]))
			start_time = time.time()
			flag=1
			while flag == 1:
				status = client.get_state()
				
				if status ==3:
					self.reached_goals.append([cost_list[0][0], cost_list[0][1], cost_list[0][3]])
					self.reward_gained.append(cost_list[0][3])
					rospy.loginfo("Reached Goal : ({},{}) with {} reward points".format(cost_list[0][0], cost_list[0][1], cost_list[0][3]))
					rospy.loginfo("Total Rewards Gained : {} ".format(np.sum(self.reward_gained)))
					flag = 0
				elif (time.time() - start_time) > 90:
					self.cancelled_goals.append([cost_list[0][0], cost_list[0][1], cost_list[0][3]])
					flag = 0
					rospy.loginfo("This goal is skipped")
			r_goals = [cost_list[0][0], cost_list[0][1], cost_list[0][3]]
			
			for i in self.rem_goals:
				if i == r_goals:
					self.rem_goals.remove(i)
					if len(self.rem_goals) == 0:
						self.rem_goals = self.cancelled_goals
						self.cancelled_goals = []
						rospy.loginfo("Now trying to reach the skipped goals")
						
							
			
			
		
	#------Function for extarcting the goal Point from sorted list to give input to Move_base--------------#
	
	def attempt_goal(self, cost_list):	
		goal_point = (cost_list[0][0],cost_list[0][1])
		return goal_point

        #-----pose function prepares the goal point that is later sent to the move_base action server-----------#

	def pose(self, goal_point):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = goal_point[0]
                goal_pose.target_pose.pose.position.y = goal_point[1]
		goal_pose.target_pose.pose.position.z = 0 
		goal_pose.target_pose.pose.orientation.x = 0 
		goal_pose.target_pose.pose.orientation.y = 0 
		goal_pose.target_pose.pose.orientation.z = 0 
		goal_pose.target_pose.pose.orientation.w = 1
		return goal_pose						
			
	#------------Function for calculating the Euclidian Distance from current Position to Goal Position------#

	def Ecd(self,x1,y1,x2,y2): 
		self.distance = sqrt((x2-x1)**2 + (y2-y1)**2)
		return self.distance

        #-----------Function for one rotation of Robot for Localization---------------#

	def one_rotation(self):
		self.vel.angular.z = 3.0
		self.vel.linear.x = 0.0
		start_time = time.time()
		rospy.loginfo("Robot Localization")
		while time.time()-start_time < 5:
                	self.pub_vel.publish(self.vel)
		self.vel.angular.z = 0.0
		self.vel.linear.x = 0.0
		self.pub_vel.publish(self.vel)
			
        #------------Function to read the Current Position of the Robot----------------#

	def robot_position(self,present_pos):
        	self.x_pos = present_pos.pose.pose.position.x
        	self.y_pos = present_pos.pose.pose.position.y
		
		
if __name__=='__main__':
	rospy.init_node('move')
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	target = move()	
	time.sleep(2)
	target.core()	
	rospy.spin()


	


