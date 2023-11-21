#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
import numpy as np

class following_path:
	def __init__(self):
		self.wheelbase = 2.00
		self.fram_id = 'odom'
#		self.twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') #获取命名空间下的参数
#		self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/vesc/low_level/ackermann_cmd_mux/input/navigation')
#		wheelbase = rospy.get_param('~wheelbase', 2.0)
#		frame_id = rospy.get_param('~frame_id', 'odom')
		
		self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
		self.path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.callback_read_goal, queue_size=1)
		self.path_info = []
		self.current_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.read_current_pose, queue_size=1)
		self.path_goal = []
		self.current_pose = []
		self.msg = AckermannDriveStamped()
		self.controller = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback, queue_size=1)
	
	def callback_read_goal(self, data):
		path_array = data.poses
		for path_pose in path_array:
			path_x = path_pose.pose.position.x
			path_y = path_pose.pose.position.y
			self.path_info.append([float(path_x), float(path_y)])
		if not len(self.path_info) == 0:
			self.path_goal = self.path_info[-1]

	def read_current_pose(self, data):
		if not len(self.path_info) == 0:
			x = data.pose.pose.position.x
			y = data.pose.pose.position.y
			self.current_pose = [float(x), float(y)]
		else:
			self.msg.drive.steering_angle = 0
			self.msg.drive.speed = 0

	def pose_dist(self, p1, p2):
		if len(self.path_info) != 0:
			try:
				return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
			except:
				return 1

	def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
		if omega==0 or v==0:
			return 0
	
		radius = v / omega
		return math.atan(wheelbase / radius)

	def cmd_callback(self, data):
		v = data.linear.x
		steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)
		self.msg.header.stamp = rospy.Time.now()
		self.msg.drive.steering_angle = 1.3*steering
		self.msg.drive.speed = v
		if self.pose_dist(self.path_goal, self.current_pose) < 1:
#			rospy.signal_shutdown(1)
			while 1:
				self.msg.drive.steering_angle = 0
				self.msg.drive.speed = 0
				self.path_info = []
		if v == 0: #and self.pose_dist(self.path_goal, self.current_pose) > 1
			self.msg.drive.speed = -0.05
		self.pub.publish(self.msg)
#*#*#*#*#*#*#*#*#*#*#*#*#*#
#                         #
#	 Mr.Song          #
#			  #
#*#*#*#*#*#*#*#*#*#*#*#*#*#

if __name__ == '__main__':
	rospy.init_node("cmd_vel_to_ackermann_drive")
	following_path()
	rospy.spin()
