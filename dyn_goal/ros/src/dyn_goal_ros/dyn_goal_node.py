#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg as geometry_msgs

from geometry_msgs.msg import PoseStamped as pose
from move_base_msgs.msg import MoveBaseGoal as move_base_goal
from dyn_goal.msg import dyn_goal_msg
from std_msgs.msg import UInt8MultiArray as head_msg

from dyn_goal.my_ros_independent_class import my_generic_sum_function
   

class Memory:
	trans = geometry_msgs.Point()
	rot = geometry_msgs.Quaternion()

class Control:
	activated = True  #TODO: Just for developement. Change this to False when testing
	goal = "tracked_person"
	origin = "base_link"

class DynGoal(object):
	def __init__(self):
		rospy.init_node('dyn_goal')

		self.listener = tf.TransformListener()

	 	#Topic to publish the pose you want to reach
		self.pose_publisher = rospy.Publisher('/move_base_simple/goal', pose)

		#Topic to publish the pose you want to reach
		self.head_publisher = rospy.Publisher('/cmd_head', head_msg)

		#Subscribe to the topic that controls the dynamic goal operation
		self.sub_control = rospy.Subscriber("/move_base_simple/dyn_goal", dyn_goal_msg , self.controlCallback) 

		#initializations
		self.memory_control = 0
		self.memory = Memory()
		self.control = Control()
		self.rate = rospy.Rate(1.0)

	def destroySubscribers(self):
		self.sub_control.unregister()
		ROS_INFO("Hope you enjoyed our service! Come back any time!")
	

	def run(self):
		
		while not rospy.is_shutdown():
			if self.control.activated:
				#obtain goal tf location in relation to origin tf (default is goal=tracked_person and origin=base_link)
				try:
				    (pre_trans,rot) = self.listener.lookupTransform(self.control.origin, self.control.goal, rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				    continue

				trans = geometry_msgs.Point()
				trans.x = pre_trans[0]
				trans.y = pre_trans[1]
				trans.z = pre_trans[2]
				print trans
				#to follow the first time, memory_control is set to 0, and then 1 so it only enters the refresh goal pose
				#cycle if the pose of the tf is different
				if self.memory.trans != trans or self.memory.rot != rot or self.memory_control == 0:	
					print "Target moved!"
					new_pose = pose()
					new_pose.header.stamp = rospy.Time.now()
					#new_pose.header.seq = 2
					new_pose.header.frame_id = "map"		#TODO: check if this is right
					new_pose.pose.position.x = trans.x
					new_pose.pose.position.y = trans.y
					new_pose.pose.position.z = trans.z
					new_pose.pose.orientation.x = rot[0]
					new_pose.pose.orientation.y = rot[1]
					new_pose.pose.orientation.z = rot[2]
					new_pose.pose.orientation.w = rot[3]

					# for i in range(1,3):
					# 	new_pose.header.seq = i
					# 	print new_pose
					# 	self.pose_publisher.publish(new_pose)
					# 	rospy.sleep(1)

					#add threshold in differences (maybe in a function)
					#saves in memory the last pose sent so that it sends only different poses, and not copies
					self.memory.trans = trans
					self.memory.rot = rot
					self.memory_control = 1
			self.update_head_orientation()
					
			#When it is off make it sleep until a new message is sent to the control topic, otherwise cycle
			self.rate.sleep()

	def update_head_orientation(self):
		print("updating_head: ")

		#get pose of the head
		try:
		    (trans_head,rot_head) = self.listener.lookupTransform("base_link", "head_link", rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    # ROS_ERROR("Couldn't get head pose")
		    return

		#get pose of the robot
		try:
		    (trans_robot,rot_robot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Couldn't get robot's pose")
			return

		#get rotation of the robot body (check if it is yaw or pitch)
		quaternion_to_euler_body = tf.transformations.euler_from_quaternion(rot_robot)
		yaw_body = quaternion_to_euler_body[2]

		#calculate the angle between the POI and the robot in the map frame. The x is always positive so that when we are 
		#with negative y the angle is negative and with positive y the angle is positive
		angle_raw = math.atan((self.memory.trans.y-trans_robot[1]) / abs(self.memory.trans.x-trans_robot[0]))
		#conditions to counter the decrease of the angle
		if self.memory.trans.x-trans_robot[0] < 0 and angle_raw < 0:
			angle_refined = - (math.pi + angle_raw)
		elif self.memory.trans.x-trans_robot[0] < 0 and angle_raw > 0:
			angle_refined = math.pi - angle_raw
		else:
			angle_refined = angle_raw
		#transition of the angle from the map referencial to the robot referencial by removing the robot's rotation in the map referencial
		angle = angle_refined - yaw_body

		#condition to solve the case where the calculated angle and the robot rotation are in different multiples of 2*pi 
		if angle > math.pi:
			angle = angle - 2 * math.pi
		elif angle < -math.pi:
			angle = angle + 2 * math.pi

		#get the rotation of the head
		quaternion_to_euler_head = tf.transformations.euler_from_quaternion(rot_head)
		yaw_head = quaternion_to_euler_head[2]

		#compare the goal angle with the head orientation
		print "angle and pitch : " , angle ," | " , yaw_head , "  | yaw_body : " , yaw_body, "  |  raw: " , angle_raw , "  | refined : ", angle_refined
		if abs(angle - yaw_head) > 0.1:	#tune this threshold
			control_head_msg = head_msg() 	#also has a layout (MultiArrayLayout. Maybe i need to work with that)
			#i dont know if the 300 is ok. theoretically its the speed. check if i need to resize the data.

			control_head_msg.data = [300 , 0]
			control_head_msg.data[0] = 300
			control_head_msg.data[1] = angle

			# self.head_publisher.publish(control_head_msg)
			# ROS_INFO("UPDATED HEAD ANGLE TO ", angle)
		return
		# print "pos(pessoa): (", self.memory.trans[0], ",", self.memory.trans[1], ") | x(robot): (" , trans_robot[0], ",", trans_robot[1], ") | angulo e : ", angle
		
		 

			
	#Callback called when receiving a control instruction		
	def controlCallback(self, data):
		self.control.activated = data.activated
		self.control.goal = data.dyn_goal_tf
		self.control.origin = data.origin_tf


def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = DynGoal()
	# call run method of class DynGoal
	my_object.run()
	# destroy subscriptions
	my_object.destroySubscribers()

if __name__ == '__main__':
	main()
