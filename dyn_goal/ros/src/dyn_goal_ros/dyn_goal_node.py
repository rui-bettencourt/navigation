#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg as geometry_msgs

from geometry_msgs.msg import PoseStamped as pose
from move_base_msgs.msg import MoveBaseGoal as move_base_goal
from move_base_msgs.msg import MoveBaseActionGoal as mbag

from dyn_goal.my_ros_independent_class import my_generic_sum_function
   

class Memory:
	rot = geometry_msgs.Point()
	trans = geometry_msgs.Quaternion()


def main():
	rospy.init_node('dyn_goal')

	listener = tf.TransformListener()

 
	pose_publisher = rospy.Publisher('/move_base_simple/goal', pose)
	goal_publisher = rospy.Publisher('/move_base/goal',mbag)
	memory_control = 0
	memory = Memory()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():

		#to test the code a random tf is created
		br = tf.TransformBroadcaster()
		br.sendTransform((0, -1, 0),tf.transformations.quaternion_from_euler(0, 0, 0),
			rospy.Time.now(), "test_goal","map")

		try:
		    (trans,rot) = listener.lookupTransform('base_link', 'tracked_person', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    continue
		if memory_control == 0:
			memory.trans = trans
			memory.rot = rot
			memory_control = 1

		if memory.trans != trans or memory.rot != rot or memory_control == 1:	
			new_pose = pose()
			new_pose.header.stamp = rospy.Time.now()
			new_pose.header.seq = 2
			new_pose.header.frame_id = "map"
			new_pose.pose.position.x = trans[0]
			new_pose.pose.position.y = trans[1]
			new_pose.pose.position.z = trans[2]
			new_pose.pose.orientation.x = rot[0]
			new_pose.pose.orientation.y = rot[1]
			new_pose.pose.orientation.z = rot[2]
			new_pose.pose.orientation.w = rot[3]

			pose_to_send = move_base_goal()
			pose_to_send.target_pose = new_pose

			new_goal = mbag()
			new_goal.goal = pose_to_send
			new_goal.header = new_pose.header
			new_goal.goal_id.stamp = new_goal.header.stamp
			new_goal.goal_id.id = "dyn_goal"

			for i in range(1,3):
				new_pose.header.seq = i
				print new_pose
				pose_publisher.publish(new_pose)
				rospy.sleep(1)
			# goal_publisher.publish(new_goal)

			memory.trans = trans
			memory.rot = rot
			memory_control = 2

		

		rate.sleep()
