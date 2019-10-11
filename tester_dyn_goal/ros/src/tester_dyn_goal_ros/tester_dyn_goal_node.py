#!/usr/bin/env python
import rospy
import tf
from dyn_goal.msg import dyn_goal_msg

from tester_dyn_goal.my_ros_independent_class import my_generic_sum_function

from rospy import init_node, is_shutdown

def main():

	init_node('tester_dyn_goal_node')

	publisher = rospy.Publisher('/move_base_simple/dyn_goal', dyn_goal_msg)

	rate = rospy.Rate(10.0)
	goal = dyn_goal_msg()
	goal.activated = False
	goal.dyn_goal_tf = "tracked_person"
	goal.origin_tf = "base_link"
	goal.dist = 1.2
	something = " "

	while not is_shutdown() and something != "exit":
	 	if publisher.get_num_connections() > 0:
			something = raw_input()

			if not something:
				goal.activated = not goal.activated
				publisher.publish(goal)
			elif something == 'r':
				goal.dyn_goal_tf = "tracked_person"
				goal.origin_tf = "base_link"
				goal.activated = False
				publisher.publish(goal)
			elif something != "exit":
				something2 = raw_input()
				goal.dyn_goal_tf = something
				goal.origin_tf = something2
				goal.activated = True
				publisher.publish(goal)
			print goal

	publisher.unregister()



