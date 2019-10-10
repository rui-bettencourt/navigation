#!/usr/bin/env python
import roslib
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg as geometry_msgs
import std_msgs
import rospkg

from rospy import loginfo as log
from geometry_msgs.msg import PoseStamped as pose
from geometry_msgs.msg import PointStamped as point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose
from move_base_msgs.msg import MoveBaseGoal as move_base_goal
from std_msgs.msg import MultiArrayDimension as MAD
from dyn_goal.msg import dyn_goal_msg
from std_msgs.msg import UInt8MultiArray as head_msg
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid

#Importing dijkstra functions
from dijkstra import Graph

from dyn_goal.my_ros_independent_class import my_generic_sum_function
   
#memory variables (trans is the last known poi position; rot last known poi orientation; control: if it is the first time)
class Memory:
	trans = geometry_msgs.Point()
	rot = geometry_msgs.Quaternion()
	control = False

class Control:
	activated = False  #TODO: Just for developement. Change this to False when testing
	goal = "tracked_person"
	origin = "base_link"
	dist = 0.0

class DynGoal(object):
	def __init__(self):
		rospy.init_node('dyn_goal')

		self.listener = tf.TransformListener()

	 	#Topic to publish the pose you want to reach
		self.pose_publisher = rospy.Publisher('/move_base_simple/goal', pose, queue_size=1)

		#Topic to publish the pose you want to reach
		self.head_publisher = rospy.Publisher('/cmd_head', head_msg, queue_size=2)

		#Subscribe to the topic that controls the dynamic goal operation
		self.sub_control = rospy.Subscriber("/move_base_simple/dyn_goal", dyn_goal_msg , self.controlCallback)

		#Subscribe to the topic of the static goal
		self.sub_static_goal = rospy.Subscriber("/move_base_simple/goal",PoseStamped, self.staticCallback) 

		#Subscribe to Person position
		self.poi_pose = rospy.Subscriber("/people_follower/person_position", point, self.poiCallback)

		#Subscribe to Costmap
		self.sub_costmap_2d_update = rospy.Subscriber("/move_base/global_costmap/costmap_updates",OccupancyGridUpdate, self.costmapUpdateCallback)
		self.sub_costmap_2d = rospy.Subscriber("/move_base/global_costmap/costmap",OccupancyGrid, self.costmapCallback)

		#Thresholds
		self.movementThreshold = 0.1
		self.availabilityThreshold = 20
		self.convert_offset = 0.0

		#initializations
		self.memory = Memory()
		self.control = Control()
  		self.map_ = None
		self.map_info = None
		self.updatingGoal = False
		self.rate = rospy.Rate(10.0)

	def destroySubscribers(self):
		rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',-0.6)
		rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.6)
		self.sub_control.unregister()
		self.poi_pose.unregister()
		self.sub_costmap_2d.unregister()
		self.sub_costmap_2d_update.unregister()
		log("Hope you enjoyed our service! Come back any time!")
	

	def run(self):
		
		while not rospy.is_shutdown():
			if self.control.activated:
				#set velocity to only front and a little slower
				if not self.memory.control:
					rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',0.0)
					rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.5)
				#obtain goal tf location in relation to origin tf (default is goal=tracked_person and origin=base_link)
				try:
				    (trans,rot) = self.listener.lookupTransform(self.control.origin, self.control.goal, rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				    continue

				target_position = self.vectortToPoint(trans)

				#get pose of the robot
				try:
				    (self.trans_robot , self.rot_robot) = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					rospy.logerr("Couldn't get robot's pose")
					return

				trans_robot = self.vectortToPoint(self.trans_robot)

				#check if it's close enough to the target using the distance threshold indicated in the control topic message
				# if self.isGoalClose(trans_robot):
				# 	self.memory.trans = target_position
				# 	target_position = trans_robot


				#to follow the first time, memory_control is set to 0, and then 1 so it only enters the refresh goal pose
				#cycle if the pose of the tf is different
				if self.hasTargetMoved(target_position) or not self.memory.control:
					print("Target moved!")
					new_pose = pose()
					new_pose.header.stamp = rospy.Time.now()
					#new_pose.header.seq = 2
					new_pose.header.frame_id = "map"		#TODO: check if this is right
					#Change target_position to be:
					final_position = self.chooseGoal(target_position, trans_robot)
					final_orientation = self.getAngle(final_position,target_position)
					new_pose.pose.position.x = final_position.x
					new_pose.pose.position.y = final_position.y
					new_pose.pose.position.z = final_position.z
					new_pose.pose.orientation.x = final_orientation[0]
					new_pose.pose.orientation.y = final_orientation[1]
					new_pose.pose.orientation.z = final_orientation[2]
					new_pose.pose.orientation.w = final_orientation[3]

					for i in range(1,3):
						new_pose.header.seq = i
						print(new_pose)
						self.updatingGoal = True
						self.pose_publisher.publish(new_pose)
						rospy.sleep(1)

					#add threshold in differences (maybe in a function and call in the if maybe)
					#saves in memory the last pose sent so that it sends only different poses, and not copies
					self.memory.trans = final_position
					self.memory.rot = rot  #rot is probably not needed
					self.memory.control = True
				else:
					self.memory.trans = target_position
					self.memory.rot = rot  #rot is probably not needed
					self.memory.control = True

				self.updateHeadOrientation()

			elif self.memory.control:
				#reset the control variable and the speed params
				self.memory.control = False
				rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',-0.6)
				rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.6)

			#When it is off make it sleep until a new message is sent to the control topic, otherwise cycle
			self.rate.sleep()

	#Function to detect if the target movement is relevant to resend as goal
	def hasTargetMoved(self,trans):
		dist = self.dist2D(self.memory.trans.x , self.memory.trans.y , trans.x , trans.y)
		if dist > self.movementThreshold:
			return True
		else:
			return False


	#Function that detects if the goal is aleady closer than the threshold
	def isGoalClose(self,trans):
		dist = self.dist2D(self.memory.trans.x , self.memory.trans.y , trans.x , trans.y)
		if dist < self.control.dist:
			print("Too close!!!!!!!!!!!!")
			return True
		else:
			return False

    #Function used to sort the points on the circle around the target goal by distance to the robot
	def distanceToRobot(self,point):
		return self.dist2D(point.x, point.y, self.robot_position.x, self.robot_position.y)

    #Function that determines if a certain point on the map is available to be set as goal
	def isCellAvailable(self, point):
        #get the costmap by subscribing to nav_msgs/OccupancyGrid.msg
		[x,y]=self.worldToMap(point.x,point.y)
		cost = self.map_[int(round(x))][int(round(y))]
		print("cost is: " + str(cost))
		if cost == 0:
			return True
		else:
			return False

	def chooseGoal(self,target_position, robot_position):
        #number of points to check in the circumference
		number_points = int(self.control.dist * 50)
        #radius of the circle
		r = self.control.dist

		self.robot_position = robot_position

		if r > 0:
			#make a circle of positions, check which ones are closer to the robot, and if it is reachable
			circle = []
			for i in range(0,number_points):
				point = Point()
				point.x = round(math.cos(2*math.pi/number_points*i)*r,2) + target_position.x
				point.y = round(math.sin(2*math.pi/number_points*i)*r,2) + target_position.y
				point.z = 0
				circle.append(point)
			#order the circle variable by proximity to the ROBOT
			circle.sort(key=self.distanceToRobot)

			#iterate the ordered set check if it is a freeeeee cell
			while not rospy.is_shutdown() and len(circle) != 0:
				if not self.isCellAvailable(circle[0]):
					del circle[0]
				else:
					break
			#BONUS: check if there is a path from this point to the target ;)
		else:
			circle.append(target_position)

        #return goal, that is the closest point to the robot that was not removed from the list by the previous conditions
		if len(circle) >=0:
			return circle[0]
		else:
			return target_position

	def dist2D(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) 

	def vectortToPoint(self,vector):
		point = geometry_msgs.Point()
		point.x = vector[0]
		point.y = vector[1]
		point.z = vector[2]
		return point

	def getAngle(self, origin, target):
		angle_raw = math.atan((target.y- origin.y) / abs(target.x-origin.x))
		#conditions to counter the decrease of the angle
		if target.x-origin.x < 0 and angle_raw < 0:
			angle_refined = - (math.pi + angle_raw)
		elif target.x-origin.x < 0 and angle_raw > 0:
			angle_refined = math.pi - angle_raw
		else:
			angle_refined = angle_raw

		angle = angle_refined

		#condition to solve the case where the calculated angle and the robot rotation are in different multiples of 2*pi
		if angle > math.pi:
			angle = angle - 2 * math.pi
		elif angle < -math.pi:
			angle = angle + 2 * math.pi

		#get the rotation of the body
		return tf.transformations.quaternion_from_euler(0,0,angle)

	#Function that depending where the target is changes the orientation of the head to follow that target
	def updateHeadOrientation(self):
		#get pose of the head
		try:
		    (trans_head,rot_head) = self.listener.lookupTransform("base_link", "head_link", rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    rospy.logerr("Couldn't get head pose")
		    return

		#get rotation of the robot body (check if it is yaw or pitch)
		quaternion_to_euler_body = tf.transformations.euler_from_quaternion(self.rot_robot)
		yaw_body = quaternion_to_euler_body[2]

		#calculate the angle between the POI and the robot in the map frame. The x is always positive so that when we are 
		#with negative y the angle is negative and with positive y the angle is positive
		if self.memory.trans.x-self.trans_robot[0] != 0:
			angle_raw = math.atan((self.memory.trans.y- self.trans_robot[1]) / abs(self.memory.trans.x-self.trans_robot[0]))
			#conditions to counter the decrease of the angle
			if self.memory.trans.x-self.trans_robot[0] < 0 and angle_raw < 0:
				angle_refined = - (math.pi + angle_raw)
			elif self.memory.trans.x-self.trans_robot[0] < 0 and angle_raw > 0:
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
			# print "angle and pitch : " , angle ," | " , yaw_head , "  | yaw_body : " , yaw_body, "  |  raw: " , angle_raw , "  | refined : ", angle_refined
			if abs(angle - yaw_head) > 0.01:	#tune this threshold
				control_head_msg = head_msg() 	#also has a layout (MultiArrayLayout. Maybe i need to work with that)
				#i dont know if the 300 is ok. theoretically its the speed. check if i need to resize the data.
				control_head_msg.layout.dim = [MAD() , MAD()]
				# control_head_msg.layout.dim[0].label = "speed"
				# control_head_msg.layout.dim[0].size = 12
				# control_head_msg.layout.dim[0].stride = 12
				# control_head_msg.layout.dim[1].label = "angle"
				# control_head_msg.layout.dim[1].size = 100
				# control_head_msg.layout.dim[1].stride = 100
				# control_head_msg.layout.data_offset = 10
				#angle needs to be in degrees and the center of the robot is at 90 degrees
				angle = 90 - (angle * 360 / (2 * math.pi))
				if angle < 5: 
					angle = 5
				elif angle > 175:
					angle = 175 
				control_head_msg.data = [0 , 200]
				control_head_msg.data[1] = 100
				control_head_msg.data[0] = angle

				self.head_publisher.publish(control_head_msg)
				log("UPDATED HEAD ANGLE TO %f", angle)
		return
		# print "pos(pessoa): (", self.memory.trans[0], ",", self.memory.trans[1], ") | x(robot): (" , trans_robot[0], ",", trans_robot[1], ") | angulo e : ", angle
		
		 
	#Function to tranform a map coordinate into a world one
	def mapToWorld(self, mx, my):
		wx = self.map_info.info.origin.position.x + (mx + self.convert_offset) * self.map_info.info.resolution
		wy = self.map_info.info.origin.position.y + (my + self.convert_offset) * self.map_info.info.resolution
		return [wx, wy]

	#Function to tranform a world coordinate into a map one
	def worldToMap(self, wx, wy):
		origin_x = self.map_info.info.origin.position.x
		origin_y = self.map_info.info.origin.position.y
		resolution = self.map_info.info.resolution

		if (wx < origin_x or wy < origin_y):
			return None

		mx = (wx - origin_x) / resolution - self.convert_offset
		my = (wy - origin_y) / resolution - self.convert_offset

		if (mx < self.map_info.info.width and my < self.map_info.info.height):
			return [mx , my]

		return None
			
	#Callback called when receiving a control instruction		
	def controlCallback(self, data):
		self.control.activated = data.activated
		self.control.goal = data.dyn_goal_tf
		self.control.origin = data.origin_tf
		if data.dist >= 0:
			self.control.dist = data.dist
		else:
			self.control.dist = 1.0
		print(self.control.dist)

	#Callback called when receiving a control instruction		
	def poiCallback(self, data):
		self.poiPose = data

	#Callback called when a static goal is received, it will deactivate the dynamic goal to ensure the mutual exclusivity
	def staticCallback(self, data):
		if self.updatingGoal:
			self.updatingGoal = False	#this means that the message received was sent by dyn_goal and should not deactivate the dyn_goal
		else:
			self.control.activated = False
			print("Dyn_Goal deactivated")

	#Callback for the costmap
	def costmapUpdateCallback(self, data):  #TODO: need to update the map
		x=0
		#self.map_ = data		#TODO: maybe smthg more
  		# print("x=" + str(self.map_.x))
		# print("y=" + str(self.map_.y))
		# print("width=" + str(self.map_.width))
		# print("height=" + str(self.map_.height))
		# # get an instance of RosPack with the default search paths
		# rospack = rospkg.RosPack()

        # # get the file path for rospy_tutorials
		# save_path = rospack.get_path('dyn_goal') + '/logs'
        # # print(save_path)

		# file_object = open(save_path+'/map_updates.txt', 'w')
		# file_object.write(str(self.map_.data))
		# file_object.close()
		# print("amount of cells: " + str(len(self.map_.data)))
  
  	#Callback for the costmap
	def costmapCallback(self, data):
		self.map_info = data		#TODO: maybe smthg more
		print("t=" + str(self.map_info.info.map_load_time))
		print("resolution=" + str(self.map_info.info.resolution))
		print("width=" + str(self.map_info.info.width))
		print("height=" + str(self.map_info.info.height))
		print(self.map_info.info.origin)

		#self.map_ = np.arrange(data.info.width*data.info.height).reshape(data.info.width,data.info.height)
		self.map_ = np.zeros((data.info.width,data.info.height))

		for i in range(0,data.info.height):
			for j in range(0,data.info.width):
				self.map_[j][i] = data.data[i*data.info.width + j]
		self.map_info.data = None

def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = DynGoal()
	# call run method of class DynGoal
	my_object.run()
	# destroy subscriptions
	my_object.destroySubscribers()

if __name__ == '__main__':
	main()
