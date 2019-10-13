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
from geometry_msgs.msg import PoseWithCovarianceStamped as pose
from geometry_msgs.msg import PointStamped as point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose
from move_base_msgs.msg import MoveBaseGoal as move_base_goal
from std_msgs.msg import MultiArrayDimension as MAD
from std_msgs.msg import String
from dyn_goal.msg import dyn_goal_msg
from std_msgs.msg import UInt8MultiArray as head_msg
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid

#for rviz visualization
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class Analysis(object):
    def __init__(self):
        rospy.init_node('analysis')
        #Subscribe to the topic of the static goal
        self.sub_localization = rospy.Subscriber("/amcl_pose", pose, self.writeCallback,queue_size=20) 
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback,queue_size=20) 
        self.amcl = None
        self.goal = None
        self.new = False
        print("here too")
        self.rate = rospy.Rate(50)
        
    def run(self):
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        save_path = rospack.get_path('dyn_goal') + '/logs'
        # print(save_path)
        print("Im here")
        file_locx = open(save_path+'/localizationx.txt', 'a')
        file_locy = open(save_path+'/localizationy.txt', 'a')
        file_locgoal = open(save_path+'/localizationgoal.txt', 'a')
        file_locangle = open(save_path+'/localizationangle.txt', 'a')
        file_loccov = open(save_path+'/localizationcov.txt', 'a')
        file_loctime = open(save_path+'/time.txt', 'a')
        while not rospy.is_shutdown():
            if self.new:
                file_locx.write(str(self.amcl.pose.pose.position.x))
                file_locx.write("\n")
                file_locy.write(str(self.amcl.pose.pose.position.y))
                file_locy.write("\n")
                file_locgoal.write(str(self.goal))
                file_locgoal.write("\n")
                file_loccov.write(str(self.amcl.pose.covariance))
                file_loccov.write("\n")
                file_loctime.write(str(rospy.get_time()))
                file_loctime.write("\n")
                q = (self.amcl.pose.pose.orientation.x,
                     self.amcl.pose.pose.orientation.y,
                     self.amcl.pose.pose.orientation.z,
                     self.amcl.pose.pose.orientation.w)
                angle = tf.transformations.euler_from_quaternion(q)
                yaw = angle[2]
                file_locangle.write(str(yaw))
                file_locangle.write("\n")
                self.new=False
                print("Writing")
            self.rate.sleep()
            
        file_locx.close()
        file_locy.close()
        file_locgoal.close()
        file_loccov.close()
        file_loctime.close()
        file_locangle.close()
        
    def writeCallback(self,data):
        self.amcl = data   
        self.new = True
        
    def goalCallback(self,data):
        self.goal = data
def main():
    	# create object of the class DynGoal (constructor will get executed!)
	my_object = Analysis()
	# call run method of class DynGoal
	my_object.run()

if __name__ == '__main__':
	main()