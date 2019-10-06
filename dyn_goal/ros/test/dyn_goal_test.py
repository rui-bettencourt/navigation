import math
import rospy
import roslib
from geometry_msgs.msg import Point	
import geometry_msgs.msg as geometry_msgs
import matplotlib.pyplot as plt 
#Choose a point close to the target person and set it as a goal

#Importing dijkstra functions
from dijkstra import Graph

class Control:
	activated = False  #TODO: Just for developement. Change this to False when testing
	goal = "tracked_person"
	origin = "base_link"
	dist = 0.0

class test(object):
    def __init__(self):
        self.control = Control()
        self.control.dist = 1.5
        self.person_position = self.vectortToPoint([1,1,0])
        self.robot_position = self.vectortToPoint([1,5,0])
        
    #Function used to sort the points on the circle around the target goal by distance to the robot
    def distanceToRobot(self,point):
        return self.dist2D(point.x, point.y, self.robot_position.x, self.robot_position.y)
    
    #Function that determines if a certain point on the map is available to be set as goal
    def isCellAvailable(self, point):
        #get the costmap by subscribing to nav_msgs/OccupancyGrid.msg
        return True

    def chooseGoal(self,person_position, robot_position):
        #number of points to check in the circumference
        number_points = int(self.control.dist * 50)
        #radius of the circle
        r = self.control.dist
        
        self.robot_position = robot_position
        
        
        #Simple solution
        #make a line from the person_position and the robot position. Use the point at self.control.dist
        
        #make a circle of positions, check which ones are closer to the robot, and if it is reachable
        circle = []
        for i in range(0,number_points):
            point = Point()
            point.x = round(math.cos(2*math.pi/number_points*i)*r,2) + person_position.x
            point.y = round(math.sin(2*math.pi/number_points*i)*r,2) + person_position.y
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

        #return goal, that is the closest point to the robot that was not removed from the list by the previous conditions
        return circle[0]


        # x = [] 
        
        # y = [] 
        
        # for j in range(0,number_points):
        #     x.append(circle[j].x)
        #     y.append(circle[j].y)
        # print x
        # print y
        
        # plt.scatter(x, y, label= "stars", color= "green",  
        #     marker= "*", s=30) 
        
        # plt.xlabel('x - axis') 

        # plt.ylabel('y - axis') 
        
        # plt.title('My first graph!') 
        
        # plt.show() 
        
    def vectortToPoint(self,vector):
            point = geometry_msgs.Point()
            point.x = vector[0]
            point.y = vector[1]
            point.z = vector[2]
            return point
        
    def dist2D(self,x1,y1,x2,y2):
    		return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) 
        
def main():
    my_object = test()
	# call run method of class DynGoal
    print my_object.chooseGoal(my_object.person_position, my_object.robot_position)
    
    graph = Graph([("a", "b"),  ("a", "c"),  ("a", "f"), ("b", "c"),("b", "d"), ("c", "d"), ("c", "f"),  ("d", "e"),("e", "f")])
    print graph.dijkstra("a", "e")
    
if __name__ == '__main__':
	main()