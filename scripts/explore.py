#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class explore(): #class to explore the map going to set points using map .pgm
    def __init__(self):
        #pub to cmd_vel to do rotations
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel',
                                       Twist, queue_size=1)
         
         #subscribe to this topic which publishes when objects are found                     
        self.found_obj_sub = rospy.Subscriber("/found_obj",
                                        String, self.found_obj_callback) 
                                        
        #Location coordinates that robot must visit                                
        self.locations = [[-4.0,-4.5], [-2,-4.5], [0.0,-4.5], [2.0,-4.5], [4.0,-4.5], 
                          [4.0,-2.0], [2.0,-1.5], [0.0,-1.5], [-2.0,-1.5], [-4.0,-2.0], 
                          [-3.5,0.5], [-3.5, 2.7], [-4.5,4.8], [-1.0,5.0], [1.0,4.5], 
                          [3.0,4.5], [-1.0, 2.0], [1.0, 2.0], [3.7, 2.0], [3.7, 0.0],
                          [0.0, 0.0]]
                          
        self.currentLoc = 0
        
        self.allFound = 0 #count to see if all objects found
        
        #Loop through locations, checking ros is still up
        while self.currentLoc < len(self.locations) and not rospy.is_shutdown():
            if self.allFound < 4: #if all objects not found go to next location and rotate           
                self.moveToGoal(self.locations[self.currentLoc])
                self.rotate()
            else: #if all objects found, return home
                self.currentLoc = len(self.locations)
                rospy.loginfo("All objects found, returning to start!")
                self.moveToGoal(self.locations[0])
               
        rospy.loginfo("All locations have been visited!")
    
    #each time object found, add 1    
    def found_obj_callback(self, msg):
            self.allFound += 1
    # function to move robots to location coordinates in map frame    
    def moveToGoal(self,Goal):
        #create move_base action client, allowing to move the turtlebot
        ac = actionlib.SimpleActionClient("/turtlebot/move_base", MoveBaseAction)
        
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for the move_base action server to come up")
        
        #create a movebasegoal which is a posestamped msg
        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map" #what in absolute frame not base_link
        goal.target_pose.header.stamp = rospy.Time.now()# time stamp

        #set the coordinates and orientation of goal location
        goal.target_pose.pose.position.x =  Goal[0]
        goal.target_pose.pose.position.y =  Goal[1]
        goal.target_pose.pose.position.z =  0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
         
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal) #send move base goal to action client
         
        ac.wait_for_result() #wait for result
         
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):#check if successful
            rospy.loginfo("You have reached location %s", self.currentLoc)
            return 1
        else:
            rospy.loginfo("The robot failed to reach the location")
            return -1
    
    #function to rotate robot once location is reached    
    def rotate(self):
        rospy.loginfo("Doing rotation at location %s", self.currentLoc)
        #create twist msg to send to cmd_vel
        vel_msg = Twist()
        vel_msg.angular.z = 0.8
        relative_angle = 9.42478
        
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        r = rospy.Rate(10)# limit whole loop to save resources
        while(current_angle < relative_angle) and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = vel_msg.angular.z*(t1-t0)
            r.sleep()
            
        vel_msg.angular.z = 0 # stop the rotation once relative_angle reached
        self.cmd_vel_pub.publish(vel_msg)
        self.currentLoc += 1 #go to next location as rotation done        

if __name__ == '__main__':
    try:
        rospy.init_node('explore')
        explore = explore()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object search node terminated.")
