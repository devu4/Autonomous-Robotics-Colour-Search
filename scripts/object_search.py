#!/usr/bin/env python
import rospy, cv2, cv_bridge
from actionlib_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#class that searches and follows coloured objects
class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()# use opencv
    
    #subscribe to turtlebot camera image                                    
    self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",
                                        Image, self.image_callback)
    # create publisher for cmd_vel to follow objects
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel',
                                       Twist, queue_size=1)
    # create publisher to publish when objects are found
    self.found_obj_pub = rospy.Publisher('/found_obj', String, queue_size=1)
    
    self.twist = Twist() # Twist msg for cmd_vel when objects found
    
    #upper and lower hsv values for colours we are searching for
    self.lower = {'red':(0, 100, 100), 'green':(55, 100, 100), 'blue':(115, 100, 100), 'yellow':(20, 100, 100)} #assign new item lower['blue'] = (93, 10, 0)
    self.upper = {'red':(5,255,255), 'green':(65,255,255), 'blue':(125,255,255), 'yellow':(40,255,255)}
    self.found = {} # dictionary for found colours
   
  #callback for turtlebot camera subscriber
  def image_callback(self, msg):
    #first convert msg to bgr8 image
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # create a hsv image from bgr
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #for each colour objects being looked for
    for key, value in self.upper.items():
        #create mask using lower and upper hsv values for this colour
        mask = cv2.inRange(hsv, self.lower[key], value)
        
        #get h and w of image to do control law
        h, w, d = image.shape
        # get data about mask used in control law
        M = cv2.moments(mask)
        # show mask in window to see what turtlebot sees
        cv2.imshow(key, mask)
        #if area of mask is big enough continue
        if M['m00'] > 5500000:
          #get the x-axis centroid of the mask 
          cx = int(M['m10']/M['m00'])
          
          # if object is not close enough move closer
          if M['m00'] < 10000000:
              # BEGIN Proportional controller
              err = cx - w/2 # calculate error using centroi
              self.twist.linear.x = 0.5 # speed to move
              # correct robot angle using error and constant
              self.twist.angular.z = -float(err) / 180 
              #publish the twist to cmd_vel
              self.cmd_vel_pub.publish(self.twist)
              # END CONTROL
          else: #close enough, announce we found object and remove from search list
              self.found[key] = value
              self.upper.pop(key, None)
              self.lower.pop(key, None)
              self.found_obj_pub.publish(str(self.found))
              rospy.loginfo("%s object has been found!!!", key)
         
    cv2.waitKey(3)

if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        #object_search()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object search node terminated.")
