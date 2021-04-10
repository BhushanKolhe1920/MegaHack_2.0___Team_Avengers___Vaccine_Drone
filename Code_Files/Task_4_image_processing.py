#!/usr/bin/env python

'''
image_processing.py Script
This python file runs a ROS-node of name image_processing. it processes the image coming from /edrone/camera/image_raw
This node publishes and subscribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /edrone/errorx          /edrone/gps   
        /edrone/errory          /logo_altitude
                                /edrone/camera/image_raw  
this python script publishes error in x and error in y on '/edrone/errorx' and '/edrone/errory' respectively at video stream 
fps rate. it subscribes '/logo_altitude' topic (published by position controller) to process errors .
in our case logo cascade file path is 'data/cascade.xml'                                   
'''

from vaccine_drone.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import numpy as np
import rospy
#from matplotlib import pyplot as plt
from PIL import Image as im 
import time

class image_proc():

	# Initialise 
	def __init__(self):
            rospy.init_node('image_processing',anonymous=True) #Initialise rosnode 

            # ----------------------ROS subscribers here 
            self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
            rospy.Subscriber('/edrone/gps', NavSatFix ,self.gps_command_callback)
            # on /logo_altitude topic the height of current Mission's edrone_logo is published by position controller
            rospy.Subscriber('/logo_altitude' , Float32 , self.logo_altitude_callback)

            # ---------------------Ros Publishers here
            self.errorx_pub = rospy.Publisher('/edrone/errorx' ,Float32 , queue_size=1)
            self.errory_pub = rospy.Publisher('/edrone/errory' , Float32 , queue_size=1)
            
            # Required arrays and variables for Image 
            self.img = np.empty([]) # This will contain image frame from camera
            self.bridge = CvBridge()

            # ---------------------Other required variables for this task
            self.altitude = 0
            self.bottom_distance = 0 
            self.logo_altitude = 0 

        # ----------------self.logo_altitude_callback    
        def logo_altitude_callback(self , msg):
            self.logo_altitude = msg.data 
        
        # ----------------gps_command-callback function 
        def gps_command_callback(self,msg):
            self.altitude = msg.altitude           

	# Callback function of camera topic
	def image_callback(self, data):
	    try:
                self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image        	
            except CvBridgeError as e:
                print(e)
	        return

        def video_stream(self): 
            # loading 'cascade.xml' for opencv image detection      
            logo_cascade = cv2.CascadeClassifier('data/cascade.xml')           
            while True:
                try:               
                    img = np.uint8(self.img)        
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

                    for (x, y, w, h) in logo:                       
                        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)  
                    cv2.imshow('Frame' , img)  #Uncomment if you want to watch video                   
                    pixel_width_x = 400 # thats equal to pixel_width_y                 
                    focal_length = (pixel_width_x/2)/(0.83909963)  # focal_length for edrone camera                  
                    centre_x = x + w/2  # this is the x coordinte of centre pixel wrt in-image pixel axis                     
                    centre_y = y + h/2  # this is the y coordinte of centre pixel wrt in-image pixel axis

                    z_m  =  self.altitude - self.logo_altitude # height from edrone logo                  
                    # Formula                    
                    err_x_m_out = (centre_x-200)*z_m/focal_length # here 200 is pixel_width_x/2
                    err_y_m_out = (centre_y-200)*z_m/focal_length # here 200 is pixel_width_y/2
                                                                   
                    # publishing @ video fps rate                        
                    self.errorx_pub.publish(err_x_m_out)
                    self.errory_pub.publish(err_y_m_out)  
                                  
                except:
                    pass
      
                if cv2.waitKey(1) & 0xFF == ord('q') :
                    break 
       
            cv2.destroyAllWindows()

if __name__ == '__main__':
    image_proc_obj = image_proc()
    while not rospy.is_shutdown():
        image_proc_obj.video_stream()
    
    
