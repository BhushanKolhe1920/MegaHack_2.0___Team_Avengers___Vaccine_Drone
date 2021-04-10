#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_controller which controls roll pitch and yaw angles of the eDrone according
to the setpoints This node publishes and subscribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /altitude_error         /edrone/gps   
        /drone_command          /pid_tuning_altitude 
        /logo_altitude          /edrone/range_finder_top
                                /edrone/gps_velocity
                                /edrone/range_finder_bottom 
                                /edrone/errorx   
                                /edrone/errory         
                      
'/edrone/errorx', '/edrone/errory', are received at video fps rate and published by image_processing script 
'/logo_altitude' is Published for image_processing script '''                                                 

# Importing the required libraries
from vaccine_drone.msg import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3Stamped
import rospy
import time
import tf
import csv #for CSV file reading
#from pid_tune.msg import PidTune       # Uncomment when in use 
#import math                            # Uncomment when in Use

# Libraries for Image processing        
#from sensor_msgs.msg import Image      # Uncomment when in Use
#from cv_bridge import CvBridge, CvBridgeError
#import cv2                             # Uncomment when in Use
#import numpy as np                     # Uncomment when in Use
#from pyzbar import pyzbar              # Uncomment in QR detection task
#from pyzbar.pyzbar import decode       # Uncomment in QR detection task
from vaccine_drone.srv import *         
#from matplotlib import pyplot as plt   # Uncomment when in Use

#----------------------------------------------------Edrone_class------------------------------------------------------

class Edrone():
    """docstring for Edrone in Position_controller"""
    def __init__(self):

        rospy.init_node('position_controller', anonymous=True)  # initializing ROS node with name position_controller
       
        # drone_cmd_out contains all the required Values for Publishing drone command.
        self.drone_cmd_out = edrone_cmd()
        self.drone_cmd_out.rcRoll = 1500
        self.drone_cmd_out.rcPitch = 1500
        self.drone_cmd_out.rcYaw = 1500
        self.drone_cmd_out.rcThrottle = 1500

        # Required Constants for PID 
        self.Kp = [21, 21, 170]        
        self.Ki = [0.0, 0.0, 0]          
        self.Kd = [99, 99, 160]        

        # ------------------------------------Other required variables-------------------------------------------------
        #
        self.count = 1               # count variable to stop PID Process if Value of Timechange is not acceptable like 0
        self.stage = 1               # Process stage       
        self.gps_pose = [0, 0, 0] 
        self.set_rcThrottle = 1500
        self.Previous_error = [0, 0, 0]
        self.Current_error = [0, 0, 0]
        self.max_values = [2000, 2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000, 1000]
        self.error_change = [0, 0, 0]
        self.lastTime = 0
        self.now = 0
        self.sample_time = 0.03333333333     # This is the sample time in which We need to run pid in second
        self.error_sum = [ 0 , 0 , 0 ] 
        self.dError = [ 0 , 0 , 0 ]
        self.timeChange = 0.0
        self.stage1_setpoint = [ 0 , 0 , 0]  # these setpoint are different from last setpoints
        self.gripper_check_result = ""      # Uncomment when in use
        self.Gripper_Status = False         # Uncomment when in use
        self.Mission = 1  
        self.BoxDistance = 8
        # -----------------these variables are for pathplanning-------------------
        self.gps_local_setpoint = [ 0 , 0 ]
        self.Obstacle_Distance= [100 , 100 ,100 ,100] # forward backward right left
        self.nearbyanobject = 0
        self.objectnearbyforward = 0
        self.objectnearbybackward = 0
        self.objectnearbyrightward = 0
        self.objectnearbyleftward = 0
        self.previousfunction = ""
        self.velocity = [ 0 , 0 , 0 ] # x   y   z
        self.Previous_velocity_error = [0 , 0 , 0] 
        self.Current_velocity_error = [0 , 0 , 0]
        self.marker_setpoint = [0 ,0 , 0]
        self.safe_height = 25   # It is changing Dynamically 
        self.err_x_m = 0
        self.err_y_m = 0
        self.WareHouseRefrence = [18.9999864489 , 71.9999430161 , 8.44099749139] # Given 
        self.gps_setpoint_Delivery = [ 0 , 0  , 0 ]
        self.gps_setpoint_Picking =  [ 0 ,  0 , 0 ]
        self.TakeOffSetpoints = [ 0 , 0 , 0]
        self.gps_setpoint_coordinate = [ 0 , 0 , 0 ]
        self.gps_pose_coordinate = [ 0 ,  0  , 0 ]
        self.StageJustChanged = False # initialised
        self.TaskJustStarted = True  # initialised
        self.StartingPoint = [ 0 ,0 ,0 ]
        # ----------------------------------------------------------------------------------------------------------
      
    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$-ROS Publishers here-$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd , queue_size = 1)
        self.Mission_pub = rospy.Publisher('/edrone/Mission' ,Int32 ,queue_size = 1)        
        self.marker_data_pub = rospy.Publisher('/edrone/marker_data' , MarkerData , queue_size = 1)
        self.logo_altitude_pub = rospy.Publisher('/logo_altitude' , Float32 , queue_size = 1)
        #-----------------------------------------------------------------------------------------------------------
       
    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$-ROS Subscribers here-$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        rospy.Subscriber('/edrone/gripper_check' ,String ,self.gripper_check_callback) 
        rospy.Subscriber('/edrone/gps', NavSatFix ,self.gps_command_callback) #Gps data Subscription
        rospy.Subscriber('/edrone/range_finder_top' , LaserScan ,self.range_finder_top_callback)
        rospy.Subscriber('/edrone/gps_velocity', Vector3Stamped , self.gps_velocity_callback)
        rospy.Subscriber('/edrone/errorx',Float32,self.errorx_callback)
        rospy.Subscriber('/edrone/errory',Float32,self.errory_callback)
        #rospy.Subscriber('/edrone/range_finder_bottom' , LaserScan ,self.range_finder_bottom_callback) #Uncomment When in Use
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll) #Uncomment When in Use
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch) #Uncomment When in Use
        #-----------------------------------------------------------------------------------------------------------   

    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Required Functions-$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
          
    #--------------------Gps callback function         
    def gps_command_callback(self,msg):
        self.gps_pose[0] = msg.latitude                            
        self.gps_pose[1] = msg.longitude                            
        self.gps_pose[2] = msg.altitude                              

    #---------------------Gripper_check callback Function     ##Uncomment when Gripper is in Use      
    def gripper_check_callback(self,msg):
        self.gripper_check_result = msg.data                         

    # ----------------- latitude/longitude to x y coordinates Converter
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (float(input_latitude) - 19) #110692
    def long_to_y(self, input_longitude):
        return 105292.0089353767 * (float(input_longitude) - 72) 

    #--------------------Laserscan callback Function
    def range_finder_top_callback(self , msg):
        result = msg.ranges
        self.Obstacle_Distance[0] = result[0]
        self.Obstacle_Distance[1] = result[2]
        self.Obstacle_Distance[2] = result[1]
        self.Obstacle_Distance[3] = result[3]

    #--------------------Velocity callback function 
    def gps_velocity_callback(self,msg):
        self.velocity[0] = msg.vector.x
        self.velocity[1] = msg.vector.y
        self.velocity[2] = msg.vector.z

    # ----------------errorx errory callback functions
    def errorx_callback(self,msg):
        self.err_x_m = msg.data
    def errory_callback(self,msg):
        self.err_y_m = msg.data   

    # in range function which check if a variable is in particular limits or not
    def InRange(self,variable,min,max):
        if variable>min and variable< max:
            return True    
        else:
            return False                
     
    #--------------------Pathplanning Function 
    def path_planning(self,TotalDistanceFromDrone):

        self.BoxDistance = 5
    
        if (self.stage == 2 or self.stage == 5) and self.Current_error[0]< 2 and self.Current_error[0] > -2  and self.Current_error[1]< 2 and self.Current_error[1] > -2 :
            
            MaxCost = 1000 # taken according to the Model in which we are Working , can be changed (only increase)
                             
            Forward_Box_Coordinate = [ 0 , 0 ]                                            
            Backward_Box_Coordinate = [ 0 , 0 ]                                          
            Rightward_Box_Coordinate = [ 0 , 0 ]                                          
            Leftward_Box_Coordinate = [ 0 , 0 ]                                           
                                                                                          
            Forward_Box_Coordinate[0] = self.gps_pose_coordinate[0]                      
            Forward_Box_Coordinate[1] = self.gps_pose_coordinate[1]  - self.BoxDistance      
                                                                                          
                                                                                          
            Backward_Box_Coordinate[0] = self.gps_pose_coordinate[0]                      
            Backward_Box_Coordinate[1] = self.gps_pose_coordinate[1] + self.BoxDistance                    
                                                                                         
            Rightward_Box_Coordinate[0] = self.gps_pose_coordinate[0] + self.BoxDistance
            Rightward_Box_Coordinate[1] = self.gps_pose_coordinate[1]
           
            Leftward_Box_Coordinate[0] = self.gps_pose_coordinate[0] - self.BoxDistance
            Leftward_Box_Coordinate[1] = self.gps_pose_coordinate[1]    

            try:
                if self.previousfunction == "leftward":
                    Forward_Box_Coordinate[0] = self.gps_pose_coordinate[0] - 2
                    Backward_Box_Coordinate[0] = self.gps_pose_coordinate[0] - 2

                elif self.previousfunction == "rightward":
                    Forward_Box_Coordinate[0] = self.gps_pose_coordinate[0] + 2
                    Backward_Box_Coordinate[0] = self.gps_pose_coordinate[0] + 2

                elif self.previousfunction == "Forward":
                    Leftward_Box_Coordinate[1] = self.gps_pose_coordinate[1]  - 2
                    Rightward_Box_Coordinate[1] = self.gps_pose_coordinate[1]  - 2   

                elif self.previousfunction == "backward":
                    Leftward_Box_Coordinate[1] = self.gps_pose_coordinate[1]  + 2
                    Rightward_Box_Coordinate[1] = self.gps_pose_coordinate[1]  + 2

            except:
                pass                  
               
         # Calculating cost of every Four Option to go
            Forward_Cost = self.BoxDistance + ((self.gps_setpoint_coordinate[0]- Forward_Box_Coordinate[0])**2 + (self.gps_setpoint_coordinate[1]-Forward_Box_Coordinate[1])**2)**0.5
            Backward_Cost = self.BoxDistance + ((self.gps_setpoint_coordinate[0] - Backward_Box_Coordinate[0])**2 + (self.gps_setpoint_coordinate[1] -Backward_Box_Coordinate[1] )**2)**0.5
            Leftward_Cost = self.BoxDistance + ((self.gps_setpoint_coordinate[0] - Leftward_Box_Coordinate[0])**2 + (self.gps_setpoint_coordinate[1] - Leftward_Box_Coordinate[1])**2)**0.5
            Rightward_Cost = self.BoxDistance + ((self.gps_setpoint_coordinate[0] - Rightward_Box_Coordinate[0])**2 + (self.gps_setpoint_coordinate[1] - Rightward_Box_Coordinate[1])**2)**0.5

       # Whenever there is an obstacle than assigning it a infinite (which is higher than our experimental ranges here is 1000)             
            if self.Obstacle_Distance[0] < 10 and self.Obstacle_Distance[0] > 0.3:
                Forward_Cost = MaxCost 
            if self.Obstacle_Distance[1] < 10 and self.Obstacle_Distance[1] > 0.3:
                Backward_Cost = MaxCost
            if self.Obstacle_Distance[2] < 10 and self.Obstacle_Distance[2] > 0.3:
                Rightward_Cost = MaxCost
            if self.Obstacle_Distance[3] < 10 and self.Obstacle_Distance[3] > 0.3:
                Leftward_Cost = MaxCost
 
       #  one more time not going in that direction in which the drone previously found any object            
            if Forward_Cost == MaxCost or Backward_Cost == MaxCost or Rightward_Cost == MaxCost or Leftward_Cost == MaxCost :
                self.nearbyanobject = 1
                if Forward_Cost == MaxCost:
                    self.objectnearbyforward = 1
                if Backward_Cost == MaxCost:
                    self.objectnearbybackward = 1
                if Leftward_Cost == MaxCost:
                    self.objectnearbyleftward = 1
                if Rightward_Cost == MaxCost:
                    self.objectnearbyrightward = 1  
           
            if self.nearbyanobject == 1 :                                            
                if self.objectnearbyforward == 1 and Forward_Cost != MaxCost:
                    Forward_Cost = MaxCost                   
                    self.objectnearbyforward = 0 
                    self.nearbyanobject = 0                  
                if self.objectnearbybackward == 1 and Backward_Cost != MaxCost:
                    Backward_Cost = MaxCost                   
                    self.objectnearbybackward =0
                    self.nearbyanobject = 0
                if self.objectnearbyrightward == 1 and Rightward_Cost != MaxCost:
                    Rightward_Cost = MaxCost                   
                    self.objectnearbyrightward = 0
                    self.nearbyanobject = 0
                if self.objectnearbyleftward == 1 and Leftward_Cost != MaxCost:
                    Leftward_Cost = MaxCost                  
                    self.objectnearbyleftward = 0
                    self.nearbyanobject = 0
           
         # We do'nt want to 'Reverse' the Step which we Did just previously
            if self.previousfunction == "Forward":
                Backward_Cost = MaxCost
            elif self.previousfunction == "backward":
                Forward_Cost = MaxCost
            elif self.previousfunction == "rightward":
                Leftward_Cost == MaxCost
            elif self.previousfunction == "leftward":
                Rightward_Cost = MaxCost    
            
        # Determining Which path has Minimum Cost
            if Forward_Cost < Backward_Cost and Forward_Cost < Leftward_Cost and Forward_Cost < Rightward_Cost :
                self.previousfunction = "Forward"
                self.gps_local_setpoint[0] = Forward_Box_Coordinate[0]
                self.gps_local_setpoint[1] = Forward_Box_Coordinate[1]

            elif Backward_Cost < Forward_Cost  and Backward_Cost < Leftward_Cost and Backward_Cost < Rightward_Cost :
                self.previousfunction ="backward"
                self.gps_local_setpoint[0] = Backward_Box_Coordinate[0]
                self.gps_local_setpoint[1] = Backward_Box_Coordinate[1]                

            elif Rightward_Cost < Leftward_Cost  and Rightward_Cost < Backward_Cost and Rightward_Cost < Forward_Cost :
                self.previousfunction ="rightward"
                self.gps_local_setpoint[0] = Rightward_Box_Coordinate[0]
                self.gps_local_setpoint[1] = Rightward_Box_Coordinate[1]

            elif Leftward_Cost < Rightward_Cost and Leftward_Cost < Forward_Cost and Leftward_Cost < Backward_Cost :
                self.previousfunction ="leftward" 
                self.gps_local_setpoint[0] = Leftward_Box_Coordinate[0]
                self.gps_local_setpoint[1] = Leftward_Box_Coordinate[1]           

            self.StageJustChanged =  True    

    # ---------------- PID Function------------
    def pid(self):
        
        self.now = rospy.Time.now().to_sec()              # Capturing Time 

      # count variable is used to avoid the very First loop because 'lastTime' is not present at that Time
      # and also this variable is helping us to stop processes which should be run only at very First Startup of Mission
      # in startup of Mission we are forcing timechange to be zero by using lastTime = now in count==1 loop to avoid conditions
      # in Which Rostime does'nt starts from Zero in accordance with the script.
      # shount variable is like count variable
        if self.count == 1: 
            self.lastTime = self.now   
            with open('/home/cromwell/Downloads/manifest.csv', 'r') as file:
                reader = csv.reader(file)
                coordinate = ["nil" , "nil" , "nil"]
                shount  =  1 
                for row in reader:
                    if(shount > self.Mission):
                        print("break called")
                        break           
                    count = 0
                    for column in row:
                        if count == 0:
                            Mission = column
                        if count == 1:
                            coordinate[0] = column
                        if count == 2:
                            coordinate[1] = column
                        if count == 3:
                            coordinate[2] = column                                
                        count = count + 1                     
                    shount = shount +1 
   
            # size of each cell is 1.5m x 1.5m which translates approximately to 0.000013552 in latitude
            # and 0.000014245 in longitude        
            if Mission[0] == "A":
                temp = 1 
            if Mission[0] == "B":
                temp = 2
            if Mission[0] == "C":
                temp = 3
            
            # Formula according to the warehouse structure
            self.gps_setpoint_Picking[0] = self.WareHouseRefrence[0] + (temp - 1)*0.000013552 
            self.gps_setpoint_Picking[1] = self.WareHouseRefrence[1] + (float(Mission[1])- 1 )*0.000014245
            self.gps_setpoint_Picking[2] = self.WareHouseRefrence[2] # 
            self.gps_setpoint_Delivery[0] = float(coordinate[0])  
            self.gps_setpoint_Delivery[1] = float(coordinate[1])
            self.gps_setpoint_Delivery[2] = float(coordinate[2])  

            if shount == self.Mission:
                print("End of Task. Stopping the motors.") 
                #self.gps_setpoint_Picking = self.StartingPoint              
                self.drone_cmd_out.rcThrottle = 1000
                
            
            if self.gps_setpoint_Picking[2] > self.gps_pose[2]:
                self.safe_height = self.gps_setpoint_Picking[2] + 5
            else:
                self.safe_height = self.gps_pose[2] + 5 

            self.TakeOffSetpoints[0] = self.gps_pose[0]
            self.TakeOffSetpoints[1] = self.gps_pose[1]
            self.TakeOffSetpoints[2] = self.safe_height                                  
        
        # if stage is less than 4 than the setpoints should be Picking type
        # else it should be delivery type
        if self.stage < 4:
            self.gps_setpoint = self.gps_setpoint_Picking    
        else:
            self.safe_height = self.gps_setpoint_Delivery[2] + 15                    
            self.gps_setpoint = self.gps_setpoint_Delivery 
                
      # Changing gps setpoint and pose (lat. long.) to x y coordinats in meter
        self.gps_setpoint_coordinate[0] = self.lat_to_x(self.gps_setpoint[0])
        self.gps_setpoint_coordinate[1] = self.long_to_y(self.gps_setpoint[1])
        self.gps_setpoint_coordinate[2] = self.gps_setpoint[2]
        self.gps_pose_coordinate[0] = self.lat_to_x(self.gps_pose[0])
        self.gps_pose_coordinate[1] = self.long_to_y(self.gps_pose[1])
        self.gps_pose_coordinate[2] = self.gps_pose[2]

      # Calculating Total Straight line distance from The End point     
        TotalDistanceFromDrone = ((self.gps_setpoint_coordinate[0]-self.gps_pose_coordinate[0])**2 + (self.gps_setpoint_coordinate[1]-self.gps_pose_coordinate[1])**2)**0.5

      #------------------Path Planning Algorithem
        self.path_planning(TotalDistanceFromDrone)

      # When We are in particular Range like when are total straight line distance from drone is less than the 2^0.5*(boxdistance) 
      # than we want to go to the landing Point (exact point). this code of line fits only like some conditions in which the 
      # destination point does'nt have any obstacle in our code's LSD(min. accuracy limit) range Without this code Works fine
      # but takes some more time to reach destination and to develope an algorithem fast and time budget we have used it 
        if TotalDistanceFromDrone < ((2)**0.5)*self.BoxDistance :
            self.gps_local_setpoint[0] = self.lat_to_x(self.gps_setpoint[0])
            self.gps_local_setpoint[1] = self.long_to_y(self.gps_setpoint[1])  
        
       # sometimes when the script starts stage1_setpoints are not properly loaded . To avoid that we used this conditional BLOCK
        if self.TakeOffSetpoints[0] == 0 and self.TakeOffSetpoints[1] == 0 :
            self.TakeOffSetpoints[0] = self.gps_pose[0]
            self.TakeOffSetpoints[1] = self.gps_pose[1] 
            self.TakeOffSetpoints[2] = self.safe_height

       ## Calculating TimeChange  
        self.timeChange = self.now - self.lastTime
     
       # ---------------Errors Calculation in Different Stages-------------------   
        # TakeOff
        if self.stage == 1:                        
            self.Current_error[0] = (self.lat_to_x(self.TakeOffSetpoints[0]) - self.gps_pose_coordinate[0])    
            self.Current_error[1] = (self.long_to_y(self.TakeOffSetpoints[1]) - self.gps_pose_coordinate[1])
            self.Current_error[2] = self.safe_height - self.gps_pose_coordinate[2]
            
        # Hovering to WareHouse reference where Box is placed 
        elif self.stage == 2:
            self.Current_error[0] =  self.gps_local_setpoint[0] - self.gps_pose_coordinate[0]
            self.Current_error[1] =  self.gps_local_setpoint[1] - self.gps_pose_coordinate[1]                           
            self.Current_error[2] =  self.safe_height - self.gps_pose_coordinate[2]
        
        # landing excatly over the Box . Picking the Box
        elif self.stage == 3:
            self.Current_error[0] =  self.gps_setpoint_coordinate[0] - self.gps_pose_coordinate[0]
            self.Current_error[1] =  self.gps_setpoint_coordinate[1] - self.gps_pose_coordinate[1]
            self.Current_error[2] =  self.gps_setpoint[2]  - self.gps_pose_coordinate[2]                      
        
        # Going up to A particular Safe_height
        elif self.stage == 4:
            self.Current_error[0] =  self.lat_to_x(self.TakeOffSetpoints[0]) - self.gps_pose_coordinate[0]
            self.Current_error[1] =  self.long_to_y(self.TakeOffSetpoints[1]) - self.gps_pose_coordinate[1]
            self.Current_error[2] =  self.safe_height - self.gps_pose_coordinate[2]    

        # Going towards destination Points Horizontally
        elif self.stage == 5:
            self.Current_error[0] =  self.gps_local_setpoint[0] - self.gps_pose_coordinate[0]
            self.Current_error[1] =  self.gps_local_setpoint[1] - self.gps_pose_coordinate[1]                           
            self.Current_error[2] =  self.safe_height - self.gps_pose_coordinate[2]

        # Decrementing Errors W.R.T. Marker
        elif self.stage == 6:
            self.Current_error[0] = self.marker_setpoint[0] - self.gps_pose_coordinate[0]
            self.Current_error[1] = self.marker_setpoint[1] - self.gps_pose_coordinate[1]
            self.Current_error[2] = self.safe_height - self.gps_pose_coordinate[2]

        # Landing on Marker
        elif self.stage == 7:
            self.Current_error[0] = self.marker_setpoint[0] - self.gps_pose_coordinate[0]
            self.Current_error[1] = self.marker_setpoint[1] - self.gps_pose_coordinate[1]
            self.Current_error[2] = self.gps_setpoint[2] - self.gps_pose_coordinate[2]               
                                     
        # ---------------Our PID Algorithm -------------------------------------------------------------
   
        # Calculating Change in Error       
        self.error_change[0] =  (self.Current_error[0] - self.Previous_error[0]) #latitude
        self.error_change[1] =  (self.Current_error[1] - self.Previous_error[1]) #logitude
        self.error_change[2] =  (self.Current_error[2] - self.Previous_error[2]) #altitude

        # Condition to Interrupt Pid and Publishing Process if timeChange is 0
        if self.timeChange != 0:

            if self.TaskJustStarted == True:
                self.StartingPoint[0] = self.gps_pose[0]
                self.StartingPoint[1] = self.gps_pose[1]
                self.StartingPoint[2] = self.gps_pose[2]
                self.TaskJustStarted = False     
                                                 
            # Calculating Sum of Error # Uncomment all integral related lines Only when in Use
            #self.error_sum[0] = self.error_sum[0] + self.Current_error[0]*self.timeChange
            #self.error_sum[1] = self.error_sum[1] + self.Current_error[1]*self.timeChange
            #self.error_sum[2] = self.error_sum[2] + self.Current_error[2]*self.timeChange
       
            # Calculating Difference of error 
            self.dError[0] = self.error_change[0]/self.timeChange
            self.dError[1] = self.error_change[1]/self.timeChange
            self.dError[2] = self.error_change[2]/self.timeChange
           
            # calculation of out_roll,out_pitch and out_throttle
            if  (self.stage != 2 and self.stage !=5) or TotalDistanceFromDrone < ((2)**0.5)*self.BoxDistance : 
                self.out_roll = self.Kp[0]*self.Current_error[0]  + self.Kd[0]*self.dError[0] # + self.Ki[0]*self.error_sum[0]
                self.out_pitch =  self.Kp[1]*self.Current_error[1] + self.Kd[1]*self.dError[1] # + self.Ki[1]*self.error_sum[1]      
                self.out_rcThrottle = self.Kp[2]*self.Current_error[2] + self.Kd[2]*self.dError[2] # + self.Ki[2]*self.error_sum[2]
            
            else:                
                if self.StageJustChanged == True:
                    self.dError = [0 , 0 , 0]
                    self.StageJustChanged = False                
                velocity_setpoint = 3 #3 # tested values suitable here are 1 , 2 , 3
                # PD constants for velocity errors
                kpv = [ 48 , 48 ]  #18 #50 these are tuned values which fits here       
                kdv = [ 32 , 32 ]  #12 #30 these are tuned values which fits here 

                # because we are not using integral in this task we have commented that part      
                if self.previousfunction == "Forward":
                    self.Current_velocity_error[1] = velocity_setpoint - self.velocity[1]
                    self.out_pitch = -(self.Current_velocity_error[1])*kpv[1] - kdv[1]*(self.Current_velocity_error[1]-self.Previous_velocity_error[1])/self.timeChange
                    self.out_roll =  self.Kp[0]*self.Current_error[0]  + self.Kd[0]*self.dError[0] #+ self.Ki[0]*self.error_sum[0]
                    self.Previous_velocity_error[1] = self.Current_velocity_error[1]
               ##-----------------------------------------------------------------------------------------
                if self.previousfunction == "backward":
                    self.Current_velocity_error[1] = (-1)*velocity_setpoint - self.velocity[1]
                    self.out_pitch = -(self.Current_velocity_error[1])*kpv[1] - kdv[1]*(self.Current_velocity_error[1]-self.Previous_velocity_error[1])/self.timeChange
                    self.out_roll = self.Kp[0]*self.Current_error[0]  + self.Kd[0]*self.dError[0] #+ self.Ki[0]*self.error_sum[0]
                    self.Previous_velocity_error[1] = self.Current_velocity_error[1]
               ##-----------------------------------------------------------------------------------------
                if self.previousfunction == "rightward":
                    self.Current_velocity_error[0] = velocity_setpoint - self.velocity[0]
                    self.out_roll = (self.Current_velocity_error[0])*kpv[0] + kdv[0]*(self.Current_velocity_error[0]-self.Previous_velocity_error[0])/self.timeChange
                    self.out_pitch =  self.Kp[1]*self.Current_error[1]  + self.Kd[1]*self.dError[1] #+ self.Ki[1]*self.error_sum[1]
                    self.Previous_velocity_error[0] = self.Current_velocity_error[0]
               ##-----------------------------------------------------------------------------------------
                if self.previousfunction == "leftward":
                    self.Current_velocity_error[0] = (-1)*velocity_setpoint - self.velocity[0]
                    self.out_roll = (self.Current_velocity_error[0])*kpv[0] + kdv[0]*(self.Current_velocity_error[0]-self.Previous_velocity_error[0])/self.timeChange
                    self.out_pitch =  self.Kp[1]*self.Current_error[1]  + self.Kd[1]*self.dError[1] #+ self.Ki[1]*self.error_sum[1]
                    self.Previous_velocity_error[0] = self.Current_velocity_error[0]
               ##-----------------------------------------------------------------------------------------
                 
                self.out_rcThrottle = self.Kp[2]*self.Current_error[2] + self.Kd[2]*self.dError[2] #+ self.Ki[2]*self.error_sum[2]
           
            # Assigning Current Error to previous error        
            self.Previous_error[0] = self.Current_error[0]
            self.Previous_error[1] = self.Current_error[1]
            self.Previous_error[2] = self.Current_error[2]
               
            # Caculation of OUTPUT DRONE Command Which will be given to  Attitude Controller
                
            self.drone_cmd_out.rcRoll = 1500 + self.out_roll
            self.drone_cmd_out.rcPitch = 1500 + self.out_pitch
            self.drone_cmd_out.rcYaw = 1500
            self.drone_cmd_out.rcThrottle =1500 + (self.out_rcThrottle)              
            
            self.lastTime = self.now  # storing This time as lastTime  
           
     # -----------------------------------Filtering acceptable Values for altitude controller----------------------------------

            if self.drone_cmd_out.rcRoll > self.max_values[0]:
                self.drone_cmd_out.rcRoll = 2000
            if self.drone_cmd_out.rcPitch > self.max_values[1]:
            	self.drone_cmd_out.rcPitch = 2000
            if self.drone_cmd_out.rcYaw > self.max_values[2]:
                self.drone_cmd_out.rcYaw = 2000
            if self.drone_cmd_out.rcThrottle > self.max_values[3]:
                self.drone_cmd_out.rcThrottle = 2000          	    	
            if self.drone_cmd_out.rcRoll < self.min_values[0]:
                self.drone_cmd_out.rcRoll = 1000	    	
            if self.drone_cmd_out.rcPitch < self.min_values[1]:
                self.drone_cmd_out.rcPitch = 1000
            if self.drone_cmd_out.rcYaw < self.min_values[2]:
                self.drone_cmd_out.rcYaw = 1000
            if self.drone_cmd_out.rcThrottle < self.min_values[3]:
                self.drone_cmd_out.rcThrottle = 1000
                            
       #------------------------------------------  Publishing Values  ----------------#teamid0654-----------------------------
       
            #self.altitude_error_pub.publish(self.altitude.error) # Uncomment this when You are using Plotjuggler to know altitude error
            self.drone_cmd_pub.publish(self.drone_cmd_out)       # Publishing Setpoint (roll,pitch,yaw and Throttle)  
            self.logo_altitude_pub.publish(self.gps_setpoint[2]) # Publishing logo altitude for image processing calculations   

       # ----------------stage changing conditions ------------------------------
      
        self.count = self.count + 1
        
        # here is latBool is true than latitude error is in 0.05 range
        # visa versa for longitude and altitude respective variables are longBool and altBool
        if self.Current_error[0] < 0.05 and self.Current_error[0] > -0.05 :
            latBool = True
        else:
            latBool = False

        if self.Current_error[1] < 0.05 and self.Current_error[1] > -0.05 :
            longBool = True
        else:
            longBool = False   

        if self.Current_error[2] < 0.05 and self.Current_error[2] > -0.05 :
            altBool = True
        else:
            altBool = False
        
        # Conditions to Go in stage 7
        if self.InRange(self.Current_error[0],-0.2 , 0.2) and self.InRange (self.Current_error[1] ,-0.2 ,0.2) and self.stage == 6:                                                          
            self.stage = 7 
                       
        # Condition to go in stage 6
        elif latBool and longBool and self.stage == 5 and self.Current_error[0] != 0:              
            self.stage = 6      
            self.marker_setpoint[0] =  self.lat_to_x(self.gps_pose[0]) + self.err_x_m
            self.marker_setpoint[1] = self.long_to_y(self.gps_pose[1]) + self.err_y_m 

        # Condition to Go in stage 5
        elif self.Current_error[2] < 0.05 and self.Current_error[2] > -0.05 and self.stage == 4: 
            self.stage = 5             

        # Condition to Go in Stage 4
        elif self.gripper_check_result == "True" and self.Gripper_Status == False and self.stage == 3 :                        
            print(" Gripper aligned ")          
            rospy.wait_for_service('/edrone/activate_gripper')         
            try:           
                Gripper_srv = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
                self.Gripper_Status = Gripper_srv(True)
                print("Box Pickup Status = " + str(self.Gripper_Status))                 
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e) 
            self.stage = 4
            self.TakeOffSetpoints = self.gps_pose 
             
        # Condition to Go in stage 3 
        elif latBool and longBool and self.stage == 2 and self.Current_error[0] != 0 : 
            self.stage = 3    # Filtering if Garbage(0) value is received 
                                      # This is Experimental conclusion made to Fix improper values of Sensors'''
                                                                                                                                    
        # Condition to Go in stage 2
        elif altBool and self.stage == 1: 
            self.stage = 2

        # Condition to Go in Stage 1 
        elif self.stage == 7 and latBool and longBool and self.Current_error[2] < 0.4 and self.Current_error[2] > -0.4 and self.dError[2] <0.1:
            # here we have taken 0.4 because there is an error in CSV altitude (discussed on Piazza)
            # Now problem's solution is that we have takne dError[2] which we have taken in account
            # as only after landing that condition will be true
            # so the real error would be very very less in altitude (~0) >> :) think about it              
            print("gripper detach")
            rospy.wait_for_service('/edrone/activate_gripper')         
            try:           
                Gripper_srv = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
                self.Gripper_Status = Gripper_srv(False)
                print("Box Delivery Status True ")                                            
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)   
            self.Gripper_Status = False   
            self.stage = 1 
            self.count = 1 
            self.Mission = self.Mission + 1                  
            
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specifying rate in Hz based upon sampling time (Frequency = 1/Timeperiod)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()