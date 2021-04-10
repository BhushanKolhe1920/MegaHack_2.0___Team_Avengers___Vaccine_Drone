#!/usr/bin/env python


# Importing the required libraries

from vaccine_drone.msg import *
#from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller', anonymous=True)  # initializing ros node with name attitude_controller

        # This corresponds to current orientation of eDrone in quaternion format. This value is updated each time in imu callback
        # [x,y,z,w] 
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        
        # This corresponds to current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500, 1500, 1500]

        # The setpoint of orientation in euler angles at which we want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0
        
        self.count = 1 # count variable is used to avoid the conditions where ROS time does'nt starts from 0 
        
        #self.error = errors() # roll pitch yaw and zero error for Plotjuggler is contained in self.error
          
        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. 
        # These are Tuned parameters
 
        self.Kp = [7.2, 7.2, 100]     
        self.Ki = [0.0, 0.0, 1.6]       
        self.Kd = [1.8, 1.8, 3.5]             

        # -------------------------------------required variables for pid ----------------------------------------------
        #
        self.set_rcThrottle = 1000
        self.setpoint_rcThrottle = 512 # hay hay change this to 512
        self.Previous_error = [0.0, 0.0, 0.0]
        self.Current_error = [0.0, 0.0, 0.0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]
        self.error_change = [0.0, 0.0, 0.0, 0.0]
        self.lastTime = 0.0
        self.now = 0.0
        self.error_sum = [ 0.0 , 0.0 , 0.0 ] 
        self.dError = [ 0.0 , 0.0 , 0.0 ]
        self.timeChange = 0.0
        self.shouldStop = 0  # aux variable's value comes in this variable     
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which we need to run pid.

        self.sample_time = 0.03  # in seconds
      
        # ------------------------------------ALL ROS Publishers here-----------------------------------------------------

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        # '/rpyz_errors' is a roll pitch yaw and zero error Topic made for publishing errors for Plotjuggler
        # to know more about it Run this command $ rostopic info /rpyz_errors
        # make a msg file in message folder of related package and put a file named errors.msg
        # then make variables (flaot64 - error_roll , error_pitch , error_yaw , error_zero) in that file and also add that in
        # dependencies in  Cmakelist file        
        #self.error_pub = rospy.Publisher('/rpyz_errors', errors, queue_size=1) #Uncomment it when in Use   
                
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)


    # --------------------------------------------Callback functions -----------------------------------------------------
    # This function records data coming from imu sensor
    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w       

    # This function records data coming from Position controller Via '/drone_command' Topic    
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.set_rcThrottle  = msg.rcThrottle
        self.shouldStop = msg.aux1      

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll  /pid_tuning_pitch  /pid_tuning_yaw

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06  
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06 
        self.Ki[1] = pitch.Ki * 0.008           
        self.Kd[1] = pitch.Kd * 0.3

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 10  
        self.Ki[2] = yaw.Ki * 0.08
        self.Kd[2] = yaw.Kd * 0.5

    # -------------------------------------------------- Pid Here--------------------------------------------------------------
    
    def pid(self):

        self.now = rospy.Time.now().to_sec()  # capturing Current time

        if self.count == 1:
            # We have to avoid condition if ROS Time does'nt starts from 0
            self.lastTime = self.now 
            self.count = self.count + 1
        
        self.timeChange = self.now - self.lastTime     # calculating Time change
       
        #  Converting the quaternion format of orientation to euler angles

        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        #  Converting the setpoint that is in the range of 1000 to 2000 into angles with the limit from -20 degree to 20 degree in euler angles and Propellel rcThrottle to 0 to 1024 Range.

        self.setpoint_euler[0] =  (self.setpoint_cmd[0]*0.04) - 60
        self.setpoint_euler[1] =  (self.setpoint_cmd[1]*0.04) - 60 
        self.setpoint_euler[2] =  (self.setpoint_cmd[2]*0.04) - 60  
        self.setpoint_rcThrottle = (self.set_rcThrottle-1000)*1024/1000   

        #   Computing error in each axis.
        # Note: We have used drone_orientation*10 because it ranges around 1 and We need Approx. Upto 10 For proper Use    
        self.Current_error[0] = self.setpoint_euler[0] - (self.drone_orientation_euler[1])*57.324840764 #i have inverted  0 nad 1 here
        self.Current_error[1] = self.setpoint_euler[1] - (self.drone_orientation_euler[0])*57.324840764 #remove 10 if causing error
        self.Current_error[2] = self.setpoint_euler[2] - (self.drone_orientation_euler[2])*57.324840764
        
        # Assigning Current error in roll pitch and yaw and Zero error to Publish For plotjuggler
        # Uncomment these lines when You are using Plotjuggler
        #self.error.error_roll = self.Current_error[0]
        #self.error.error_pitch = self.Current_error[1]
        #self.error.error_yaw = self.Current_error[2]
        #self.error.error_zero = 0
       
        if self.timeChange != 0:      # Condition for not to Run PID main Algorithem when timeChange is not acceptable

        # Computing the Current_error (for proportional), error_change (for derivative) and error_sum (for integral) in each axis.

            self.error_change[0] =  self.Current_error[0] - self.Previous_error[0] 
            self.error_change[1] =  self.Current_error[1] - self.Previous_error[1] 
            self.error_change[2] =  self.Current_error[2] - self.Previous_error[2]
 
            self.error_sum[0] = self.error_sum[0] + self.Current_error[0]*self.timeChange
            self.error_sum[1] = self.error_sum[1] + self.Current_error[1]*self.timeChange
            self.error_sum[2] = self.error_sum[2] + self.Current_error[2]*self.timeChange

        # Calculting rate of change in error and assigning it in dError

            self.dError[0] = self.error_change[0]/self.timeChange
            self.dError[1] = self.error_change[1]/self.timeChange
            self.dError[2] = self.error_change[2]/self.timeChange

        # Using Current_error , error_sum and dError to calculate suitable roll pitch and yaw

            self.out_roll = self.Kp[0]*self.Current_error[0] + self.Ki[0]*self.error_sum[0] + self.Kd[0]*self.dError[0]
            self.out_pitch =  self.Kp[1]*self.Current_error[1] + self.Ki[1]*self.error_sum[1] + self.Kd[1]*self.dError[1]
            self.out_yaw =  self.Kp[2]*self.Current_error[2] + self.Ki[2]*self.error_sum[2] + self.Kd[2]*self.dError[2]
        
        #   Using this computed output value in the equations to compute the pwm for each propeller.

            self.pwm_cmd.prop1 = self.setpoint_rcThrottle - self.out_roll + self.out_pitch - self.out_yaw
            self.pwm_cmd.prop2 = self.setpoint_rcThrottle - self.out_roll - self.out_pitch + self.out_yaw
            self.pwm_cmd.prop3 = self.setpoint_rcThrottle + self.out_roll - self.out_pitch - self.out_yaw 
            self.pwm_cmd.prop4 = self.setpoint_rcThrottle + self.out_roll + self.out_pitch + self.out_yaw

           # Uncomment these line when aux -> shouldStop variable is in use to stop the drone at last point
           # if  self.shouldStop ==  1 :
             #  self.pwm_cmd.prop1 = 0
             #  self.pwm_cmd.prop2 = 0
             #  self.pwm_cmd.prop3 = 0
             #  self.pwm_cmd.prop4 = 0
                
        #  Limiting the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. 

            if self.pwm_cmd.prop1 > self.max_values[0]:
                self.pwm_cmd.prop1 = 1024
            if self.pwm_cmd.prop2 > self.max_values[1]:
                self.pwm_cmd.prop2 = 1024
            if self.pwm_cmd.prop3 > self.max_values[2]:
                self.pwm_cmd.prop3 = 1024
            if self.pwm_cmd.prop4 > self.max_values[3]:
                self.pwm_cmd.prop4 = 1024
            if self.pwm_cmd.prop1 < self.min_values[0]:
                self.pwm_cmd.prop1 = 0
            if self.pwm_cmd.prop2 < self.min_values[1]:
                self.pwm_cmd.prop2 = 0
            if self.pwm_cmd.prop3 < self.min_values[2]:
                self.pwm_cmd.prop3 = 0
            if self.pwm_cmd.prop4 < self.min_values[3]:
                self.pwm_cmd.prop4 = 0 

        # Capturing This time as last_time

            self.lastTime = self.now 

        # Assigning Current error in Previous Error to use in Future Loop

            self.Previous_error[0] = self.Current_error[0]
            self.Previous_error[1] = self.Current_error[1]
            self.Previous_error[2] = self.Current_error[2]
       
        # Publishing final values         
            #self.error_pub.publish(self.error) #Uncomment when in Use publishing errors for Plotjuggler on '/rpyz_errors' Topic             
            self.pwm_pub.publish(self.pwm_cmd)
             

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specifying rate in Hz , 30Hz in our Case
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
        