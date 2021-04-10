#!/usr/bin/env python2

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command
'''
# Importing the required libraries

from vaccine_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """........Essential values and functions are being initialized in this class........"""

    def __init__(self):
        rospy.init_node('attitude_controller')    # Initializing ros node with name attitude_controller

        # This corresponds to the current orientation of eDrone in quaternion format.
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to the current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint, t_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which we want to stabilize the drone
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Gain values for PID algorithm,Left yaw gain as zero for future challenges
        self.Kp = [10, 10.5, 0]
        self.Ki = [0.2, 0.16, 0]
        self.Kd = [450, 437.5, 0]

        # Additional required variables for PID algorithms
        self.error = [0,0,0]    # for proportionality constant
        self.prev_error = [0,0,0]    # To store previous errors
        self.change_error = [0,0,0]    # for differential coefficient
        self.error_sum = [0,0,0]    # Iterm values
        self.pwmThrottle = 0    # altitude value
        self.max_values = [1024, 1024, 1024, 1024]  # Maimum limit of pwm for proppeles
        self.min_values = [0, 0, 0, 0]    # Minimum limit of pwm for proppeles

        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which we run pid.
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error',Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command and /edrone/imu/data
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data
    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle


    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------PID algorithm--------------------------------------------------------------
        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles

        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        self.pwmThrottle = self.setpoint_cmd[3] * 1.023 -1023

        #   3. Compute error in each axis.

        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[1]   #because the order is 
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[0]   #[pitch,roll,yaw]            
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

        #   4. Compute the change in error.

        self.change_error[0] = (self.error[0] - self.prev_error[0])                       
        self.change_error[1] = (self.error[1] - self.prev_error[1])
        self.change_error[2] = (self.error[2] - self.prev_error[2])

        #   5. Calculate the pid output required for each axis.

        self.out_roll = (self.Kp[0] * self.error[0]) + (self.error_sum[0]) + (self.Kd[0] * self.change_error[0])
        self.out_pitch = (self.Kp[1] * self.error[1]) + (self.error_sum[1]) + (self.Kd[1] * self.change_error[1])
        self.out_yaw = (self.Kp[2] * self.error[2]) + (self.error_sum[2]) + (self.Kd[2] * self.change_error[2]) 

        #   6. Computed output value in the equations to compute the pwm for each propeller.....

        self.pwm_cmd.prop1 = self.pwmThrottle - self.out_roll + self.out_pitch - self.out_yaw
        self.pwm_cmd.prop2 = self.pwmThrottle - self.out_roll - self.out_pitch + self.out_yaw
        self.pwm_cmd.prop3 = self.pwmThrottle + self.out_roll - self.out_pitch - self.out_yaw
        self.pwm_cmd.prop4 = self.pwmThrottle + self.out_roll + self.out_pitch + self.out_yaw

        #   8. Limit the output value and the final command value between the maximum(1024) and minimum(0)

        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]

        elif self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]

        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]

        elif self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]

        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]

        elif self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]

        elif self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]

        #   9. Update previous errors

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        #   10. Add error_sum to use for integral component

        self.error_sum[0] = (self.error_sum[0] + self.error[0]) * self.Ki[0]
        self.error_sum[1] = (self.error_sum[1] + self.error[1]) * self.Ki[1]
        self.error_sum[2] = (self.error_sum[2] + self.error[2]) * self.Ki[2]

        #publishing the errors

        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])
        self.yaw_error_pub.publish(self.error[2])

        #publishing the propeller speed
        self.pwm_pub.publish(self.pwm_cmd)



if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)

    while not rospy.is_shutdown():

        try:

            try:

                e_drone.pid()
                r.sleep()
    
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

        except rospy.exceptions.ROSInterruptException:
            pass
