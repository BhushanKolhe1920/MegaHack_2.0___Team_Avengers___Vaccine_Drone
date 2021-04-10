
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
        rospy.Subscriber('/edrone/imu/data', 
