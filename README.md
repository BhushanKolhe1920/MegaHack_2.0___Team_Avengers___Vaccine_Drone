# MegaHack_2.0___Team_Avengers___Vaccine_Drone
This repository is created for collection of coding files which our team is going to make in Megahack2.0 

Team Number : 32
Team Name : Avengers
Hackathon Name : MegaHack 2.0   St.John College Of Engineering & Management.

### Project/Idea Name : Vaccine Drone  -- Optimized Distribuition of Covid Vaccines to Rural Areas using Drones.

Language that we will use : Python3 & HTML5, CSS
Software we will use : Gazebo simulator, Python IDE.

In this MegaHack2.0, we will be going to design codes for drone in python language so that Drone will be able to carry 
<br> Covid19 Vaccine Package and it will carry that package from one point another point according to GPS location.

                                                          DRONE 
Covid19 Vaccine doses package                      ------------------->               Rural Areas


This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes (when max. of them are used) the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /rpyz_errors            /pid_tuning_altitude
        /edrone/pwm             /pid_tuning_pitch
                                /pid_tuning_roll
                                /edrone/imu/data
                                /edrone/drone_command
                                /pid_tuning_yaw
this script have some changes over previous script
-> angle for edrone is set to -20 to 20 degrees
-> tuning parameteres are changed
-> some code causing extra-overhead is removed





























## Team memebers:

- Bhushan Bhagwat Kolhe
- Harshal Rajesh Rathod
- Cromwell Mendes
- Atharva Bondre 
