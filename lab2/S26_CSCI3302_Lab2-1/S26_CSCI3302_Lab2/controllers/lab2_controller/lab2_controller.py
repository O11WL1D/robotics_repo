"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from enum import Enum
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Set a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 0

class STATES(Enum):
    speed_measurement=1
    line_follower=2



class SUBSTATES(Enum):
    Drive_Forward=1
    Start_Line_Detection=2
    Stop=3
    Calculate_Speed=4
    #line follower state sub states
    Center_Sensor_detects_line=5
    Left_Sensor_detects_line=6
    Right_Sensor_detects_line=7
    No_Sensors_Detect_Line=8


robotstate=STATES.speed_measurement
robotsubstate=SUBSTATES.Drive_Forward


# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
# TODO: set the ePuck wheel speed in m/s after measuring the speed (Part 1)
EPUCK_MAX_WHEEL_SPEED = 0
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize variable for left and right speed
vL = 0
vR = 0


# robot output function, please try to have all output go in here 
# so that it can be customized. 
def report(option, message):
    if(option==0):
        print("CURRENT ROBOT STATE:  " + str(robotstate)+ "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate)) 
        print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
        print("GROUND SENSOR VALUES: " + str(gsr))
        print("ELAPSED TIME: " + str(currenttime)) 
        print("Left detection? : " + str(leftsensordetection) + " center detection? " + str(centersensordetection) + " right detection? " + str(rightsensordetection)) 
        print(message)


groundthresh=300
groundcount=0
currenttime=0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

  
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()


    leftsensordetection=(gsr[0]<groundthresh)
    centersensordetection=(gsr[1]<groundthresh)
    rightsensordetection=(gsr[1]<groundthresh)
    paststart=(not leftsensordetection and not centersensordetection and not rightsensordetection)

    rightcliff=(centersensordetection and rightsensordetection and not leftsensordetection)

    if(robotstate==STATES.speed_measurement):
            1==1


            if(robotsubstate==SUBSTATES.Drive_Forward):
                leftSpeed  =  MAX_SPEED
                rightSpeed = MAX_SPEED
                

                if((gsr[0]<groundthresh) or (gsr[2]<groundthresh)):
                    robotsubstate=SUBSTATES.Stop
                    
                    

            if(robotsubstate==SUBSTATES.Stop):
                leftSpeed  =  0
                rightSpeed = 0
                robotsubstate=SUBSTATES.Calculate_Speed



            if(robotsubstate==SUBSTATES.Calculate_Speed):
                currenttime = robot.getTime()

                #todo, calculate linear translation distance and store
                #in the var EPUCK_MAX_WHEEL_SPEED
                #This allows you to utilize speed in m/s for future calculations without measuring wheel diameter.

                robotstate=STATES.line_follower
                robotsubstate=SUBSTATES.Center_Sensor_detects_line
            
            


    if(robotstate==STATES.line_follower):

            

             #detect conditions               


            if(leftsensordetection):
                 robotsubstate=SUBSTATES.Left_Sensor_detects_line


            if(rightsensordetection):
                 robotsubstate=SUBSTATES.Right_Sensor_detects_line

            if(centersensordetection):
                 robotsubstate=SUBSTATES.Center_Sensor_detects_line



            if(rightcliff):
                 print("RIGHT CLIFF")
                 robotsubstate=SUBSTATES.Right_Sensor_detects_line


                

            if(robotsubstate==SUBSTATES.Center_Sensor_detects_line):
                leftSpeed  =  MAX_SPEED
                rightSpeed = MAX_SPEED

            else:
                
                rotamt=0.001

                if(robotsubstate==SUBSTATES.Left_Sensor_detects_line):
                    leftSpeed  =  MAX_SPEED*rotamt 
                    rightSpeed = -MAX_SPEED*rotamt

                if(robotsubstate==SUBSTATES.Right_Sensor_detects_line):
                    leftSpeed  = -MAX_SPEED*rotamt
                    rightSpeed = MAX_SPEED*rotamt



        
    report(0,currenttime)



    # TODO: Uncomment to see the ground sensor values!
    # TODO: But when you don't need it, please comment it so you have a clean terminal.
    

    # Part 1
    # TODO: Implement Maximum Speed Measurement under state "speed_measurement"
    # TODO: Save the speed within XZ-plane to EPUCK_MAX_WHEEL_SPEED after measuring it.

    # Part 2
    # TODO: Implement Line Following under state "line_follower"
    # TODO: Also implement update_odometry and then call update_odometry here
    # Hints for Line Following:
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    # Hints for update_odometry:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    # Part 3
    # TODO: Implement Loop Closure also under state "line_follower" to reset pose when robot passes over the Start Line.
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last
    # for best results


    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)



