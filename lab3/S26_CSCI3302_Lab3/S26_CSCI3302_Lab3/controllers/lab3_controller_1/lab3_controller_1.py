"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

import math
import numpy as np
from enum import Enum
from controller import Robot, Motor, DistanceSensor



pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
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
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = []
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")


#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  ↓ OUR CODE ↓












































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




#init sensors.
left_wheel_sensor = robot.getDevice("left wheel sensor")
right_wheel_sensor = robot.getDevice("right wheel sensor")
left_wheel_sensor.enable(SIM_TIMESTEP)
right_wheel_sensor.enable(SIM_TIMESTEP)

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
# TODO: set the ePuck wheel speed in m/s after measuring the speed (Part 1)
EPUCK_MAX_WHEEL_SPEED = 0
EPUCK_WHEEL_RADIUS = 0.025
MAX_SPEED = 6.28




angle_of_rotation_left_total = left_wheel_sensor.getValue()  # radians
angle_of_rotation_right_total = right_wheel_sensor.getValue()  # radians

prevleft=0
prevright=0
prevtime=0

diffleft=0
diffright=0

infvelofrotleft=0
infvelofrotright=0

inf_time=0

ldetectioncnt=0






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
        
        print("left_Wheel angle (rad):", angle_of_rotation_left_total)
        print("right_Wheel angle (rad):", angle_of_rotation_right_total)

        print("left_Wheel angle inf (rad):", diffleft)
        print("right_Wheel angle inf (rad):", diffright)

        print("left_Wheel angle velo inf (rad):", infvelofrotleft)
        print("right_Wheel angle velo inf (rad):", infvelofrotright)
        print("Line detected?  " + str(linedetected))
        print("line detected count " + str(ldetectioncnt))
        print("total Robot frame: \n", totalrobotframe)
        print("total I frame: \n", totalIframe)

        print("temp Inverse solved robot frame: \n", tempinvrobotframe)
        #print("full Inverse solved robot frame: \n", invrobotframe)

        print("Theta " + str(theta))
        

        #print("inf_time :", inf_time)

    if(option==1):
         print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
         






def loopclosure2():
     

        global leftsensordetection
        global centersensordetection
        global rightsensordetection
        global start_line_timer
        global start_line_time
        global pose_theta
        global pose_y
        global pose_x
        global linedetected

        if(linedetected):
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            print("LOOP CLOSURE!!! RESETTING POSE")
            pose_x, pose_y, pose_theta=0,0,0
            resetmatricies()









def find_infi_left_angle_rot(totleft):
     global prevleft
     difference=totleft-prevleft
     prevleft=totleft
     return difference

     
     
def find_infi_right_angle_rot(totright):
     global prevright
     difference=totright-prevright
     prevright=totright
     return difference

def find_inf_time(currenttime):
     global prevtime
     difference=currenttime-prevtime
     prevtime=currenttime
     return difference

def calc_velocity(distance,time):
     return (distance/time)
     


def calculate_inf_velo_matrix(rightinf):
     1==1
     





#Odometry
def update_odometry(vL, vR, delta_time):
    global pose_x, pose_y, pose_theta

 
    # normalize and scale with speeds
    vL_mps = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    vR_mps = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED

    #find the distances
    dist_left = vL_mps * delta_time
    dist_right = vR_mps * delta_time

    # find the robot's linear and angular displacement
    dist_center = (dist_left + dist_right) / 2.0
    delta_theta = (dist_right - dist_left) / EPUCK_AXLE_DIAMETER

    # update pose
    pose_x += dist_center * math.cos(pose_theta)
    pose_y += dist_center * math.sin(pose_theta)
    pose_theta += delta_theta







robotframe= np.array([[0],
            [0],
            [0]])


totalrobotframe=np.array([[0],
                 [0],
                [0]])

totalIframe=np.array([[0],
             [0],
             [0]])


tempIframe=np.array([[0],
             [0],
             [0]])


tmatrix=np.array([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]])



tempframe=np.array ([[0],
            [0],
            [0]])







invtmatrix=np.array([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]])



invrobotframe= np.array([[0],
            [0],
            [0]])



tempinvrobotframe= np.array([[0],
            [0],
            [0]])


#top is left,
#bottom is right.
invangleveloframe= np.array([[0],
            [0]])





theta=0


def resetmatricies():
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
    robotframe= np.array([[0],
        [0],
        [0]])


    totalrobotframe=np.array([[0],
                [0],
                [0]])

    totalIframe=np.array([[0],
            [0],
            [0]])


    tempIframe=np.array([[0],
            [0],
            [0]])


    tmatrix=np.array([[0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]])

    tempframe=np.array ([[0],
            [0],
            [0]])


    theta=0





def update_odometry2(infveloleft,infveloright):
     1==1
     global robotframe
     global totalrobotframe
     global totalIframe
     global tempframe
     global tmatrix
     global theta
     global tempIframe
     global pose_x
     global pose_y
     global pose_theta  
     
     #correction factor was calculated like this:
     # pre correction factor reported radians turned after 90 deg = 1.15
     # correct num radians = 1.57= 90 deg
     # 1.57 = x (1.15)
     # x= correctionfactor= 1.57/1.15
     
     correctionfactor=1.37
    


     tempframe=np.array([[(((infveloleft*EPUCK_WHEEL_RADIUS))  + ((infveloright*EPUCK_WHEEL_RADIUS) ))/(2)],
                                                                   [0],
                   [  correctionfactor*math.radians(((infveloright*EPUCK_WHEEL_RADIUS)  - (infveloleft*EPUCK_WHEEL_RADIUS) )/(EPUCK_AXLE_DIAMETER))  ]])





     totalrobotframe=np.add(tempframe,totalrobotframe)

     theta = totalrobotframe[2][0]   

     tmatrix=np.array([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta), 0],
                      [0, 0, 1]])
     

     tempIframe=np.dot(tmatrix,tempframe)

     totalIframe=np.add(totalIframe,tempIframe)
     pose_x, pose_y, pose_theta=totalIframe[0][0] ,totalIframe[1][0] ,theta






def IKrobotsolver():

    #old globals
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
    global pose_x
    global pose_y
    global pose_theta  

    #new globals
    global invtmatrix
    global invrobotframe
    global tempinvrobotframe
    global invangleveloframe


    invtmatrix=np.array([[math.cos(theta), math.sin(theta), 0],
                         [-math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])
    
    tempinvrobotframe=np.dot(invtmatrix,totalIframe)





    
    #doesnt work that way
    #invrobotframe=np.add(tempinvrobotframe,invrobotframe)


    #there is some error when the robot turns 
    #when it comes to the inverse solving, 
    #it falsely solves for some y component of 
    #the robot frame being higher than zero 
    #which is impossile. 

    #this likely stems from the issues we had with the 
    #under-reporting of the angles












def IKanglevelosolver():

    #old globals
    global robotframe
    global totalrobotframe
    global totalIframe
    global tempframe
    global tmatrix
    global theta
    global tempIframe
    global pose_x
    global pose_y
    global pose_theta  

    #new globals
    global invtmatrix
    global invrobotframe
    global tempinvrobotframe
    global invangleveloframe


    invtmatrix=np.array([[math.cos(theta), math.sin(theta), 0],
                         [-math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])
    
    tempinvrobotframe=np.dot(invtmatrix,totalIframe)


    xrvelo=tempinvrobotframe[0][0]
    anglevelo=tempinvrobotframe[2][0]


    rotvelosolver(xrvelo,anglevelo)




def rotvelosolver(xrvelo,anglevelo):

    rotleft=((xrvelo-((anglevelo*EPUCK_AXLE_DIAMETER)/2)))/EPUCK_WHEEL_RADIUS
    rotright=((xrvelo+((anglevelo*EPUCK_AXLE_DIAMETER)/2)))/EPUCK_WHEEL_RADIUS

    invangleveloframe= np.array([[rotleft],
            [rotright]])
    
































# Main Control Loop:
#while robot.step(SIM_TIMESTEP) != -1:


 #   print("test")

    # Set the position of the marker
    #marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
  #  for i, gs in enumerate(ground_sensors):
   #     gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    #pose_x = gps.getValues()[0]
    #pose_y = gps.getValues()[1]
    #pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    ## TODO: controller
    


    #print("Current pose: [%5f, %5f, %5f]" % (xr, yr, theta))
    #leftMotor.setVelocity(vL)
    #rightMotor.setVelocity(vR)





groundthresh=600
groundcount=0
currenttime=0

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    currenttime = robot.getTime()
  
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()


    leftsensordetection=(gsr[0]<groundthresh)
    centersensordetection=(gsr[1]<groundthresh)
    rightsensordetection=(gsr[2]<groundthresh)
    paststart=(not leftsensordetection and not centersensordetection and not rightsensordetection)

    rightcliff=(centersensordetection and not rightsensordetection and leftsensordetection)
    linedetected= ((gsr[0]<groundthresh) and (gsr[2]<groundthresh) and (gsr[1]<groundthresh)) and ((theta==0) or (theta>6))

    if(linedetected):
         ldetectioncnt+=1

    if(robotstate==STATES.speed_measurement):
            1==1


            if(robotsubstate==SUBSTATES.Drive_Forward):
                leftSpeed  =  MAX_SPEED
                rightSpeed = MAX_SPEED
                

                if(linedetected):
                    robotsubstate=SUBSTATES.Stop
                    
                    

            if(robotsubstate==SUBSTATES.Stop):
                leftSpeed  =  0
                rightSpeed = 0
                robotsubstate=SUBSTATES.Calculate_Speed



            if(robotsubstate==SUBSTATES.Calculate_Speed):               
                WHEEL_RADIUS = 0.025
                distance_left = angle_of_rotation_left_total * WHEEL_RADIUS
                print("DISTANCE LEFT " + str(distance_left))

                EPUCK_MAX_WHEEL_SPEED = distance_left / currenttime
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")
                print(f"Calculated Speed: {EPUCK_MAX_WHEEL_SPEED} m/s")

                #todo, calculate linear translation distance and store
                #in the var EPUCK_MAX_WHEEL_SPEED
                #This allows you to utilize speed in m/s for future calculations without measuring wheel diameter.

                robotstate=STATES.line_follower
                robotsubstate=SUBSTATES.Center_Sensor_detects_line
            
            




    if(robotstate==STATES.line_follower):


            #loopclosure()
            loopclosure2()


            if(leftsensordetection):
                 robotsubstate=SUBSTATES.Left_Sensor_detects_line


            if(rightsensordetection):
                 robotsubstate=SUBSTATES.Right_Sensor_detects_line

            if(centersensordetection):
                 robotsubstate=SUBSTATES.Center_Sensor_detects_line



            if(rightcliff):
                 #print("RIGHT CLIFF")
                 robotsubstate=SUBSTATES.Left_Sensor_detects_line


                

            if(robotsubstate==SUBSTATES.Center_Sensor_detects_line):
                leftSpeed  =  MAX_SPEED
                rightSpeed = MAX_SPEED

            else:
                
                rotamt=0.05

                if(robotsubstate==SUBSTATES.Left_Sensor_detects_line):
                    leftSpeed  = -MAX_SPEED*rotamt 
                    rightSpeed = MAX_SPEED*rotamt


                if(robotsubstate==SUBSTATES.Right_Sensor_detects_line):
                    leftSpeed  = MAX_SPEED*rotamt
                    rightSpeed = -MAX_SPEED*rotamt
             
                    
    


    #odometry calculations.
    angle_of_rotation_left_total = left_wheel_sensor.getValue()  # radians
    angle_of_rotation_right_total = right_wheel_sensor.getValue()  # radians
    diffright=find_infi_right_angle_rot(angle_of_rotation_right_total) #radians per step.
    diffleft=find_infi_left_angle_rot(angle_of_rotation_left_total) #radians per step.

    inf_time=find_inf_time(currenttime)


    infvelofrotleft=calc_velocity(diffleft,inf_time)
    infvelofrotright=calc_velocity(diffright,inf_time)

    #see brainstorm doc if confused. 

    delta_time = SIM_TIMESTEP / 1000.0  

    #update_odometry(leftSpeed, rightSpeed, delta_time)

    if(ldetectioncnt):
        update_odometry2(infvelofrotleft,infvelofrotright)
        IKrobotsolver()


    


    report(0,currenttime)





    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

