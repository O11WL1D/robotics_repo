"""line_following_with_localizationpy controller."""



import math
import numpy as np
from enum import Enum
from controller import Robot, Motor, DistanceSensor



# -------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = timestep/1000.0    # [s]



# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]


# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]


# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
D = 0.057    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05


# -------------------------------------------------------


# Initialize devices

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []


# motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#######################################################################
# Robot Localization functions
#


def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """Computes speed of the wheels based on encoder readings
    """
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*np.pi*(encoderValues[0] - oldEncoderValues[0])/pulses_per_turn
    ang_diff_r = 2*np.pi*(encoderValues[1] - oldEncoderValues[1])/pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr




def get_robot_speeds(wl, wr, R, D):
    u = R/2.0 * (wr + wl)
    w = R/D * (wr - wl)
    
    return u, w


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    
    update_matrix(u, w, x_old, y_old, phi_old, delta_t)
    
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi

    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y
    


    return x, y, phi

# -------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller








SIM_TIMESTEP = int(robot.getBasicTimeStep())





prevleft=0
prevright=0
prevtime=0

diffleft=0
diffright=0

infvelofrotleft=0
infvelofrotright=0

inf_time=0

ldetectioncnt=0








groundthresh=600
groundcount=0
currenttime=0






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











def report(option, message):
  
    if(option==0):
        print("CURRENT ROBOT STATE:  " + str(robotstate)+ "  CURRENT ROBOT SUBSTATE:    " + str(robotsubstate)) 
        
        print("GROUND SENSOR VALUES: " + str(gsr))
        print("ELAPSED TIME: " + str(currenttime)) 
        print("Left detection? : " + str(leftsensordetection) + " center detection? " + str(centersensordetection) + " right detection? " + str(rightsensordetection)) 
        print(message)
        
        print(
        f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')
   
        print("left_Wheel angle inf (rad):", diffleft)
        print("right_Wheel angle inf (rad):", diffright)

        print("left_Wheel angle velo inf (rad):", infvelofrotleft)
        print("right_Wheel angle velo inf (rad):", infvelofrotright)
        print("Line detected?  " + str(linedetected))
        print("line detected count " + str(ldetectioncnt))
        print("temp Robot frame: \n", tempframe)
        print("total Robot frame: \n", totalrobotframe)
        print("total I frame: \n", totalIframe)

        print(
        f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')
        print("CURRENT U AND W VALUES: " + str(u) + " " + str(w))

        #print("Theta " + str(theta))

        #print("inf_time :", inf_time)

    if(option==1):
           print(
        f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')
      
         
















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

temptheta=0








def update_matrix(u, w, x_old, y_old, phi_old, delta_t):

    global totalIframe

    # --- 1. Compute new heading ---
    phi_new = phi_old + w * delta_t

    if phi_new >= np.pi:
        phi_new -= 2 * np.pi
    elif phi_new < -np.pi:
        phi_new += 2 * np.pi

    # --- 2. Build current world pose matrix T_k ---
    T_k = np.array([
        [np.cos(phi_old), -np.sin(phi_old), x_old],
        [np.sin(phi_old),  np.cos(phi_old), y_old],
        [0,                0,               1]
    ])

    # --- 3. Motion in robot frame (incremental motion) ---
    dT = np.array([
        [np.cos(w * delta_t), -np.sin(w * delta_t), u * delta_t],
        [np.sin(w * delta_t),  np.cos(w * delta_t), 0],
        [0,                    0,                   1]
    ])

    # --- 4. Compose transformations ---
    T_new = np.dot(T_k, dT)

    # --- 5. Extract new pose ---
    x_new = T_new[0, 2]
    y_new = T_new[1, 2]
    phi_new = np.arctan2(T_new[1, 0], T_new[0, 0])

    totalIframe = np.array([
        [x_new],
        [y_new],
        [phi_new]
    ])





















while robot.step(timestep) != -1:

    print("TEST")

    ############################################
    #                  See                     #
    ############################################

    # Update sensor readings
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())



    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]

    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())




    # --------- Robot Localization ------------
    # Using the equations for the robot kinematics based on speed
    # Compute speed of the wheels

    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, 2 * math.pi, delta_t)

    
    # update old encoder values for the next cycle
    oldEncoderValues = encoderValues
    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)

    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    


    #update matricies




    ############################################
    #                 Think                    #
    ############################################





    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()














































    leftsensordetection=(gsr[0]<groundthresh)
    centersensordetection=(gsr[1]<groundthresh)
    rightsensordetection=(gsr[2]<groundthresh)
    paststart=(not leftsensordetection and not centersensordetection and not rightsensordetection)

    rightcliff=(centersensordetection and not rightsensordetection and leftsensordetection)
    linedetected= ((gsr[0]<groundthresh) and (gsr[2]<groundthresh) and (gsr[1]<groundthresh)) 

    offtrack=(not leftsensordetection and not centersensordetection and not rightsensordetection)



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
                

                #todo, calculate linear translation distance and store
                #in the var EPUCK_MAX_WHEEL_SPEED
                #This allows you to utilize speed in m/s for future calculations without measuring wheel diameter.

                robotstate=STATES.line_follower
                robotsubstate=SUBSTATES.Center_Sensor_detects_line
            
            




    if(robotstate==STATES.line_follower):


            #loopclosure()
            #loopclosure2()


            if(leftsensordetection):
                 robotsubstate=SUBSTATES.Left_Sensor_detects_line


            if(rightsensordetection):
                 robotsubstate=SUBSTATES.Right_Sensor_detects_line

            if(centersensordetection):
                 robotsubstate=SUBSTATES.Center_Sensor_detects_line




            if(rightcliff):
                 #print("RIGHT CLIFF")
                 robotsubstate=SUBSTATES.Left_Sensor_detects_line


            if(offtrack):
                robotsubstate=SUBSTATES.Left_Sensor_detects_line



                

            if(robotsubstate==SUBSTATES.Center_Sensor_detects_line):
                leftSpeed  =  MAX_SPEED
                rightSpeed = MAX_SPEED

            else:
                
                rotamt=0.01

                if(robotsubstate==SUBSTATES.Left_Sensor_detects_line):
                    leftSpeed  = -MAX_SPEED*rotamt 
                    rightSpeed = MAX_SPEED*rotamt


                if(robotsubstate==SUBSTATES.Right_Sensor_detects_line):
                    leftSpeed  = MAX_SPEED*rotamt
                    rightSpeed = -MAX_SPEED*rotamt


























    # To help on debugging:

    
    report(0, 0)

    ############################################
    #                  Act                     #
    ############################################

    # Set motor speeds with the values defined by the state-machine
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Repeat all steps while the simulation is running.
