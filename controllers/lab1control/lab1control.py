from controller import Robot, DistanceSensor, Motor
from enum import Enum

class STATES(Enum):
    STABLIZE=7
    MOVING=8
    REVERSE=9
    RSTABLIZE=10


class SUBSTATES(Enum):
    NEARWALL=1
    MIDROTATE=2



 





# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

ls = []
lsNames = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']

for i in range(len(lsNames)):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)


leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(2)
rightMotor.setVelocity(2)

leftSpeed=0
rightSpeed=0

robotstate=STATES.RSTABLIZE


prevhighest=0

stablecount=0

stabilizecounter=0

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs


    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())

    print("PS VALUE PRINTOUT:")
    print(psValues)
    print("\n")



  

    leftsen=100
    frontsen=200
    cfrontsen=210

    ldiagsen=60
    bdiagsen=70

    cornersen=80
    rcornersen=70

    targetdist=230

    #right_obstacle = psValues[0] > rightsen or psValues[1] > rightsen or psValues[2] > rightsen
    #left_obstacle = psValues[5] > leftsen or psValues[6] > leftsen or psValues[7] > leftsen

    #forwards = psValues[0] < forwardsen and psValues[1] < forwardsen and psValues[2] < forwardsen

    #targetmet=psValues[5] > targetdist
    

    #if three left sensors are active, then activate leftsensor
    
    # LEFTSIDE NEAR WALL MOVEMENT.
    leftsensoractive= psValues[5] > leftsen and psValues[4] > ldiagsen and psValues[6] > ldiagsen
    frontsensoractive=psValues[0] >frontsen and psValues[7] >frontsen 
    fdiagsensoractive=psValues[6] >frontsen
    leftcorner=psValues[5] >cornersen or psValues[6] >cornersen
  
    

   
    # RIGHTSIDE NEAR WALL MOVEMENT.
    Rleftsensoractive= psValues[2] > leftsen and psValues[1] > ldiagsen and psValues[3] > ldiagsen
    Rfrontsensoractive=psValues[0] >frontsen and psValues[7] >frontsen 
    Rfdiagsensoractive=psValues[1] >frontsen
    rightcorner=psValues[2] >rcornersen or psValues[3] >rcornersen
  



 

    if(robotstate==STATES.STABLIZE):
        stablecount+=1

        print("STABILIZING!!!!!!")
       
        print("PREV HIGHEST!" + str(prevhighest))
        if(psValues[5]>prevhighest):
            print("HIGHER VALUE")
            highestvalue=psValues[5]
            leftSpeed  = 0.001 * MAX_SPEED
            rightSpeed = -0.001 * MAX_SPEED

        else:
             leftSpeed  = 0
             rightSpeed = 0
        

        prevhighest=highestvalue


        if(stablecount==10):
            if(prevhighest-psValues[5] > 10):
                leftSpeed  = -0.1 * MAX_SPEED
                rightSpeed = 0.1 * MAX_SPEED
            else:
                stablecount=0
                prevhighest=0
                robotstate=STATES.MOVING



    if(robotstate==STATES.RSTABLIZE):
        stablecount+=1

        print(" R STABILIZING!!!!!!")
       
        print("PREV HIGHEST!" + str(prevhighest))
        if(psValues[2]>prevhighest):
            print("HIGHER VALUE")
            highestvalue=psValues[2]
            leftSpeed  = -0.001 * MAX_SPEED
            rightSpeed = 0.001 * MAX_SPEED

        else:
             leftSpeed  = 0
             rightSpeed = 0
        

        prevhighest=highestvalue


        if(stablecount==10):
            if(prevhighest-psValues[2] > 10):
                leftSpeed  = 0.1 * MAX_SPEED
                rightSpeed = -0.1 * MAX_SPEED
            else:
                stablecount=0
                prevhighest=0
                robotstate=STATES.REVERSE







    if(robotstate==STATES.MOVING):
        print("STANDARD MOVEMENT ENABLED!")
        stabilizecounter+=1
        

        #if(abs(rotationdif)>10):
        #    robotstate=STATES.STABLIZE
        

        if(stabilizecounter==19):
            robotstate=STATES.STABLIZE
            

    
        if(leftsensoractive):
           print("LEFT SENSOR ACTIVE")
           
           leftSpeed  = 0.5 * MAX_SPEED
           rightSpeed = 0.5 * MAX_SPEED
        
        else:

                    leftSpeed  = -0.5 * MAX_SPEED
                    rightSpeed = 0.5 * MAX_SPEED
                    if(leftcorner):
                        leftSpeed  = 0.5 * MAX_SPEED
                        rightSpeed = 0.5 * MAX_SPEED








        if(fdiagsensoractive):
             leftSpeed  = 0.5 * MAX_SPEED
             rightSpeed = -0.5 * MAX_SPEED



        if(frontsensoractive):
             leftSpeed  = 0.5 * MAX_SPEED
             rightSpeed = -0.5 * MAX_SPEED


    if(robotstate==STATES.REVERSE):
        print("REVERSE ACTIVE!")


        stabilizecounter+=1
                

        if(stabilizecounter==18):
            robotstate=STATES.RSTABLIZE
            

    
        if(Rleftsensoractive):
            print("LEFT SENSOR ACTIVE")
            
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            
        else:
                leftSpeed  = 0.5 * MAX_SPEED
                rightSpeed = -0.5 * MAX_SPEED

                if(rightcorner):
                    #leftSpeed  = 0.5 * MAX_SPEED
                    #rightSpeed = 0.5 * MAX_SPEED
                    print("")








        if(Rfdiagsensoractive):
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED



        if(Rfrontsensoractive):
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED




 




    """
    if left_obstacle:
        print("!!!!!!!!!!!!!!!!!!!LEFT OBSTACLE DETECTED!")
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED

    elif right_obstacle:
        print("!!!!!!!!!!!!!!!!!!!RIGHT OBSTACLE DETECTED!")
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    """

   

    




   
    """
    if(forwardsnf):
        print("!!!!!!!!!!!!!!!!!!!!OVERSHOOT")
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    """



    # write actuators inputs


    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)





    

    #leftMotor.setPosition(float('inf'))
    #rightMotor.setPosition(float('inf'))

