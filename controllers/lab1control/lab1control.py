from controller import Robot, DistanceSensor, Motor

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



# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    #leftSpeed=0
    #rightSpeed=0

    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())

    print("PS VALUE PRINTOUT:")
    print(psValues)
    print("\n")



    rightsen=80
    leftsen=80

    forwardsen=100
    fcolsen=200



    right_obstacle = psValues[0] > rightsen or psValues[1] > rightsen or psValues[2] > rightsen
    left_obstacle = psValues[5] > leftsen or psValues[6] > leftsen or psValues[7] > leftsen

    #forwards = psValues[0] < forwardsen and psValues[1] < forwardsen and psValues[2] < forwardsen
    forwards= psValues[5] > forwardsen
    fcol=psValues[0] >fcolsen and psValues[7] >fcolsen 


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

   
    if(forwards):
        print("!!!!!!!!!!!!!!!!!!!!MOVING FORWARD!")
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
  
    if(fcol):
        print("!!!!!!!!!!!!!!!!!!!!FRONT COLLISION!")
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
  


   
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

