from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64
MAX_SPEED = 6.28
ZERO_THRESHOLD = 50 

#Wall following thresholds
leftsen = 100
frontsen = 200
ldiagsen = 60
cornersen = 80
rcornersen = 80

robot = Robot()

# Initialize Sensors
ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for s in ps: s.enable(TIME_STEP)

ls = [robot.getDevice(f'ls{i}') for i in range(8)]
for s in ls: s.enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#STATES 0,1,2,3
state = 0
turn_timer = 0
light_cooldown = 0
startup_wait = 15

while robot.step(TIME_STEP) != -1:
    if startup_wait > 0:
        startup_wait -= 1
        continue

    psValues = [s.getValue() for s in ps]
    lsValues = [s.getValue() for s in ls]
    
  
 
    target_indices = [0, 1, 2, 5, 6, 7]
    is_seeing_light = all(lsValues[i] < ZERO_THRESHOLD for i in target_indices)

    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    #STATES
    if state == 0:  # FOLLOW LEFT WALL
        # Original movement logic
        leftsensoractive = psValues[5] > leftsen and psValues[4] > ldiagsen and psValues[6] > ldiagsen
        frontsensoractive = psValues[0] > frontsen and psValues[7] > frontsen
        fdiagsensoractive = psValues[6] > frontsen
        leftcorner = psValues[5] > cornersen or psValues[6] > cornersen
        
        if leftsensoractive:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        else:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            if leftcorner:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
        
        if fdiagsensoractive or frontsensoractive:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED

        # Trigger 180 
        if is_seeing_light:
            state = 1
            turn_timer = 28

    elif state == 1:  # 180 DEGREE TURN
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        turn_timer -= 1
        if turn_timer <= 0:
            state = 2
            light_cooldown = 100 

    elif state == 2:  # FOLLOW RIGHT WALL
        Rleftsensoractive = psValues[2] > leftsen and psValues[1] > ldiagsen and psValues[3] > ldiagsen
        Rfrontsensoractive = psValues[0] > frontsen and psValues[7] > frontsen
        Rfdiagsensoractive = psValues[1] > frontsen
        rightcorner = psValues[1] > rcornersen or psValues[2] > rcornersen
        
        if Rleftsensoractive:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        else:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
            if rightcorner:
                leftSpeed = 0.5 * MAX_SPEED
                rightSpeed = 0.5 * MAX_SPEED
        
        if Rfdiagsensoractive or Rfrontsensoractive:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED

        if light_cooldown > 0:
            light_cooldown -= 1
        elif is_seeing_light:
            state = 3

    elif state == 3:  # STOP
        leftSpeed = 0
        rightSpeed = 0

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)