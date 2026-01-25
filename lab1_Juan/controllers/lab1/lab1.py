# Note I asked chatgpt to help me with some debugging for tunning and such

from controller import Robot

# time in [ms] of a simulation step

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.

robot = Robot()

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# initialize devices
ps = []
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for n in psNames:
    s = robot.getDevice(n)
    s.enable(TIME_STEP)
    ps.append(s)

ls = []
lsNames = ['ls0','ls1','ls2','ls3','ls4','ls5','ls6','ls7']
for n in lsNames:
    s = robot.getDevice(n)
    s.enable(TIME_STEP)
    ls.append(s)

#states
FOLLOW_LEFT = 0
FOLLOW_RIGHT = 1
TURN_RIGHT = 2
TURN_LEFT = 3
TURN_AROUND_180 = 4
STOPPED = 5

state = FOLLOW_LEFT
follow_mode = FOLLOW_LEFT

# wall follow , tune for improved movement
FRONT_THRESHOLD = 95
WALL_DETECT_TH = 55
TARGET = 110
BAND = 25
MISSING_STEPS = 10

BASE_SPEED = 3.0
TURN_SPEED = 2.1
NUDGE = 0.55

left_missing = 0
right_missing = 0

# 180 turn when detecting the light close enough, 22 seems to work the best forn now.
TURN_180_STEPS = 22
turn_180_count = 0
follow_after_180 = FOLLOW_RIGHT

# disable to try movement with not lights
ENABLE_LIGHT = True

# detectors you asked for
DETECT_IDXS = [7, 0]

# for some reason 0 seems to be light 
T_LOW = 5

# confirmation: require many sensors to be low at once
# based on your logs, the real spot often has 6-8 sensors at 0
MIN_LOW_COUNT = 6

LIGHT_CONFIRM_STEPS = 8     # smaller now, because we confirm by low-count too
LIGHT_COOLDOWN_STEPS = 80
light_confirm = 0
light_cooldown = 0

# ------------------ DEBUG ------------------
DEBUG = True
debug_every = 10
step_count = 0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def front_blocked(p):
    return (p[7] > FRONT_THRESHOLD) or (p[0] > FRONT_THRESHOLD)

def left_val(p):
    return max(p[6], p[7])

def right_val(p):
    return max(p[1], p[0])

def left_detected(p):
    return left_val(p) > WALL_DETECT_TH

def right_detected(p):
    return right_val(p) > WALL_DETECT_TH

def light_active(lsVals):
    det_vals = [lsVals[i] for i in DETECT_IDXS]
    detectors_low = all(v <= T_LOW for v in det_vals)

    low_count = sum(1 for v in lsVals if v <= T_LOW)

    # candidate only if detectors are low, being somewhat in the direction of light put sensors to 0
    # we also take into account the back sensors are high and most sensors pereceive to be clsoe to the light.
    active = detectors_low and (low_count >= MIN_LOW_COUNT)
    return active, det_vals, low_count

while robot.step(TIME_STEP) != -1:
    psVals = [ps[i].getValue() for i in range(8)]
    lsVals = [ls[i].getValue() for i in range(8)]

    lv = left_val(psVals)
    rv = right_val(psVals)

    # logic for light
    active, det_vals, low_count = light_active(lsVals)

    if ENABLE_LIGHT and state not in (TURN_AROUND_180, STOPPED):
        if light_cooldown > 0:
            light_cooldown -= 1
            light_confirm = 0
        else:
            if active:
                light_confirm += 1
            else:
                light_confirm = 0

            if light_confirm >= LIGHT_CONFIRM_STEPS:
                if follow_mode == FOLLOW_LEFT:
                    state = TURN_AROUND_180
                    follow_after_180 = FOLLOW_RIGHT
                    turn_180_count = 0
                    light_cooldown = LIGHT_COOLDOWN_STEPS
                    light_confirm = 0
                    if DEBUG:
                        print(f"LIGHT: -> 180  det{DETECT_IDXS}={list(map(int,det_vals))} low_count={low_count}")
                else:
                    state = STOPPED
                    if DEBUG:
                        print(f"LIGHT: -> STOP det{DETECT_IDXS}={list(map(int,det_vals))} low_count={low_count}")

    # missing wall counts , in case the weboot loses the wall it was following, this makes sure it goes around the obstacle.
    if left_detected(psVals):
        left_missing = 0
    else:
        left_missing += 1

    if right_detected(psVals):
        right_missing = 0
    else:
        right_missing += 1

    # state machine
    vL = BASE_SPEED
    vR = BASE_SPEED

    if state == FOLLOW_LEFT:
        follow_mode = FOLLOW_LEFT
        if front_blocked(psVals):
            state = TURN_RIGHT
        elif left_missing >= MISSING_STEPS:
            state = TURN_LEFT
        else:
            if lv > TARGET + BAND:
                vL = BASE_SPEED + NUDGE
                vR = BASE_SPEED - NUDGE
            elif lv < TARGET - BAND:
                vL = BASE_SPEED - NUDGE
                vR = BASE_SPEED + NUDGE

    elif state == FOLLOW_RIGHT:
        follow_mode = FOLLOW_RIGHT
        if front_blocked(psVals):
            state = TURN_LEFT
        elif right_missing >= MISSING_STEPS:
            state = TURN_RIGHT
        else:
            if rv > TARGET + BAND:
                vL = BASE_SPEED - NUDGE
                vR = BASE_SPEED + NUDGE
            elif rv < TARGET - BAND:
                vL = BASE_SPEED + NUDGE
                vR = BASE_SPEED - NUDGE

    elif state == TURN_RIGHT:
        vL = TURN_SPEED
        vR = -TURN_SPEED
        if not front_blocked(psVals):
            state = FOLLOW_LEFT if follow_mode == FOLLOW_LEFT else FOLLOW_RIGHT

    elif state == TURN_LEFT:
        vL = -TURN_SPEED
        vR = TURN_SPEED
        if not front_blocked(psVals):
            state = FOLLOW_LEFT if follow_mode == FOLLOW_LEFT else FOLLOW_RIGHT

    elif state == TURN_AROUND_180:
        vL = TURN_SPEED
        vR = -TURN_SPEED
        turn_180_count += 1
        if turn_180_count >= TURN_180_STEPS:
            state = follow_after_180
            follow_mode = follow_after_180
            left_missing = 0
            right_missing = 0
            if DEBUG:
                print("TURN: 180 COMPLETE -> follow side switched")

    elif state == STOPPED:
        vL = 0.0
        vR = 0.0

    vL = clamp(vL, -MAX_SPEED, MAX_SPEED)
    vR = clamp(vR, -MAX_SPEED, MAX_SPEED)
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    # debugging.
    if DEBUG:
        step_count += 1
        if step_count % debug_every == 0:
            state_name = {
                FOLLOW_LEFT: "FOLLOW_LEFT",
                FOLLOW_RIGHT: "FOLLOW_RIGHT",
                TURN_RIGHT: "TURN_RIGHT",
                TURN_LEFT: "TURN_LEFT",
                TURN_AROUND_180: "TURN_AROUND_180",
                STOPPED: "STOPPED"
            }[state]

            ls_all = [int(v) for v in lsVals]
            print(
                f"STATE={state_name} ENABLE_LIGHT={ENABLE_LIGHT} active={active} conf={light_confirm} cd={light_cooldown} "
                f"ls_all={ls_all} det{DETECT_IDXS}={list(map(int,det_vals))} low_count={low_count} "
                f"ps7,ps0=({psVals[7]:.0f},{psVals[0]:.0f})"
            )

