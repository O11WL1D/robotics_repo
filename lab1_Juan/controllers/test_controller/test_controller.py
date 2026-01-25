from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

robot = Robot()

# init proximity sensors
ps = []
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# init light sensors
ls = []
lsNames = ['ls0','ls1','ls2','ls3','ls4','ls5','ls6','ls7']
for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

# init motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# keyboard (keep it; optional manual test still works)
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# -------- states --------
FOLLOW_LEFT = 0
FOLLOW_RIGHT = 1
TURN_RIGHT = 2
TURN_LEFT = 3
TURN_AROUND_180 = 4
STOPPED = 5

state = FOLLOW_LEFT
follow_mode = FOLLOW_LEFT

# -------- thresholds / tuning --------
FRONT_THRESHOLD = 95
WALL_DETECT_TH = 55
TARGET = 110
BAND = 25
MISSING_STEPS = 10

# -------- speeds --------
BASE_SPEED = 3.0
TURN_SPEED = 2.1
NUDGE = 0.55

# wall-missing counters
left_missing_count = 0
right_missing_count = 0

# -------- 180 turn (time-based) --------
TURN_180_STEPS = 22
turn_180_count = 0
follow_mode_after_180 = FOLLOW_RIGHT

# -------- light detection tuning --------
# IMPORTANT: In many Webots e-puck worlds, light sensors can be:
# - higher value when brighter OR lower value when brighter (depends on setup)
# So we keep a configurable mode here.

LIGHT_HIGHER_IS_BRIGHTER = True   # if triggers feel backwards, flip to False

LIGHT_THRESHOLD = 3500            # <-- you will tune this (see debug print)
LIGHT_CONFIRM_STEPS = 6          # must see light condition for N steps
LIGHT_COOLDOWN_STEPS = 35        # ignore light for a bit after a 180

light_confirm_count = 0
light_cooldown = 0

# debug
DEBUG = True
debug_every = 10
step_count = 0

# -------- helper funcs --------
def front_blocked(p):
    # true front corners: ps7 (front-left), ps0 (front-right)
    return (p[7] > FRONT_THRESHOLD) or (p[0] > FRONT_THRESHOLD)

def left_val(p):
    # LEFT follow uses: ps6 (left) + ps7 (front-left)
    return max(p[6], p[7])

def right_val(p):
    # RIGHT follow uses true mirror: ps1 (right) + ps0 (front-right)
    return max(p[1], p[0])

def left_detected(p):
    return left_val(p) > WALL_DETECT_TH

def right_detected(p):
    return right_val(p) > WALL_DETECT_TH

def light_value(lsVals, idxs):
    # use average of chosen sensors to reduce noise
    return sum(lsVals[i] for i in idxs) / float(len(idxs))

def light_active(lsVals):
    # sensors to check: ls0, ls1, ls2, ls5, ls6, ls7
    idxs = [0, 1, 2, 5, 6, 7]
    avg = light_value(lsVals, idxs)

    if LIGHT_HIGHER_IS_BRIGHTER:
        return avg >= LIGHT_THRESHOLD, avg
    else:
        return avg <= LIGHT_THRESHOLD, avg


while robot.step(TIME_STEP) != -1:
    psVals = [ps[i].getValue() for i in range(8)]
    lsVals = [ls[i].getValue() for i in range(8)]

    lv = left_val(psVals)
    rv = right_val(psVals)

    # ---- optional manual 180 with T (still useful for testing) ----
    key = keyboard.getKey()
    if key == ord('T') and state not in (TURN_AROUND_180, STOPPED):
        follow_mode_after_180 = FOLLOW_RIGHT if follow_mode == FOLLOW_LEFT else FOLLOW_LEFT
        state = TURN_AROUND_180
        turn_180_count = 0
        light_cooldown = LIGHT_COOLDOWN_STEPS
        light_confirm_count = 0
        if DEBUG:
            print("MANUAL: 180 TURN TRIGGERED (T) -> will switch follow side")

    # ---- light detection (only when not stopped/turning) ----
    if state not in (TURN_AROUND_180, STOPPED):
        if light_cooldown > 0:
            light_cooldown -= 1
            light_confirm_count = 0
        else:
            active, avg_light = light_active(lsVals)
            if active:
                light_confirm_count += 1
            else:
                light_confirm_count = 0

            if light_confirm_count >= LIGHT_CONFIRM_STEPS:
                # first time: do 180 and switch to right-follow
                if follow_mode == FOLLOW_LEFT:
                    follow_mode_after_180 = FOLLOW_RIGHT
                    state = TURN_AROUND_180
                    turn_180_count = 0
                    light_cooldown = LIGHT_COOLDOWN_STEPS
                    light_confirm_count = 0
                    if DEBUG:
                        print(f"LIGHT: detected -> 180 + switch to FOLLOW_RIGHT (avg={avg_light:.1f})")

                # second time: stop
                elif follow_mode == FOLLOW_RIGHT:
                    state = STOPPED
                    if DEBUG:
                        print(f"LIGHT: detected again -> STOP (avg={avg_light:.1f})")

    # hysteresis counters for wall detection
    if left_detected(psVals):
        left_missing_count = 0
    else:
        left_missing_count += 1

    if right_detected(psVals):
        right_missing_count = 0
    else:
        right_missing_count += 1

    vL = BASE_SPEED
    vR = BASE_SPEED

    # -------------- FSM --------------
    if state == FOLLOW_LEFT:
        follow_mode = FOLLOW_LEFT

        if front_blocked(psVals):
            state = TURN_RIGHT
        elif left_missing_count >= MISSING_STEPS:
            state = TURN_LEFT
        else:
            if lv > TARGET + BAND:
                vL = BASE_SPEED + NUDGE
                vR = BASE_SPEED - NUDGE
            elif lv < TARGET - BAND:
                vL = BASE_SPEED - NUDGE
                vR = BASE_SPEED + NUDGE
            else:
                vL = BASE_SPEED
                vR = BASE_SPEED

    elif state == FOLLOW_RIGHT:
        follow_mode = FOLLOW_RIGHT

        if front_blocked(psVals):
            state = TURN_LEFT
        elif right_missing_count >= MISSING_STEPS:
            state = TURN_RIGHT
        else:
            if rv > TARGET + BAND:
                vL = BASE_SPEED - NUDGE
                vR = BASE_SPEED + NUDGE
            elif rv < TARGET - BAND:
                vL = BASE_SPEED + NUDGE
                vR = BASE_SPEED - NUDGE
            else:
                vL = BASE_SPEED
                vR = BASE_SPEED

    elif state == TURN_RIGHT:
        vL = TURN_SPEED
        vR = -TURN_SPEED
        if not front_blocked(psVals):
            if follow_mode == FOLLOW_LEFT and left_detected(psVals):
                state = FOLLOW_LEFT
            elif follow_mode == FOLLOW_RIGHT and right_detected(psVals):
                state = FOLLOW_RIGHT

    elif state == TURN_LEFT:
        vL = -TURN_SPEED
        vR = TURN_SPEED
        if not front_blocked(psVals):
            if follow_mode == FOLLOW_LEFT and left_detected(psVals):
                state = FOLLOW_LEFT
            elif follow_mode == FOLLOW_RIGHT and right_detected(psVals):
                state = FOLLOW_RIGHT

    elif state == TURN_AROUND_180:
        # spin in place (right spin) for a fixed number of steps
        vL = TURN_SPEED
        vR = -TURN_SPEED
        turn_180_count += 1

        if turn_180_count >= TURN_180_STEPS:
            state = follow_mode_after_180
            follow_mode = follow_mode_after_180
            left_missing_count = 0
            right_missing_count = 0
            if DEBUG:
                print("TURN: 180 COMPLETE -> switched follow side")

    elif state == STOPPED:
        vL = 0.0
        vR = 0.0

    # clamp + apply
    vL = max(-MAX_SPEED, min(MAX_SPEED, vL))
    vR = max(-MAX_SPEED, min(MAX_SPEED, vR))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    # debug
    if DEBUG:
        step_count += 1
        if step_count % debug_every == 0:
            active, avg_light = light_active(lsVals)
            state_name = {
                FOLLOW_LEFT: "FOLLOW_LEFT",
                FOLLOW_RIGHT: "FOLLOW_RIGHT",
                TURN_RIGHT: "TURN_RIGHT",
                TURN_LEFT: "TURN_LEFT",
                TURN_AROUND_180: "TURN_AROUND_180",
                STOPPED: "STOPPED"
            }[state]
            print(
                f"STATE={state_name}  "
                f"front(ps7,ps0)=({psVals[7]:.0f},{psVals[0]:.0f})  "
                f"lv={lv:.0f} missL={left_missing_count}  "
                f"rv={rv:.0f} missR={right_missing_count}  "
                f"light_avg={avg_light:.1f} active={active} conf={light_confirm_count} cd={light_cooldown}  "
                f"t180={turn_180_count}"
            )


