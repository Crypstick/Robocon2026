from XInput import *
import serial
import json
import ast
import time
import math
import threading
import rerun as rr
import logging
from rerun.encodings import Angle, RotationAxisAngle

# Signals a clean Ctrl+C shutdown from outside
stop_event = threading.Event()

# --- Serial config ---
SERIAL_PORT = "COM9"  
BAUD_RATE   = 115200
POLL_RATE   = 0.02    # set to 50 hz rn, just f = 1/number

# --- Rerun Config ---
rr.init("Robocon2026")
rr.spawn()
startTime = time.time()

# --- Robot Variables ---
MAX_TRANS_SPEED = 8
MAX_ROT_SPEED = 2
KP = 1
KI = 1
KD = 1

controllerConnected = False

set_deadzone(DEADZONE_TRIGGER, 10)


def drawController():
    rr.log("robot/cmd/gate", rr.Ellipses2D(centers=[(0, 0), (20,-20)], half_sizes=[(MAX_TRANS_SPEED, MAX_TRANS_SPEED), (MAX_ROT_SPEED*3, MAX_ROT_SPEED*3)]), static=True)
    rr.log()

def rrRobotLogs(robotTelemetry):
    rr.log("robot/motors/TRodrv1-speed", rr.Scalars(robotTelemetry["odrv1-speed"]))
    rr.log("robot/motors/TLodrv2-speed", rr.Scalars(robotTelemetry["odrv2-speed"]))
    rr.log("robot/motors/BLodrv1-speed", rr.Scalars(robotTelemetry["odrv3-speed"]))
    rr.log("robot/motors/BRodrv1-speed", rr.Scalars(robotTelemetry["odrv4-speed"]))


# in order of indexing
BUTTON_ORDER = [
    "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT",
    "START", "BACK", "LEFT_THUMB", "RIGHT_THUMB",
    "LEFT_SHOULDER", "RIGHT_SHOULDER", "A", "B", "X", "Y",
]


def get_controller_payload(controller_index=0):
    try:
        state = get_state(controller_index)
    except XInputNotConnectedError:
        return -1

    buttons  = get_button_values(state)
    triggers = get_trigger_values(state)
    thumbs   = get_thumb_values(state)

    #  "buttons" is indexed in the same order as BUTTON_ORDER
    #   0 DPAD_UP        8  LEFT_SHOULDER
    #   1 DPAD_DOWN      9  RIGHT_SHOULDER
    #   2 DPAD_LEFT      10 A
    #   3 DPAD_RIGHT     11 B
    #   4 START          12 X
    #   5 BACK           13 Y
    #   6 LEFT_THUMB
    #   7 RIGHT_THUMB
    '''
    dpadUp         = buttons[0]
    dpadDown       = buttons[1]
    dpadLeft       = buttons[2]
    dpadRight      = buttons[3]
    leftShoulder   = buttons[8]
    rightShoulder  = buttons[9]
    buttonA        = buttons[10]
    buttonB        = buttons[11]
    buttonX        = buttons[12]
    buttonY        = buttons[13]
'''
    print(f"buttons were: {thumbs}")

    # joysticks = [[left_x, left_y], [right_x, right_y]]
    #deadzone implementation
    leftX = thumbs[0][0] if abs(thumbs[0][0]) > 0.08 else 0
    leftY = thumbs[0][1] if abs(thumbs[0][1]) > 0.08 else 0
    print()
    rightY = thumbs[1][0] if abs(thumbs[1][0]) > 0.08 else 0


    #omni drive from the latest joystick values
    transVel = round(directionalTrig(leftX, leftY, 45)[0] * MAX_TRANS_SPEED, 3) # translational velocity * speed
    transAngle = round(directionalTrig(leftX, leftY, 45)[1], 3)
    cmdRot = round(rightY * MAX_ROT_SPEED, 3) # rotational velocity * speed


    rr.log("robot/cmd", rr.Transform3D(rotation=rr.RotationAxisAngle(axis=[1, 0, 0], angle=rr.Angle(rad=math.pi)),))
   # rr.log("robot/cmd/transVel", rr.Arrows3D(origins=[0,0,0], vectors=[leftX,leftY,0],radii=0.025))
    rr.log("robot/cmd/transVel", rr.Points2D([[leftX * MAX_TRANS_SPEED, leftY * MAX_TRANS_SPEED]], radii=MAX_TRANS_SPEED * 0.4))
    rr.log("robot/cmd/cmdRot", rr.Points2D([cmdRot*MAX_ROT_SPEED+20,-20],radii=MAX_ROT_SPEED * 0.4 * 2))  
    rr.log("logs", rr.TextLog(f"TransAngle: {transAngle}",level=rr.TextLogLevel.INFO))
    print(f"normalised: {transVel}")
    print(f"angle: {transAngle}")
    print(f"rot vel is: {cmdRot}")
    print(f"expected vel: {transVel * math.sin(transAngle) if (math.sin(transAngle) != 0) else transVel}")

    #send translational velocity and translational angle, and rotation. slave side convert back 

    # rightPower = map((int)((leftY + rightX) * 255), -255, 255)

    button_states = []
    for name in BUTTON_ORDER:
        if buttons[name]:
            button_states.append(1)
        else:
            button_states.append(0)

    payload = {
        "KP": KP,
        "KI": KI,
        "KD": KD,
        "transVel": transVel,
        "transAngle": transAngle,
        "cmdRot": cmdRot,
        # buttons: button_states,
        # "triggers": [round(triggers[0], 3), round(triggers[1], 3)],
    }
        # compact JSON + newline as message delimiter
    return json.dumps(payload, separators=(",", ":")) + "\n"


def normalise(value, min, max):
    return (value-(min))/(max-min)

def directionalTrig(x, y, baseAngle):
    hypotenuse = (x**2 + y**2)**0.5
    try:
        angle = math.acos(y / hypotenuse) #radians unit
    except:
        angle = 0
    if y < 0:
        #angle = 360 - angles
        angle = angle
    if x < 0:
        angle *=-1
    # angle = (angle - 90) % 180
    return (hypotenuse,angle+math.radians(baseAngle))

drawController()

def serial_loop():
    global controllerConnected
    ser = None
    while not stop_event.is_set():
        currentTime = time.time()
        rr.set_time("my_timeline", duration=currentTime - startTime)
        print(f"time: {currentTime - startTime}")
                    
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")  
            except serial.SerialException as e:                                 
                print(f"Serial error: {e}, retry in 2s")
                time.sleep(2)
                continue

        try:
            payload = get_controller_payload(0)
            if payload is not -1:
                ser.write(payload.encode("utf-8"))
                if not controllerConnected:
                    controllerConnected = True

            else:
                print("No controller detected")
                controllerConnected = False
        except serial.SerialException as e:
            print(f"Write error: {e}")
            ser = None

        while ser.in_waiting: 
            robotMessage = ser.readline().decode('utf-8', errors='ignore').strip('\x00\r\n ') #the suffix came from null byte that was sent from C++ (tells system where string ends), and basically full length buffer was sent so it came w it
            try:
                robotDict = json.loads(robotMessage)
            except:
                continue
            prefix = 'odrv'
            try: # 13/9/2026 for some reason it had indexing error so just a precaution
                if robotMessage[0] == "{" and robotMessage[-1] == "}":
                    print(robotDict["odrv1-speed"])
                    rrRobotLogs(robotDict)
                    #print(f"the robot said: {robotMessage}")
                    
            except:
                print("OH MY GOSH DEBUG: ")
                print(robotMessage)



        time.sleep(POLL_RATE)



    if ser and ser.is_open:
        ser.close()

while True:
    serial_loop()