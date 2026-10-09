from XInput import *
import serial
import json
import time
import threading
import math

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

# Signals a clean Ctrl+C shutdown from outside
stop_event = threading.Event()

# --- Serial config ---
SERIAL_PORT = "COM11"   # <-- change to your ESP32 port
BAUD_RATE   = 115200
POLL_RATE   = 0.02    # seconds between sends (20 Hz)

# --- Tkinter UI ---
root = tk.Tk()
root.title("XInput tp ESP32 Serial Bridge")
canvas = tk.Canvas(root, width=600, height=400, bg="white")
canvas.pack()

# Status label at the bottom
status_var = tk.StringVar(value="Connecting to serial...")
status_label = tk.Label(root, textvariable=status_var, fg="gray")
status_label.pack()

# --- Robot Variables ---
MAX_TRANS_SPEED = 8
MAX_ROT_SPEED = 2
KP = 1
KI = 1
KD = 1

set_deadzone(DEADZONE_TRIGGER, 10)

class Controller:
    def __init__(self, center):
        self.center = center

        self.on_indicator_pos = (self.center[0], self.center[1] - 50)
        self.on_indicator = canvas.create_oval(
            (self.on_indicator_pos[0] - 10, self.on_indicator_pos[1] - 10,
             self.on_indicator_pos[0] + 10, self.on_indicator_pos[1] + 10))

        self.r_thumb_pos = (self.center[0] + 50, self.center[1] + 20)
        canvas.create_oval(
            (self.r_thumb_pos[0] - 25, self.r_thumb_pos[1] - 25,
             self.r_thumb_pos[0] + 25, self.r_thumb_pos[1] + 25))
        self.r_thumb_stick = canvas.create_oval(
            (self.r_thumb_pos[0] - 10, self.r_thumb_pos[1] - 10,
             self.r_thumb_pos[0] + 10, self.r_thumb_pos[1] + 10))

        self.l_thumb_pos = (self.center[0] - 100, self.center[1] - 20)
        canvas.create_oval(
            (self.l_thumb_pos[0] - 25, self.l_thumb_pos[1] - 25,
             self.l_thumb_pos[0] + 25, self.l_thumb_pos[1] + 25))
        self.l_thumb_stick = canvas.create_oval(
            (self.l_thumb_pos[0] - 10, self.l_thumb_pos[1] - 10,
             self.l_thumb_pos[0] + 10, self.l_thumb_pos[1] + 10))

        self.l_trigger_pos = (self.center[0] - 120, self.center[1] - 70)
        canvas.create_rectangle(
            (self.l_trigger_pos[0] - 5, self.l_trigger_pos[1] - 20,
             self.l_trigger_pos[0] + 5, self.l_trigger_pos[1] + 20))
        self.l_trigger_index = canvas.create_rectangle(
            (self.l_trigger_pos[0] - 10, self.l_trigger_pos[1] - 25,
             self.l_trigger_pos[0] + 10, self.l_trigger_pos[1] - 15))

        self.r_trigger_pos = (self.center[0] + 120, self.center[1] - 70)
        canvas.create_rectangle(
            (self.r_trigger_pos[0] - 5, self.r_trigger_pos[1] - 20,
             self.r_trigger_pos[0] + 5, self.r_trigger_pos[1] + 20))
        self.r_trigger_index = canvas.create_rectangle(
            (self.r_trigger_pos[0] - 10, self.r_trigger_pos[1] - 25,
             self.r_trigger_pos[0] + 10, self.r_trigger_pos[1] - 15))

        buttons_pos = (self.center[0] + 100, self.center[1] - 20)
        self.A_button = canvas.create_oval(
            (buttons_pos[0] - 10, buttons_pos[1] + 10,
             buttons_pos[0] + 10, buttons_pos[1] + 30))
        self.B_button = canvas.create_oval(
            (buttons_pos[0] + 10, buttons_pos[1] - 10,
             buttons_pos[0] + 30, buttons_pos[1] + 10))
        self.Y_button = canvas.create_oval(
            (buttons_pos[0] - 10, buttons_pos[1] - 30,
             buttons_pos[0] + 10, buttons_pos[1] - 10))
        self.X_button = canvas.create_oval(
            (buttons_pos[0] - 30, buttons_pos[1] - 10,
             buttons_pos[0] - 10, buttons_pos[1] + 10))

        dpad_pos = (self.center[0] - 50, self.center[1] + 20)
        self.dpad_left  = canvas.create_rectangle(
            (dpad_pos[0] - 30, dpad_pos[1] - 10, dpad_pos[0] - 10, dpad_pos[1] + 10), outline="")
        self.dpad_up    = canvas.create_rectangle(
            (dpad_pos[0] - 10, dpad_pos[1] - 30, dpad_pos[0] + 10, dpad_pos[1] - 10), outline="")
        self.dpad_right = canvas.create_rectangle(
            (dpad_pos[0] + 10, dpad_pos[1] - 10, dpad_pos[0] + 30, dpad_pos[1] + 10), outline="")
        self.dpad_down  = canvas.create_rectangle(
            (dpad_pos[0] - 10, dpad_pos[1] + 10, dpad_pos[0] + 10, dpad_pos[1] + 30), outline="")
        canvas.create_polygon(
            (dpad_pos[0]-30, dpad_pos[1]-10), (dpad_pos[0]-10, dpad_pos[1]-10),
            (dpad_pos[0]-10, dpad_pos[1]-30), (dpad_pos[0]+10, dpad_pos[1]-30),
            (dpad_pos[0]+10, dpad_pos[1]-10), (dpad_pos[0]+30, dpad_pos[1]-10),
            (dpad_pos[0]+30, dpad_pos[1]+10), (dpad_pos[0]+10, dpad_pos[1]+10),
            (dpad_pos[0]+10, dpad_pos[1]+30), (dpad_pos[0]-10, dpad_pos[1]+30),
            (dpad_pos[0]-10, dpad_pos[1]+10), (dpad_pos[0]-30, dpad_pos[1]+10),
            fill="", outline="black")

        self.back_button  = canvas.create_oval(
            (self.center[0]-25, self.center[1]-25, self.center[0]-15, self.center[1]-15))
        self.start_button = canvas.create_oval(
            (self.center[0]+15, self.center[1]-25, self.center[0]+25, self.center[1]-15))
        self.l_shoulder = canvas.create_rectangle(
            (self.center[0]-110, self.center[1]-75, self.center[0]-70, self.center[1]-65))
        self.r_shoulder = canvas.create_rectangle(
            (self.center[0]+70,  self.center[1]-75, self.center[0]+110, self.center[1]-65))


controllers = (
    Controller((150., 100.)),
    Controller((450., 100.)),
    Controller((150., 300.)),
    Controller((450., 300.)),
)


def updateCanvas():
    # Poll XInput events and update the tkinter canvas to reflect controller state.
    for event in get_events():
        c = controllers[event.user_index]

        if event.type == EVENT_CONNECTED:
            canvas.itemconfig(c.on_indicator, fill="light green")

        elif event.type == EVENT_DISCONNECTED:
            canvas.itemconfig(c.on_indicator, fill="")

        elif event.type == EVENT_STICK_MOVED:
            if event.stick == LEFT:
                pos = (int(round(c.l_thumb_pos[0] + 25 * event.x)),
                       int(round(c.l_thumb_pos[1] - 25 * event.y)))
                canvas.coords(c.l_thumb_stick,
                    pos[0]-10, pos[1]-10, pos[0]+10, pos[1]+10)
            elif event.stick == RIGHT:
                pos = (int(round(c.r_thumb_pos[0] + 25 * event.x)),
                       int(round(c.r_thumb_pos[1] - 25 * event.y)))
                canvas.coords(c.r_thumb_stick,
                    pos[0]-10, pos[1]-10, pos[0]+10, pos[1]+10)

        elif event.type == EVENT_TRIGGER_MOVED:
            if event.trigger == LEFT:
                y = c.l_trigger_pos[1] - 20 + int(round(40 * event.value))
                canvas.coords(c.l_trigger_index,
                    c.l_trigger_pos[0]-10, y-5, c.l_trigger_pos[0]+10, y+5)
            elif event.trigger == RIGHT:
                y = c.r_trigger_pos[1] - 20 + int(round(40 * event.value))
                canvas.coords(c.r_trigger_index,
                    c.r_trigger_pos[0]-10, y-5, c.r_trigger_pos[0]+10, y+5)

        elif event.type == EVENT_BUTTON_PRESSED:
            btn_map = {
                "LEFT_THUMB": c.l_thumb_stick, "RIGHT_THUMB": c.r_thumb_stick,
                "LEFT_SHOULDER": c.l_shoulder,  "RIGHT_SHOULDER": c.r_shoulder,
                "BACK": c.back_button, "START": c.start_button,
                "DPAD_LEFT": c.dpad_left, "DPAD_RIGHT": c.dpad_right,
                "DPAD_UP": c.dpad_up,   "DPAD_DOWN": c.dpad_down,
                "A": c.A_button, "B": c.B_button, "Y": c.Y_button, "X": c.X_button,
            }
            if event.button in btn_map:
                canvas.itemconfig(btn_map[event.button], fill="red")

        elif event.type == EVENT_BUTTON_RELEASED:
            btn_map = {
                "LEFT_THUMB": c.l_thumb_stick, "RIGHT_THUMB": c.r_thumb_stick,
                "LEFT_SHOULDER": c.l_shoulder,  "RIGHT_SHOULDER": c.r_shoulder,
                "BACK": c.back_button, "START": c.start_button,
                "DPAD_LEFT": c.dpad_left, "DPAD_RIGHT": c.dpad_right,
                "DPAD_UP": c.dpad_up,   "DPAD_DOWN": c.dpad_down,
                "A": c.A_button, "B": c.B_button, "Y": c.Y_button, "X": c.X_button,
            }
            if event.button in btn_map:
                canvas.itemconfig(btn_map[event.button], fill="")


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

    #  "buttons" is indexed in the same order as esp32_controller_ESPNOW.py's BUTTON_ORDER
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
    rightY = thumbs[1][0] if abs(thumbs[1][0]) > 0.08 else 0


    #omni drive from the latest joystick values
    transVel = round(directionalTrig(leftX, leftY,45)[0] * MAX_TRANS_SPEED, 3) # translational velocity * speed
    transAngle = round(directionalTrig(leftX, leftY,45)[1], 3)
    cmdRot = round(rightY * MAX_ROT_SPEED, 3) # rotational velocity * speed

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
        # "joysticks": [
        #     [round(thumbs[0][0], 3), round(thumbs[0][1], 3)],
        #     [round(thumbs[1][0], 3), round(thumbs[1][1], 3)],
        # ],
    }
    



    # compact JSON + newline as message delimiter
    return json.dumps(payload, separators=(",", ":")) + "\n"


# gotta define these cos if i call directly it crashes tkinter loop LOL
def apply_status(msg):
    status_var.set(msg)


def set_status(msg):
    root.after(0, apply_status, msg)

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


def serial_loop():
    ser = None
    while not stop_event.is_set():
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                set_status(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")  
            except serial.SerialException as e:                                 
                set_status(f"Serial error: {e}, retry in 2s")
                time.sleep(2)
                continue

        try:
            payload = get_controller_payload(0)
            if payload is not -1:

                ser.write(payload.encode("utf-8"))
            else:
                set_status("No controller detected")
        except serial.SerialException as e:
            set_status(f"Write error: {e}")
            ser = None

        while ser.in_waiting: 
            robotMessage = (ser.readline().strip()).decode('utf-8')
            prefix = 'speed'
            try: # 13/9/2026 for some reason it had indexing error so just a precaution
                if robotMessage[0] == "{" and robotMessage[-1] == "}":
                    print(f"the robot said: {robotMessage}")
                    print(f"first is: {robotMessage[0]}")
            except:
                print("OH MY GOSH DEBUG: ")
                print(robotMessage)

        time.sleep(POLL_RATE)



    if ser and ser.is_open:
        ser.close()



serial_thread = threading.Thread(target=serial_loop, daemon=True)
serial_thread.start()


# ── Tkinter main loop ──────────────────────────────────────────────────────────

def tk_loop():
    try:
        updateCanvas()
        root.after(int(POLL_RATE * 1000), tk_loop)
    except tk.TclError:
        stop_event.set()

root.after(0, tk_loop)

try:
    root.mainloop()
finally:
    stop_event.set()
    serial_thread.join(timeout=2)
