from XInput import *
import serial
import json
import time

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

# Workaround: bless's BLEAdapter calls win32file.CreateFile on a device path
# that doesn't exist for MediaTek (and some other) Bluetooth adapters.
# BLEAdapter is only needed for set_local_name(), which this script doesn't use.
import bless.backends.winrt.ble.adapter as _bt_adapter
_bt_adapter.BLEAdapter.__init__ = lambda self: None

import asyncio
import logging
import sys
import threading
import json

from typing import Any, Dict, Union

from bless import (  # type: ignore
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(name=__name__)

# Signals a clean Ctrl+C shutdown from outside the coroutine
stop_event = threading.Event()
server = None  # module-level so KeyboardInterrupt handler can access it

# ── Tkinter UI ────────────────────────────────────────────────────────────────

root = tk.Tk()
root.title("XInput → ESP32 BLE Bridge")
canvas = tk.Canvas(root, width=600, height=400, bg="white")
canvas.pack()

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


def get_controller_payload(controller_index=0):
    state = get_state(controller_index)
    if state is None:
        return bytearray(b"{}")

    buttons  = get_button_values(state)   # dict of button_name → bool
    triggers = get_trigger_values(state)  # (left_float, right_float)
    thumbs   = get_thumb_values(state)    # ((lx, ly), (rx, ry))

    payload = {
        "buttons":  buttons,              # {"A": False, "B": True, ...}
        "triggers": list(triggers),       # [left_float, right_float]
        "joysticks": [list(thumbs[0]),    # [[left_x,  left_y],
                      list(thumbs[1])],   #  [right_x, right_y]]
    }
    return bytearray(json.dumps(payload, separators=(",", ":")), "utf-8")


# ── BLE callbacks ─────────────────────────────────────────────────────────────

def on_read(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    """Client pulled the value, return whatever is currently stored."""
    logger.debug(f"Read request → {characteristic.value}")
    return characteristic.value


def on_write(characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
    """Client wrote to us, log it. Extend here to handle commands from ESP32."""
    characteristic.value = value
    logger.debug(f"Write received: {value}")


# ── Main async loop ───────────────────────────────────────────────────────────
SERVICE_UUID = "A07498CA-AD5B-474E-940D-16F1FBE7E8CD"
CHAR_UUID = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B"
POLL_RATE    = 0.008  # seconds between updates

async def run(loop):
    global server
    stop_event.clear()

    gatt: Dict = {
     # Service 1: read/write/indicate characteristic (the main data channel). client (esp32) will receive from this channel
          SERVICE_UUID: { #service UUID
              CHAR_UUID: { #known as characteristic i.e. specific data that the service carries
                "Properties": (
                    GATTCharacteristicProperties.read
                    | GATTCharacteristicProperties.write
                    | GATTCharacteristicProperties.indicate
                ),
                "Permissions": (
                    GATTAttributePermissions.readable
                    | GATTAttributePermissions.writeable
                ),
                "Value": get_controller_payload(),  # initial snapshot
            }
        },
    }

    server = BlessServer(name="ESP32Bridge", loop=loop)
    server.read_request_func = on_read
    server.write_request_func = on_write

    await server.add_gatt(gatt)
    await server.start()
    logger.info(f"Advertising.")

    # ── Continuous loop: poll controller → push BLE → update UI ──────────
    while not stop_event.is_set():
        # 1. Get fresh controller state
        payload = get_controller_payload(0)
        server.get_characteristic(CHAR_UUID).value = payload

        server.update_value(SERVICE_UUID, CHAR_UUID) #updates the buttonstate

        # 4. Update tkinter canvas with latest XInput events
        updateCanvas()
        try:
            root.update()
        except tk.TclError:
            # Window was closed
            stop_event.set()
            break

        await asyncio.sleep(POLL_RATE)

    await server.stop()
    logger.info("Server stopped.")


# ── Entry point ───────────────────────────────────────────────────────────────

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(run(loop))
except KeyboardInterrupt:
    logger.info("Ctrl+C — shutting down...")
    stop_event.set()
    if server is not None:
        loop.run_until_complete(server.stop())
    logger.info("Done.")
finally:
    loop.close()