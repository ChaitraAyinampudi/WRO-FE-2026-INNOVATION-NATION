import os
import sys
import time
import signal
from pathlib import Path
from typing import Tuple
from sense_hat import SenseHat
from time import sleep

import donkeycar as dk
import numpy as np
import pygame
from dynamixel_sdk import *
from donkeycar.parts.transform import Lambda
from donkeycar.parts.tub_v2 import TubWriter, TubWiper

try:
    from donkeycar.parts.keras import KerasInterpreter, KerasLinear
except ImportError:
    sys.exit("Keras/TensorFlow not available - install DonkeyCar with AI support.")

FREERUN_MODEL_PATH_CW    = "~/WRO_FE_2026/models/fcwm/fcwm001-006/mypilot.h5"
FREERUN_MODEL_PATH_CCW   = "~/WRO_FE_2026/models/fccwm/fccwm001-006/mypilot.h5"
OBSTACLE_MODEL_PATH_CW   = "~/WRO_FE_2026/models/ocwm/ocwm001-008/mypilot.h5"
OBSTACLE_MODEL_PATH_CCW  = "~/WRO_FE_2026/models/occwm/occwm001-010/mypilot.h5"

sense = SenseHat()
sense.set_rotation(180)
sense.low_light = True
sense.clear()

def flash(msg, seconds=0.7):
    sense.show_message(msg, scroll_speed=0.10, text_colour=[255, 255, 255])
    sleep(seconds)

sense.clear()

DRIVE_MODE = "FCW"
flash(DRIVE_MODE)
print("Move joystick, press middle to confirm.")
while True:
    for ev in sense.stick.get_events():
        if ev.action != "pressed":
            continue
        if ev.direction == "left":
            DRIVE_MODE = "FCW";  flash("FCW")
        elif ev.direction == "right":
            DRIVE_MODE = "FCCW"; flash("FCCW")
        elif ev.direction == "up":
            DRIVE_MODE = "OCW";  flash("OCW")
        elif ev.direction == "down":
            DRIVE_MODE = "OCCW"; flash("OCCW")
        elif ev.direction == "middle":
            flash(DRIVE_MODE)
            sense.clear()
            print("Selection finished.")
            break
    else:
        continue
    break

if DRIVE_MODE == "FCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(FREERUN_MODEL_PATH_CW)
elif DRIVE_MODE == "FCCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(FREERUN_MODEL_PATH_CCW)
elif DRIVE_MODE == "OCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(OBSTACLE_MODEL_PATH_CW)
elif DRIVE_MODE == "OCCW":
    MODEL_PATH_DEFAULT = os.path.expanduser(OBSTACLE_MODEL_PATH_CCW)
else:
    sys.exit("Invalid DRIVE_MODE")

print("Model:", MODEL_PATH_DEFAULT)

CAPTURE_W, CAPTURE_H = (176, 132)
CROP_W, CROP_H       = (160, 120)

DATA_PATH          = os.path.expanduser("~/WRO_FE_2026/data")

DRIVE_LOOP_HZ      = 20
JOYSTICK_DEADZONE  = 0.05
RECORD_THRESHOLD   = 0.05
MAX_SPEED_PERCENT  = 25
STEERING_MAX_SPEED = 100
angle_offset       = 0.7

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
]
TUB_TYPES  = ["image_array", "float", "float", "str"]

DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 57600
DXL_PROTOCOL_VERSION = 2.0

DXL_STEER_ID = 2
DXL_THROTTLE_IDS = [1]

DXL_THROTTLE_DIRECTIONS = [1]

DXL_STEER_CENTER_TICKS = 3126
DXL_STEER_LEFT_DEG = -60
DXL_STEER_RIGHT_DEG = 60
DXL_STEER_DIRECTION = 1
DXL_TICKS_PER_DEG = 4096 / 360

DXL_VELOCITY_UNIT_RPM = 0.229
DXL_THROTTLE_MAX_RPM = 100
DXL_STEER_PROFILE_ACCEL = 200
DXL_THROTTLE_PROFILE_ACCEL = 200

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

MODE_VELOCITY = 1
MODE_POSITION = 3
TORQUE_OFF = 0
TORQUE_ON = 1

def _to_uint32(value: int) -> int:
    return int(value) & 0xFFFFFFFF

def _rpm_to_velocity_lsb(rpm: float) -> int:
    return int(round(float(rpm) / DXL_VELOCITY_UNIT_RPM))

class DynamixelBus:
    def __init__(self, port=DXL_PORT, baudrate=DXL_BAUDRATE):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.ref_count = 0
        self.closed = False

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open DYNAMIXEL port: {port}")
        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set DYNAMIXEL baudrate: {baudrate}")

        print(f"DYNAMIXEL bus opened on {port} at {baudrate} baud")

    def register_part(self):
        self.ref_count += 1

    def release_part(self):
        self.ref_count = max(0, self.ref_count - 1)
        if self.ref_count == 0:
            self.close()

    def _check(self, dxl_id, result, error, action):
        if result != COMM_SUCCESS:
            print(f"[DYNAMIXEL ID {dxl_id}] {action} failed: "
                  f"{self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            print(f"[DYNAMIXEL ID {dxl_id}] {action} error: "
                  f"{self.packet_handler.getRxPacketError(error)}")
            return False
        return True

    def write1(self, dxl_id, addr, value, action="write1"):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, addr, int(value)
        )
        return self._check(dxl_id, result, error, action)

    def write4(self, dxl_id, addr, value, action="write4"):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, addr, _to_uint32(value)
        )
        return self._check(dxl_id, result, error, action)

    def configure_motor(self, dxl_id, mode, profile_accel=200, profile_velocity=None):

        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "torque off")
        time.sleep(0.02)
        self.write1(dxl_id, ADDR_OPERATING_MODE, mode, "set operating mode")
        time.sleep(0.02)
        self.write4(dxl_id, ADDR_PROFILE_ACCEL, profile_accel, "set profile accel")
        if profile_velocity is not None:
            self.write4(dxl_id, ADDR_PROFILE_VELOCITY, profile_velocity,
                        "set profile velocity")
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque on")
        time.sleep(0.02)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.port_handler.closePort()
            print("DYNAMIXEL bus closed")
        except Exception:
            pass

_DXL_BUS = None

def get_dxl_bus() -> DynamixelBus:
    global _DXL_BUS
    if _DXL_BUS is None or _DXL_BUS.closed:
        _DXL_BUS = DynamixelBus()
    return _DXL_BUS

CAMERA_FRAMERATE             = 30
PICAMERA_AWB_MODE            = 'off'
PICAMERA_EXPOSURE_MODE       = 'off'
PICAMERA_ISO                 = 100
PICAMERA_SHUTTER_SPEED       = 15000
PICAMERA_AWB_GAINS           = (1.5, 1.2)
PICAMERA_EXPOSURE_COMPENSATION = 0

def center_crop(img, tw=CROP_W, th=CROP_H):
    h, w = img.shape[:2]
    x0 = (w - tw) // 2
    y0 = (h - th) // 2 - 6
    return img[y0:y0 + th, x0:x0 + tw]

class DynamixelSteering:
    def __init__(self, motor_id=DXL_STEER_ID,
                 left=DXL_STEER_LEFT_DEG,
                 right=DXL_STEER_RIGHT_DEG):
        self.bus = get_dxl_bus()
        self.bus.register_part()
        self.id = int(motor_id)
        self.left = float(left)
        self.right = float(right)
        self.prev_goal = None
        self.closed = False

        steer_profile_velocity = _rpm_to_velocity_lsb(STEERING_MAX_SPEED)
        self.bus.configure_motor(
            self.id,
            MODE_POSITION,
            profile_accel=DXL_STEER_PROFILE_ACCEL,
            profile_velocity=steer_profile_velocity
        )
        time.sleep(0.1)
        self._write_goal(DXL_STEER_CENTER_TICKS)

    def _angle_to_goal_ticks(self, angle: float) -> int:
        angle = angle * angle_offset
        angle = max(min(angle, 1.0), -1.0)

        steer_deg = self.left + (angle + 1) * (self.right - self.left) / 2
        goal = DXL_STEER_CENTER_TICKS + DXL_STEER_DIRECTION * steer_deg * DXL_TICKS_PER_DEG

        return int(max(0, min(4095, round(goal))))

    def _write_goal(self, goal_ticks: int):
        if goal_ticks == self.prev_goal:
            return
        self.bus.write4(self.id, ADDR_GOAL_POSITION, goal_ticks, "set steering goal")
        self.prev_goal = goal_ticks

    def run(self, angle: float):
        goal_ticks = self._angle_to_goal_ticks(angle)
        self._write_goal(goal_ticks)

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._write_goal(DXL_STEER_CENTER_TICKS)
            time.sleep(0.25)
            self.bus.write1(self.id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steering torque off")
        finally:
            self.bus.release_part()

class DynamixelThrottle:
    def __init__(self, motor_ids=DXL_THROTTLE_IDS, max_speed=MAX_SPEED_PERCENT):
        self.bus = get_dxl_bus()
        self.bus.register_part()
        self.ids = [int(i) for i in motor_ids]
        self.max_speed = int(max(min(max_speed, 100), 0))
        self.last_speed = None
        self.closed = False

        for dxl_id in self.ids:
            self.bus.configure_motor(
                dxl_id,
                MODE_VELOCITY,
                profile_accel=DXL_THROTTLE_PROFILE_ACCEL,
                profile_velocity=None
            )
        self._stop()

    def _direction_for_index(self, i: int) -> int:
        if i < len(DXL_THROTTLE_DIRECTIONS):
            return -1 if DXL_THROTTLE_DIRECTIONS[i] < 0 else 1
        return 1

    def _stop(self):
        print('run stop!')
        for dxl_id in self.ids:
            self.bus.write4(dxl_id, ADDR_GOAL_VELOCITY, 0, "stop throttle")

    def run(self, throttle: float):
        throttle = max(min(throttle, 1.0), -1.0)
        speed = int(throttle * self.max_speed)
        speed = int(round(speed / 10.0) * 10)

        if self.last_speed == 0 and speed == 0:
            return

        if self.last_speed != 0 and speed == 0:
            self._stop()
            self.last_speed = speed
            return

        if speed != 0 and speed == self.last_speed:
            return

        if speed != 0 and speed != self.last_speed:
            rpm = DXL_THROTTLE_MAX_RPM * (speed / 100.0)
            velocity_lsb = _rpm_to_velocity_lsb(rpm)
            for i, dxl_id in enumerate(self.ids):
                goal_velocity = velocity_lsb * self._direction_for_index(i)
                self.bus.write4(dxl_id, ADDR_GOAL_VELOCITY, goal_velocity,
                                "set throttle velocity")

        self.last_speed = speed

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._stop()
            time.sleep(0.05)
            for dxl_id in self.ids:
                self.bus.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF,
                                "throttle torque off")
        finally:
            self.bus.release_part()

class PS4Joystick:
    def __init__(self, deadzone=JOYSTICK_DEADZONE):
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected.")
        self.js = pygame.joystick.Joystick(0); self.js.init()
        self.dz = deadzone
        print(f"Connected joystick: {self.js.get_name()}")

    def _dz(self, v):
        return 0.0 if abs(v) < self.dz else float(v)

    def run(self) -> Tuple[float, float, str]:
        pygame.event.pump()
        angle = self._dz(self.js.get_axis(0))
        throttle = -self._dz(self.js.get_axis(4))

        if self.js.get_button(2):
            return angle, throttle, "erase"

        if self.js.get_button(4):
            raise KeyboardInterrupt

        return angle, throttle, "user"

    def shutdown(self):
        pygame.quit()

class ConsoleDisplay:
    def __init__(self):
        self.last_t = 0
    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 1.0:
            print(f"Pred -> angle {angle:+.2f}  thr {throttle:+.2f}")
            self.last_t = t

def add_camera(car: dk.vehicle.Vehicle):
    from donkeycar.parts.camera import PiCamera

    cam = PiCamera(
        image_w = CAPTURE_W,
        image_h = CAPTURE_H,
        image_d = 3
    )

    time.sleep(2)

    cam.camera.framerate           = CAMERA_FRAMERATE
    cam.camera.exposure_mode       = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode            = PICAMERA_AWB_MODE
    cam.camera.iso                 = PICAMERA_ISO
    cam.camera.shutter_speed       = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains           = PICAMERA_AWB_GAINS

    car.add(cam, outputs=["cam/raw"], threaded=True)
    car.add(Lambda(center_crop), inputs=["cam/raw"], outputs=["cam/image_array"])

def build_vehicle_recording() -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])
    car.add(DynamixelSteering(), inputs=["user/angle"])
    car.add(DynamixelThrottle(), inputs=["user/throttle"])

    car.add(Lambda(lambda t: abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle"], outputs=["recording"])

    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    class RecordCounter:
        def __init__(self):
            self.last_ten = -1
        def run(self, n):
            if n is None:
                return
            ten = n // 10
            if ten != self.last_ten:
                print(f"Recorded samples: {n}")
                self.last_ten = ten

    car.add(RecordCounter(), inputs=["tub/num_records"], outputs=[])

    class PromptWiper:
        def __init__(self, tub, num_records=100):
            self.tub = tub
            self.num = num_records
            self.erased = False

        def run(self, mode):
            if mode == "erase" and not self.erased:
                self.tub.delete_last_n_records(self.num)
                print(f"Deleted last {self.num} records.")
                self.erased = True
            if mode != "erase":
                self.erased = False

    wiper = PromptWiper(tub.tub, num_records=100)
    car.add(wiper, inputs=["user/mode"], outputs=[])
    return car

def build_vehicle_driving(model_path: str) -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    interpreter = KerasInterpreter()
    pilot = KerasLinear(interpreter=interpreter, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)

    car.add(pilot, inputs=["cam/image_array"], outputs=["pilot/angle", "pilot/throttle"])

    car.add(ConsoleDisplay(), inputs=["pilot/angle", "pilot/throttle"], outputs=[])

    car.add(DynamixelSteering(), inputs=["pilot/angle"])
    car.add(DynamixelThrottle(), inputs=["pilot/throttle"])
    return car

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Record data or drive autonomously")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--recording", action="store_true", help="Manual driving & data logging")
    group.add_argument("--driving",   action="store_true", help="Autonomous driving with model")
    parser.add_argument("--model", default=MODEL_PATH_DEFAULT, help=".h5 model path for --driving mode")
    args = parser.parse_args()

    if args.recording:
        vehicle = build_vehicle_recording()
    else:
        mpath = Path(args.model).expanduser()
        if not mpath.is_file():
            sys.exit(f"Model file not found: {mpath}")
        vehicle = build_vehicle_driving(str(mpath))

    def _sigterm(_s, _f):
        print("\nSIGTERM - shutting down...")
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
        sys.exit(0)
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        vehicle.start(rate_hz=DRIVE_LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl-C - shutting down...")
    finally:
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
